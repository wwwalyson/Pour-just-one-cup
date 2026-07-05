#!/usr/bin/env python3
"""RDKX5 本机麦克风/扬声器桥接器。

这个脚本把 ALSA 的原始 PCM 音频接到小智服务器的 websocket 协议上，
并把服务器返回的 Opus 音频解码后播放到本地扬声器。

使用方式示例：
  python tools/rdkx5_audio_bridge.py \
    --url ws://127.0.0.1:8000/xiaozhi/v1/ \
    --device-id rdkx5-demo \
    --client-id rdkx5-local-audio \
    --device-name RDKX5 \
    --speaker-device default

如果你的麦克风或扬声器不是当前默认的 USB 声卡，把 `--input-device` 和 `--speaker-device`
改成 `arecord -l` / `aplay -l` 看到的 ALSA 设备名即可。

当前这台 RDKX5 上默认使用 `plughw:0,0` 采集输入，`default` 播放输出；
两边仍然都指向同一块 `MCP01 / USB Audio`。
"""

from __future__ import annotations

import argparse
import asyncio
import audioop
import contextlib
import json
import os
import shutil
import signal
import subprocess
from dataclasses import dataclass
from typing import Optional

import opuslib_next
import websockets


SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2
FRAME_MS = 60
FRAME_BYTES = SAMPLE_RATE * FRAME_MS // 1000 * SAMPLE_WIDTH


@dataclass
class AudioDeviceConfig:
    input_device: Optional[str]
    speaker_device: Optional[str]


class RDKX5AudioBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.url = args.url
        self.device_id = args.device_id
        self.client_id = args.client_id
        self.device_name = args.device_name
        self.device_mac = args.device_mac or args.device_id
        self.token = args.token
        self.auth_header = args.auth_header
        self.vad_threshold = args.vad_threshold
        self.silence_ms = args.silence_ms
        self.min_record_ms = args.min_record_ms
        self.speech_start_frames = max(1, args.speech_start_frames)
        self.enable_barge_in = args.enable_barge_in
        self.device = AudioDeviceConfig(args.input_device, args.speaker_device)

        self.decoder = opuslib_next.Decoder(SAMPLE_RATE, CHANNELS)
        self.encoder = opuslib_next.Encoder(
            SAMPLE_RATE,
            CHANNELS,
            opuslib_next.APPLICATION_AUDIO,
        )
        self.shutdown_event = asyncio.Event()
        self.hello_ready = asyncio.Event()
        self.remote_speaking = False
        self.current_tts_session_id: Optional[str] = None
        self.playback_queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=256)

        self.recording = False
        self.aborted_for_barge_in = False
        self.voice_frames = 0
        self.silence_frames = 0
        self.silence_frames_needed = max(1, self.silence_ms // FRAME_MS)
        self.min_record_frames_needed = max(1, self.min_record_ms // FRAME_MS)
        self.recorded_frames = 0

        self.websocket = None
        self.recorder_proc: Optional[asyncio.subprocess.Process] = None
        self.player_proc: Optional[asyncio.subprocess.Process] = None
        self.stop_requested = False

    async def run(self) -> None:
        if shutil.which("arecord") is None:
            raise RuntimeError("未找到 arecord，请先安装 alsa-utils")
        if shutil.which("aplay") is None:
            raise RuntimeError("未找到 aplay，请先安装 alsa-utils")

        while not self.stop_requested:
            self.shutdown_event.clear()
            try:
                await self._run_once()
            except (asyncio.CancelledError, KeyboardInterrupt):
                self.stop_requested = True
                self.shutdown_event.set()
                raise
            except Exception as exc:
                print(f"[bridge] 连接异常: {exc}")
            if not self.stop_requested:
                await asyncio.sleep(2)

    async def _run_once(self) -> None:
        headers = {
            "device-id": self.device_id,
            "client-id": self.client_id,
        }
        if self.token:
            headers["authorization"] = self.token if self.token.startswith("Bearer ") else f"Bearer {self.token}"
        if self.auth_header:
            headers["authorization"] = self.auth_header

        async with websockets.connect(
            self.url,
            additional_headers=headers,
            max_size=None,
            ping_interval=20,
            ping_timeout=20,
        ) as websocket:
            self.websocket = websocket
            self.hello_ready.clear()
            self.remote_speaking = False
            self.current_tts_session_id = None
            self.recording = False
            self.aborted_for_barge_in = False
            self.voice_frames = 0
            self.silence_frames = 0
            self.recorded_frames = 0

            await self._send_hello()

            reader_task = asyncio.create_task(self._websocket_reader())
            player_task = asyncio.create_task(self._playback_worker())

            hello_wait_task = asyncio.create_task(self.hello_ready.wait())
            done, pending = await asyncio.wait(
                {reader_task, hello_wait_task},
                return_when=asyncio.FIRST_COMPLETED,
            )
            if reader_task in done and not self.hello_ready.is_set():
                raise RuntimeError("WebSocket 在握手完成前断开")
            hello_wait_task.cancel()

            self.recorder_proc = await self._start_recorder()
            try:
                await self._capture_worker()
            finally:
                self.shutdown_event.set()
                reader_task.cancel()
                player_task.cancel()
                await self._stop_process(self.recorder_proc)
                await self._stop_process(self.player_proc)
                self.recorder_proc = None
                self.player_proc = None
                self.websocket = None

    async def _send_hello(self) -> None:
        assert self.websocket is not None
        hello = {
            "type": "hello",
            "device_id": self.device_id,
            "device_name": self.device_name,
            "device_mac": self.device_mac,
            "token": self.token or "",
            "features": {"mcp": False},
            "audio_params": {
                "format": "opus",
                "sample_rate": SAMPLE_RATE,
                "channels": CHANNELS,
                "frame_duration": FRAME_MS,
            },
        }
        await self.websocket.send(json.dumps(hello, ensure_ascii=False))

    async def _websocket_reader(self) -> None:
        assert self.websocket is not None
        try:
            async for message in self.websocket:
                if isinstance(message, bytes):
                    await self._enqueue_audio(message)
                    continue

                try:
                    payload = json.loads(message)
                except json.JSONDecodeError:
                    print(f"[bridge] 收到非 JSON 文本: {message}")
                    continue

                msg_type = payload.get("type")
                if msg_type == "hello":
                    self.hello_ready.set()
                    print(f"[bridge] 服务器握手成功，session_id={payload.get('session_id')}")
                elif msg_type == "tts":
                    state = payload.get("state")
                    if state:
                        print(f"[tts] {state}")
                    await self._handle_tts_message(payload)
                elif msg_type == "stt":
                    text = payload.get("text", "")
                    if text:
                        print(f"[asr] {text}")
                elif msg_type == "llm":
                    text = payload.get("text", "")
                    if text:
                        print(f"[llm] {text}")
                else:
                    print(f"[bridge] 收到消息: {payload}")
        finally:
            self.shutdown_event.set()

    async def _handle_tts_message(self, payload: dict) -> None:
        state = payload.get("state")
        if state in {"start", "sentence_start"}:
            self.remote_speaking = True
            self.current_tts_session_id = payload.get("session_id")
            self.aborted_for_barge_in = False
        elif state == "stop":
            self.remote_speaking = False
            self.current_tts_session_id = None

    async def _enqueue_audio(self, packet: bytes) -> None:
        try:
            self.playback_queue.put_nowait(packet)
        except asyncio.QueueFull:
            print("[bridge] 播放队列已满，丢弃一帧音频")

    async def _playback_worker(self) -> None:
        self.player_proc = await self._start_player()
        assert self.player_proc is not None and self.player_proc.stdin is not None
        writer = self.player_proc.stdin

        while not self.shutdown_event.is_set():
            try:
                packet = await asyncio.wait_for(self.playback_queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            try:
                pcm = self.decoder.decode(packet, int(SAMPLE_RATE * FRAME_MS / 1000))
                writer.write(pcm)
                await writer.drain()
            except Exception as exc:
                print(f"[bridge] 播放解码失败: {exc}")

    async def _capture_worker(self) -> None:
        assert self.recorder_proc is not None and self.recorder_proc.stdout is not None
        reader = self.recorder_proc.stdout
        pcm_buffer = bytearray()
        while not self.shutdown_event.is_set():
            try:
                chunk = await asyncio.wait_for(
                    reader.read(FRAME_BYTES),
                    timeout=1.0,
                )
            except asyncio.TimeoutError:
                continue
            if not chunk:
                self.shutdown_event.set()
                break

            pcm_buffer.extend(chunk)
            while len(pcm_buffer) >= FRAME_BYTES:
                frame = bytes(pcm_buffer[:FRAME_BYTES])
                del pcm_buffer[:FRAME_BYTES]

                rms = audioop.rms(frame, SAMPLE_WIDTH)
                voice = rms >= self.vad_threshold

                if (
                    self.enable_barge_in
                    and self.remote_speaking
                    and voice
                    and not self.aborted_for_barge_in
                ):
                    await self._send_abort()
                    self.remote_speaking = False
                    self.aborted_for_barge_in = True

                if self.remote_speaking:
                    continue

                if voice:
                    self.silence_frames = 0
                    self.voice_frames += 1
                    if not self.recording and self.voice_frames >= self.speech_start_frames:
                        await self._send_listen_start()
                        self.recording = True
                        self.recorded_frames = 0
                    if self.recording:
                        self.recorded_frames += 1
                        await self._send_audio(frame)
                else:
                    self.voice_frames = 0
                    if self.recording:
                        self.recorded_frames += 1
                        self.silence_frames += 1
                        await self._send_audio(frame)
                        if (
                            self.silence_frames >= self.silence_frames_needed
                            and self.recorded_frames >= self.min_record_frames_needed
                        ):
                            await self._send_listen_stop()
                            self.recording = False
                            self.silence_frames = 0
                            self.recorded_frames = 0

    async def _send_audio(self, chunk: bytes) -> None:
        if self.websocket is None:
            return
        opus_packet = self.encoder.encode(chunk, int(SAMPLE_RATE * FRAME_MS / 1000))
        await self.websocket.send(opus_packet)

    async def _send_listen_start(self) -> None:
        if self.websocket is None:
            return
        message = {"type": "listen", "state": "start", "mode": "auto"}
        await self.websocket.send(json.dumps(message, ensure_ascii=False))

    async def _send_listen_stop(self) -> None:
        if self.websocket is None:
            return
        message = {"type": "listen", "state": "stop"}
        await self.websocket.send(json.dumps(message, ensure_ascii=False))

    async def _send_abort(self) -> None:
        if self.websocket is None or not self.current_tts_session_id:
            return
        message = {
            "type": "abort",
            "session_id": self.current_tts_session_id,
            "reason": "wake_word_detected",
        }
        await self.websocket.send(json.dumps(message, ensure_ascii=False))

    async def _start_recorder(self) -> asyncio.subprocess.Process:
        cmd = [
            "arecord",
            "-q",
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-r",
            str(SAMPLE_RATE),
            "-c",
            str(CHANNELS),
        ]
        if self.device.input_device:
            cmd.extend(["-D", self.device.input_device])
        return await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )

    async def _start_player(self) -> asyncio.subprocess.Process:
        cmd = [
            "aplay",
            "-q",
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-r",
            str(SAMPLE_RATE),
            "-c",
            str(CHANNELS),
        ]
        if self.device.speaker_device:
            cmd.extend(["-D", self.device.speaker_device])
        return await asyncio.create_subprocess_exec(
            *cmd,
            stdin=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )

    async def _stop_process(self, proc: Optional[asyncio.subprocess.Process]) -> None:
        if proc is None:
            return
        with contextlib.suppress(Exception):
            proc.terminate()
        with contextlib.suppress(Exception):
            await asyncio.wait_for(proc.wait(), timeout=2)
        with contextlib.suppress(Exception):
            if proc.returncode is None:
                proc.kill()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDKX5 小智音频桥接器")
    parser.add_argument(
        "--url",
        default="ws://127.0.0.1:8000/xiaozhi/v1/",
        help="小智服务器 websocket 地址",
    )
    parser.add_argument("--device-id", default="rdkx5-demo", help="设备 ID")
    parser.add_argument("--client-id", default="rdkx5-local-audio", help="客户端 ID")
    parser.add_argument("--device-name", default="RDKX5", help="设备名称")
    parser.add_argument(
        "--device-mac",
        default="",
        help="设备 MAC，默认复用 device-id",
    )
    parser.add_argument("--token", default="", help="认证 token")
    parser.add_argument(
        "--auth-header",
        default="",
        help="直接覆盖 authorization 头，优先级高于 token",
    )
    parser.add_argument(
        "--input-device",
        default="plughw:0,0",
        help="ALSA 输入设备，例如 plughw:1,0",
    )
    parser.add_argument(
        "--speaker-device",
        default="default",
        help="ALSA 输出设备，例如 default 或 plughw:0,0",
    )
    parser.add_argument(
        "--vad-threshold",
        type=int,
        default=500,
        help="本地静音检测阈值，越大越不敏感",
    )
    parser.add_argument(
        "--silence-ms",
        type=int,
        default=800,
        help="静音多久后结束一次发言",
    )
    parser.add_argument(
        "--min-record-ms",
        type=int,
        default=900,
        help="一次发言最短持续时长，避免噪声导致快速开始/停止",
    )
    parser.add_argument(
        "--speech-start-frames",
        type=int,
        default=1,
        help="检测到多少帧语音后开始发送",
    )
    parser.add_argument(
        "--enable-barge-in",
        action="store_true",
        help="开启后可通过说话打断正在播放的回复（默认关闭，避免本机回声误触发）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = RDKX5AudioBridge(args)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    for signame in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(signame, lambda: setattr(bridge, "stop_requested", True) or bridge.shutdown_event.set())

    try:
        loop.run_until_complete(bridge.run())
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


if __name__ == "__main__":
    main()