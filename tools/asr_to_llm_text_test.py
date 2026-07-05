#!/usr/bin/env python3
import argparse
import asyncio
import base64
import hashlib
import hmac
import json
import os
import subprocess
import sys
import wave
from datetime import datetime
from time import mktime
from urllib.parse import urlencode, urlparse

import websockets
from wsgiref.handlers import format_date_time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from config.config_loader import load_config
from core.providers.llm.ollama.ollama import LLMProvider as OllamaProvider


def record_wav(device: str, duration: int, output_wav: str) -> None:
    os.makedirs(os.path.dirname(output_wav), exist_ok=True)
    cmd = [
        "arecord",
        "-D",
        device,
        "-f",
        "S16_LE",
        "-r",
        "16000",
        "-c",
        "1",
        "-d",
        str(duration),
        output_wav,
    ]
    subprocess.run(cmd, check=True)


def read_pcm_from_wav(wav_path: str) -> bytes:
    with wave.open(wav_path, "rb") as wf:
        ch = wf.getnchannels()
        sw = wf.getsampwidth()
        sr = wf.getframerate()
        if ch != 1 or sw != 2 or sr != 16000:
            raise ValueError(f"WAV格式需为16k/mono/s16，当前为 ch={ch}, sw={sw}, sr={sr}")
        return wf.readframes(wf.getnframes())


def create_xunfei_url(api_url: str, api_key: str, api_secret: str) -> str:
    parsed = urlparse(api_url)
    host = parsed.netloc
    path = parsed.path or "/v2/iat"
    now = datetime.now()
    date = format_date_time(mktime(now.timetuple()))

    signature_origin = f"host: {host}\n"
    signature_origin += f"date: {date}\n"
    signature_origin += f"GET {path} HTTP/1.1"

    signature_sha = hmac.new(
        api_secret.encode("utf-8"),
        signature_origin.encode("utf-8"),
        digestmod=hashlib.sha256,
    ).digest()
    signature_sha = base64.b64encode(signature_sha).decode("utf-8")

    authorization_origin = (
        f'api_key="{api_key}", algorithm="hmac-sha256", '
        f'headers="host date request-line", signature="{signature_sha}"'
    )
    authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
        "utf-8"
    )

    params = {"authorization": authorization, "date": date, "host": host}
    return f"{api_url}?{urlencode(params)}"


async def xunfei_asr_once(asr_cfg: dict, pcm_bytes: bytes) -> str:
    ws_url = create_xunfei_url(
        asr_cfg.get("api_url", "wss://iat-api.xfyun.cn/v2/iat"),
        asr_cfg["api_key"],
        asr_cfg["api_secret"],
    )
    app_id = asr_cfg["app_id"]
    domain = asr_cfg.get("domain", "iat")
    language = asr_cfg.get("language", "zh_cn")
    accent = asr_cfg.get("accent", "mandarin")

    chunk_size = 1280  # 40ms @ 16k, s16, mono
    chunks = [pcm_bytes[i : i + chunk_size] for i in range(0, len(pcm_bytes), chunk_size)]
    if not chunks:
        return ""

    result_text = ""

    async with websockets.connect(
        ws_url,
        max_size=1000000000,
        ping_interval=None,
        ping_timeout=None,
        close_timeout=10,
    ) as ws:
        first_data = {
            "status": 0,
            "format": "audio/L16;rate=16000",
            "audio": base64.b64encode(chunks[0]).decode("utf-8"),
            "encoding": "raw",
        }
        first_frame = {
            "common": {"app_id": app_id},
            "business": {
                "domain": domain,
                "language": language,
                "accent": accent,
                "vad_eos": 1500,
            },
            "data": first_data,
        }
        await ws.send(json.dumps(first_frame, ensure_ascii=False))

        for ch in chunks[1:]:
            frame = {
                "data": {
                    "status": 1,
                    "format": "audio/L16;rate=16000",
                    "audio": base64.b64encode(ch).decode("utf-8"),
                    "encoding": "raw",
                }
            }
            await ws.send(json.dumps(frame, ensure_ascii=False))
            await asyncio.sleep(0.04)

        last_frame = {
            "data": {
                "status": 2,
                "format": "audio/L16;rate=16000",
                "audio": "",
                "encoding": "raw",
            }
        }
        await ws.send(json.dumps(last_frame, ensure_ascii=False))

        while True:
            resp = json.loads(await ws.recv())
            code = resp.get("code", 0)
            if code != 0:
                raise RuntimeError(f"讯飞ASR错误 code={code}, message={resp.get('message', '')}")

            data = resp.get("data", {})
            result = data.get("result", {})
            ws_list = result.get("ws", []) if isinstance(result, dict) else []
            for ws_item in ws_list:
                for cw in ws_item.get("cw", []):
                    result_text += cw.get("w", "")

            if data.get("status") == 2:
                break

    return result_text.strip()


def ollama_reply(config: dict, user_text: str) -> str:
    llm_name = config["selected_module"]["LLM"]
    llm_cfg = config["LLM"][llm_name]
    provider = OllamaProvider(llm_cfg)

    dialogue = [
        {"role": "system", "content": "请用简短中文回答用户问题。"},
        {"role": "user", "content": user_text},
    ]
    parts = []
    for token in provider.response("asr-llm-test", dialogue):
        parts.append(token)
    return "".join(parts).strip()


async def ollama_reply_with_timeout(config: dict, user_text: str, timeout: int) -> str:
    return await asyncio.wait_for(
        asyncio.to_thread(ollama_reply, config, user_text), timeout=timeout
    )


async def main_async(args: argparse.Namespace) -> None:
    cfg = load_config()
    asr_name = cfg["selected_module"]["ASR"]
    asr_cfg = cfg["ASR"][asr_name]

    if asr_cfg.get("type") != "xunfei_stream":
        raise RuntimeError(f"当前ASR不是XunfeiStreamASR，当前为: {asr_name}/{asr_cfg.get('type')}")

    if args.input_text:
        print("[1/4] 跳过录音与ASR，使用 --input-text")
        asr_text = args.input_text.strip()
    else:
        print(f"[1/4] 录音 {args.duration}s -> {args.wav}")
        try:
            record_wav(args.device, args.duration, args.wav)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                "录音失败，可能是麦克风被占用。请先停止桥接/占用进程后重试。"
            ) from e

        print("[2/4] 讯飞ASR识别中...")
        pcm = read_pcm_from_wav(args.wav)
        asr_text = await xunfei_asr_once(asr_cfg, pcm)

    print(f"ASR_TEXT: {asr_text}")

    if not asr_text:
        print("LLM_TEXT: (空，ASR未识别到文本)")
        return

    print("[3/4] Ollama LLM生成中...")
    try:
        llm_text = await ollama_reply_with_timeout(cfg, asr_text, args.llm_timeout)
    except asyncio.TimeoutError:
        print(
            "LLM_TEXT: (超时，Ollama无响应。可先执行: "
            "curl -sS --max-time 30 http://127.0.0.1:11434/api/generate "
            "-d '{\"model\":\"qwen2.5:1.5b\",\"prompt\":\"你好\",\"stream\":false}')"
        )
        return

    print(f"LLM_TEXT: {llm_text}")
    print("[4/4] 完成（本脚本不走TTS）")


def main() -> None:
    parser = argparse.ArgumentParser(description="ASR -> Ollama 文本链路测试（不走TTS）")
    parser.add_argument("--device", default="plughw:0,0", help="ALSA输入设备")
    parser.add_argument("--duration", type=int, default=4, help="录音时长（秒）")
    parser.add_argument("--wav", default="tmp/asr_llm_test.wav", help="录音输出WAV路径")
    parser.add_argument("--llm-timeout", type=int, default=45, help="Ollama超时时间（秒）")
    parser.add_argument(
        "--input-text",
        default="",
        help="可选：跳过录音/ASR，直接把该文本送给Ollama（用于定位LLM问题）",
    )
    args = parser.parse_args()

    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
