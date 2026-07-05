#!/usr/bin/env python3
import argparse
import asyncio
import audioop
import os
import sys
import subprocess
import wave

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from core.providers.asr.fun_local import ASRProvider


def analyze_wav(wav_path: str) -> dict:
    with wave.open(wav_path, "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        total_frames = wf.getnframes()
        pcm = wf.readframes(total_frames)

    frame_ms = 20
    frame_bytes = int(sample_rate * frame_ms / 1000) * sample_width * channels
    frames = [
        pcm[i : i + frame_bytes]
        for i in range(0, len(pcm), frame_bytes)
        if len(pcm[i : i + frame_bytes]) == frame_bytes
    ]
    rms_values = [audioop.rms(frame, sample_width) for frame in frames] if frames else [0]
    silence_ratio = sum(1 for x in rms_values if x < 200) / len(rms_values)

    return {
        "sample_rate": sample_rate,
        "channels": channels,
        "sample_width": sample_width,
        "duration_s": round(total_frames / sample_rate, 2) if sample_rate else 0,
        "rms_max": max(rms_values),
        "rms_avg": round(sum(rms_values) / len(rms_values), 1),
        "silence_ratio": round(silence_ratio, 3),
    }


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


async def run_asr(wav_path: str, model_dir: str, output_dir: str) -> str:
    with wave.open(wav_path, "rb") as wf:
        pcm = wf.readframes(wf.getnframes())

    provider = ASRProvider(
        {"model_dir": model_dir, "output_dir": output_dir}, delete_audio_file=True
    )
    text, _ = await provider.speech_to_text_wrapper(
        [pcm], session_id="asr_only_test", audio_format="pcm"
    )
    return text or ""


def main() -> None:
    parser = argparse.ArgumentParser(description="ASR isolated test (record + transcribe)")
    parser.add_argument("--device", default="plughw:0,0", help="ALSA capture device")
    parser.add_argument("--duration", type=int, default=5, help="Record seconds")
    parser.add_argument("--wav", default="tmp/asr_only_test.wav", help="WAV path")
    parser.add_argument(
        "--model-dir", default="models/SenseVoiceSmall", help="FunASR model directory"
    )
    parser.add_argument("--output-dir", default="tmp/", help="ASR output directory")
    parser.add_argument(
        "--skip-record",
        action="store_true",
        help="Skip recording and transcribe existing wav",
    )
    args = parser.parse_args()

    if not args.skip_record:
        print(f"[1/3] Recording {args.duration}s from {args.device} -> {args.wav}")
        record_wav(args.device, args.duration, args.wav)

    if not os.path.exists(args.wav):
        raise FileNotFoundError(f"WAV file not found: {args.wav}")

    stats = analyze_wav(args.wav)
    print(f"[2/3] Audio stats: {stats}")
    if stats["rms_max"] < 500 or stats["silence_ratio"] > 0.95:
        print("[WARN] Input appears too quiet; please check mic device/gain and speak closer.")

    print("[3/3] Running ASR...")
    text = asyncio.run(run_asr(args.wav, args.model_dir, args.output_dir))
    print(f"ASR_TEXT: {text}")


if __name__ == "__main__":
    main()
