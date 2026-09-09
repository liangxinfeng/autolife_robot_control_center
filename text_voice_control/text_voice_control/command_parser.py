import json
from dataclasses import dataclass
from typing import Optional


STOP_WORDS = {"stop", "/stop", "停止", "停下", "停止播报", "停止语音"}


@dataclass(frozen=True)
class VoiceCommand:
    status: str
    text: str = ""

    def to_tts_payload(self, use_json: bool = True) -> str:
        if self.status == "stop":
            return json.dumps({"status": "stop"}, ensure_ascii=False)
        if use_json:
            return json.dumps({"status": "play", "text": self.text}, ensure_ascii=False)
        return self.text


def parse_voice_command(raw: str) -> Optional[VoiceCommand]:
    text = (raw or "").strip()
    if not text:
        return None

    if text.lower() in STOP_WORDS or text in STOP_WORDS:
        return VoiceCommand(status="stop")

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return VoiceCommand(status="play", text=text)

    if not isinstance(data, dict):
        return VoiceCommand(status="play", text=text)

    status = str(data.get("status", "play")).strip().lower()
    if status == "stop":
        return VoiceCommand(status="stop")

    speak_text = str(data.get("text", "")).strip()
    if not speak_text:
        return None
    return VoiceCommand(status="play", text=speak_text)


def strip_asr_prefix(text: str, prefixes: list[str]) -> Optional[str]:
    value = (text or "").strip()
    if not value:
        return None

    for prefix in prefixes:
        clean_prefix = prefix.strip()
        if clean_prefix and value.startswith(clean_prefix):
            return value[len(clean_prefix):].strip(" ：:，,")
    return None
