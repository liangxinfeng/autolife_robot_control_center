import json

from text_voice_control.command_parser import parse_voice_command, strip_asr_prefix


def test_plain_text_becomes_play_command():
    command = parse_voice_command("你好")
    assert command.status == "play"
    assert command.text == "你好"
    assert json.loads(command.to_tts_payload()) == {"status": "play", "text": "你好"}


def test_stop_words_become_stop_command():
    command = parse_voice_command("停止播报")
    assert command.status == "stop"
    assert json.loads(command.to_tts_payload()) == {"status": "stop"}


def test_json_play_command_is_normalized():
    command = parse_voice_command('{"status":"play","text":"欢迎"}')
    assert command.status == "play"
    assert command.text == "欢迎"


def test_asr_prefix_extraction():
    assert strip_asr_prefix("播报：欢迎光临", ["播报"]) == "欢迎光临"
    assert strip_asr_prefix("今天天气不错", ["播报"]) is None
