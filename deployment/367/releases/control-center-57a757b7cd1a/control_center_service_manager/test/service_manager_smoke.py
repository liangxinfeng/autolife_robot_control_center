#!/usr/bin/env python3
"""Command-safety, parser and offscreen rendering smoke test."""

import importlib.util
from pathlib import Path
import sys

from PyQt5 import QtWidgets


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'control_center_service_manager'
    / 'panel.py'
)
spec = importlib.util.spec_from_file_location('service_manager_panel', SOURCE)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def main():
    program, arguments = module.command_for(
        'restart', 'vision-service.service'
    )
    assert program == 'systemctl'
    assert arguments == [
        '--user', 'restart', 'vision-service.service'
    ]
    program, arguments = module.command_for(
        'journal-follow', 'arm-control-service.service'
    )
    assert program == 'journalctl'
    assert '-f' in arguments
    try:
        module.command_for('start', 'bad.service;touch /tmp/injected')
        raise AssertionError('unsafe service name was accepted')
    except ValueError:
        pass
    print('PASS: commands use argv and reject shell injection')

    state = module.parse_show_output(
        'ActiveState=active\nSubState=running\n'
        'UnitFileState=enabled\nDescription=Vision core\n'
    )
    assert state['active'] == 'active'
    assert state['enabled'] == 'enabled'
    unit_files = module.parse_unit_files(
        'vision-service.service enabled enabled\n'
        'demo.service disabled enabled\n'
    )
    running = module.parse_running_units(
        'vision-service.service loaded active running Vision core service\n'
    )
    assert unit_files['vision-service.service'] == 'enabled'
    assert running['vision-service.service'] == 'Vision core service'
    print('PASS: service state, running list and autostart list parsing')

    relay_config = (
        'port: 9000\n'
        'ip: "192.168.50.138"  # robot address\n'
        'enabled: true\n'
    )
    assert module.parse_relay_ip(relay_config) == '192.168.50.138'
    updated, address = module.update_relay_ip(
        relay_config, '192.168.50.144'
    )
    assert address == '192.168.50.144'
    assert module.parse_relay_ip(updated) == '192.168.50.144'
    assert 'port: 9000' in updated and 'enabled: true' in updated
    try:
        module.update_relay_ip(relay_config, '192.168.50.999')
        raise AssertionError('invalid IPv4 address was accepted')
    except ValueError:
        pass
    print('PASS: relay IP parsing, validation and format-preserving update')

    vision = 'ai_chatbot_enabled = false\ntts_enabled = false\n'
    navigation = '    voice_feedback_enabled: false\n'
    vision = module.update_boolean_setting(
        vision, module.TTS_ENABLED_RE, 'tts_enabled', True
    )
    navigation = module.update_boolean_setting(
        navigation,
        module.NAVIGATION_VOICE_ENABLED_RE,
        'voice_feedback_enabled',
        True,
    )
    assert module.parse_boolean_setting(
        vision, module.TTS_ENABLED_RE, 'tts_enabled'
    ) is True
    assert module.parse_boolean_setting(
        navigation,
        module.NAVIGATION_VOICE_ENABLED_RE,
        'voice_feedback_enabled',
    ) is True
    print('PASS: navigation voice status parsing and format-preserving update')

    vision_config = '{\n    "speaker_volume": 70,\n    "speaker_volume_factor": 5\n}\n'
    assert module.parse_speaker_volume(vision_config) == 70
    updated_volume = module.update_speaker_volume(vision_config, 85)
    assert module.parse_speaker_volume(updated_volume) == 85
    assert '"speaker_volume_factor": 5' in updated_volume
    for invalid in (-1, 101):
        try:
            module.update_speaker_volume(vision_config, invalid)
            raise AssertionError('invalid speaker volume was accepted')
        except ValueError:
            pass
    print('PASS: speaker volume parsing, bounds and format-preserving update')

    cards = (
        ' 0 [D4K            ]: USB-Audio - Camera\n'
        ' 4 [PCH            ]: HDA-Intel - HDA Intel PCH\n'
    )
    assert module.parse_pch_card_index(cards) == 4
    program, arguments, card = module.speaker_mixer_command(85, cards)
    assert program == sys.executable and card == 4
    assert arguments[0].endswith('speaker_volume.py')
    assert arguments[1:] == ['--card','4','--volume','85']
    _, zero_arguments, _ = module.speaker_mixer_command(0, cards)
    assert zero_arguments[-1] == '0'
    print('PASS: physical PCH speaker mixer command is safe and card-index independent')

    application = (
        QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    )
    panel = module.ServiceManagerPanel(auto_refresh=False)
    panel.resize(1440, 900)
    panel.show()
    application.processEvents()
    assert len(panel._core_cards) == 4
    assert panel.service_table.columnCount() == 4
    assert panel.log_service.count() == 4
    assert panel.joint_reset_button.text() == '全身关节复位'
    assert panel.relay_ip_save_button.text() == '保存并应用'
    assert panel.relay_ip_edit.placeholderText()
    assert panel.voice_feedback_on_button.text() == '开启播报'
    assert panel.voice_feedback_off_button.text() == '关闭播报'
    assert panel.voice_volume_slider.minimum() == 0
    assert panel.voice_volume_slider.maximum() == 100
    assert panel.voice_volume_apply_button.text() == '应用并试听'
    assert panel.logo_display_start_button.text() == '开启 Logo 网页'
    assert panel.logo_display_stop_button.text() == '关闭 Logo 网页'
    assert module.LOGO_SERVICE_UNIT == 'autolife-logo-display.service'
    assert f'/control_reset_{panel.topic_suffix}' in panel.joint_reset_button.toolTip()
    for card in panel._core_cards.values():
        card.apply_state({
            'active': 'active', 'sub': 'running', 'enabled': 'enabled'
        })
        assert '运行中' in card.state_label.text()
        assert card.enabled_label.text() == '自启：已启用'
    observed = []
    panel.service_state_changed.connect(
        lambda unit, active, enabled: observed.append(
            (unit, active, enabled)
        )
    )
    unit = 'gv-slam-service.service'
    panel._core_state_received(
        unit,
        panel._core_cards[unit],
        True,
        'ActiveState=active\nSubState=running\nUnitFileState=enabled\n',
        0,
    )
    assert observed[-1] == (unit, 'active', 'enabled')
    print('PASS: core service state changes are exposed to the integrated shell')
    output = Path('/tmp/control_center_service_manager_smoke.png')
    assert panel.grab().save(str(output))
    print(f'PASS: lightweight service manager rendering: {output}')
    panel.shutdown()
    panel.close()
    application.processEvents()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
