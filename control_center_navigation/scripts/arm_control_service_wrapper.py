#!/usr/bin/env python3
import autolife_robot_arm as robot_arm
import autolife_robot_arm.main as arm_main


def main() -> None:
    control_settings = robot_arm.PROGRAM_SETTINGS['app_settings']['robot_control_settings']
    control_settings['vr_gv_control_enabled'] = True
    arm_main.main()


if __name__ == '__main__':
    main()
