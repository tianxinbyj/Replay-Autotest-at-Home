"""
Author: Bu Yujun
Date: 1/7/24
"""
import argparse
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.VersionControl import VersionControl


def main():
    parser = argparse.ArgumentParser(description="power on and off, switch mode, get power status")
    parser.add_argument("-c", "--command", type=str, required=True, help="specify command")
    args = parser.parse_args()

    command = args.command
    versionControl = VersionControl()

    if command == 'on_without_wait':
        res = versionControl.power_ctrl_power_on()
    elif command == 'on_with_wait':
        versionControl.power_ctrl_power_on()
        res = versionControl.check_power_on_success()
    elif command == 'off':
        res = versionControl.power_ctrl_power_off()
    elif command == 'switch2remote':
        res = versionControl.switch_to_remote_mode()
    elif command == 'switch2panel':
        res = versionControl.switch_to_panel_mode()
    elif command == 'get_power':
        res = versionControl.get_power_status()
    else:
        res = False

    print(res)


if __name__ == "__main__":
    main()