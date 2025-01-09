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
    try:
        versionControl = VersionControl()
    except Exception as err:
        print(False)
        return

    if command == 'on':
        res = versionControl.power_ctrl_power_on()
    elif command == 'on_with_waiting':
        versionControl.power_ctrl_power_on()
        res = versionControl.check_power_on_success()
    elif command == 'off':
        res = versionControl.power_ctrl_power_off()
    elif command == 'power':
        res = versionControl.power_ctrl_get_status()
    elif command == 'remote':
        res = versionControl.switch_to_remote_mode()
    elif command == 'panel':
        res = versionControl.switch_to_panel_mode()

    else:
        res = False

    print(res)


if __name__ == "__main__":
    main()