import argparse
import sys

from Libs import get_project_path
sys.path.append(get_project_path())
from Envs.ReplayClient.Modules.VersionControl import VersionControl


def main():
    parser = argparse.ArgumentParser(description="Change Camera Config")
    parser.add_argument("-e", "--ecu_type", default='ES37',type=str,required=False,help="ECU TYPE")
    args = parser.parse_args()

    if args.ecu_type not in ['ES37', 'J6E', 'J6', '1J5']:
        raise Exception("Invalid ECU Type, ECU tpye must in ('ES37', 'J6E', 'J6', '1J5')")
    VC = VersionControl(args.ecu_type)

    res = VC.flash_camera_config_in_one()
    if res:
        print("flash camera config success")
    else:
        print(0)

if __name__ == "__main__":
    main()

