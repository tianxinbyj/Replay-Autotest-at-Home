import argparse
import sys

from Libs import get_project_path
sys.path.append(get_project_path())

from Envs.Master.Tools.DataReplayTest import DataReplayTest


def main():
    parser = argparse.ArgumentParser(description="start replay test")
    parser.add_argument("-t", "--task_folder", type=str, required=True, help="task folder")

    args = parser.parse_args()
    task_folder = args.task_folder
    ddd = DataReplayTest(task_folder)
    ddd.start()


if __name__ == '__main__':
    main()