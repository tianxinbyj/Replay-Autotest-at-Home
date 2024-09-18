import argparse
import os
import re
import shutil
import socket
import subprocess
import time

import paramiko
from scp import SCPClient
from wcwidth import wcswidth

# 设置目标设备的信息
S32G = ''
USERNAME = 'root'
PASSWORD = ''  # 空密码
PORT = 22
J5A = ''
J5B = ''


def boxed_text(text):
    # Split the text into lines if it's multi-line
    lines = text.split('\n')
    # Determine the length of the longest line
    max_length = max(wcswidth(line) for line in lines)
    # Calculate the box width
    box_width = max_length + 4  # Add padding for spaces and box borders

    # Print the top border of the box
    print("#" * box_width)

    # Print each line with padding and side borders
    for line in lines:
        line_width = wcswidth(line)
        padding = max_length - line_width
        left_padding = padding // 2
        right_padding = padding - left_padding
        centered_line = ' ' * left_padding + line + ' ' * right_padding
        print(f'# {centered_line} #')

    # Print the bottom border of the box
    print("#" * box_width)


def create_symlink(source_folder, symlink_target):
    # 给install文件夹创建软连接
    # 如果目标路径已经存在，删除它
    if os.path.islink(symlink_target):
        os.unlink(symlink_target)  # 如果是符号链接，直接删除
    elif os.path.exists(symlink_target):
        if os.path.isdir(symlink_target):
            shutil.rmtree(symlink_target)  # 如果是文件夹，递归删除目录中的所有内容
        else:
            os.remove(symlink_target)  # 如果是文件，删除文件

    # 创建新的符号链接
    try:
        os.symlink(source_folder, symlink_target)
        print(f"{symlink_target}已更新")
    except OSError as e:
        print(f"创建符号链接时出错：{e}")


def check_device_connection(hostname: str) -> bool:
    response = os.system("ping -c 1 " + hostname)
    if response != 0:
        print("无法ping通目标设备，请检查网络连接")

        # 切换网络到172.31.131.66
        ubuntu_password = 'admin'
        command = 'sudo -S ifconfig eth0 172.31.131.66 netmask 255.255.255.0'
        os.system('echo %s | %s' % (ubuntu_password, command))

        command = 'sudo -S service network-manager restart'
        os.system('echo %s | %s' % (ubuntu_password, command))
        return False
    else:
        return True


def check_version(ip_address, timeout=10) -> str:
    # 检查设备当前版本号
    software_version = ''
    client = None  # 初始化 client 变量
    try:
        # 创建 SSH 客户端
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(ip_address, PORT, USERNAME, PASSWORD, timeout=timeout)

        # Execute the command to get the software version
        stdin, stdout, stderr = client.exec_command("cat /system/build.prop")
        output = stdout.read().decode('utf-8')

        # Extract the software version from the output
        pattern = r"ro.build.version.release=(.+)"
        match = re.search(pattern, output)
        if match:
            software_version = match.group(1)
        else:
            print('没有版本号！')

    except paramiko.AuthenticationException:
        print("认证失败，请检查用户名和密码")
    except paramiko.SSHException as e:
        print(f"SSH 连接错误: {e}")
    except socket.timeout:
        print(f"Connection timed out after {timeout} seconds.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if client is not None:
            client.close()  # 确保在发生异常时也能够关闭SSH连接

    return software_version


def check_existing_files(artifact_path, targets):
    folder_paths = {}
    missing_targets = []
    existing_folders = [folder for folder in os.listdir(artifact_path) if
                        os.path.isdir(os.path.join(artifact_path, folder))]

    for target in targets:
        temp = target
        if target == "mcu" or target == "mpu":
            target = "s32g"  # MCU和MPU都是用s32g文件夹
        found_folder = next((folder for folder in existing_folders if target in folder), None)
        if not found_folder:
            print(f"Error: Target '{temp}' not found in '{artifact_path}'.")
            missing_targets.append(temp)
            continue
        folder_paths[target] = os.path.join(artifact_path, found_folder)

    return folder_paths, missing_targets


def upload_files(client, local_path, remote_path):
    with SCPClient(client.get_transport()) as scp:
        scp.put(local_path, remote_path)


# 递归查找目标文件夹，返回目标文件夹的路径
def find_target_dir(directory, target: str):
    for root, dirs, files in os.walk(directory):
        if target in dirs:
            return os.path.join(root, target)


def flash_mcu(artifact_path):
    # ================================OTA刷MCU================================
    # if not check_device_connection(S32G):
    #     print("无法ping通目标设备，请检查网络连接")
    boxed_text(f"当前MCU版本为 {check_version(S32G)} \n开始刷MCU")

    # 文件路径
    remote_path = '/tmp/'
    fota_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Resources',
                                  'Fota_Update_Program')
    norflash_file_path = find_target_dir(artifact_path, "image")

    # 创建 SSH 客户端
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(S32G, PORT, USERNAME, PASSWORD)

    # 使用 SCP 将 fota 文件夹中的所有文件上传到远程服务器
    for file in os.listdir(fota_file_path):
        upload_files(client, os.path.join(fota_file_path, file), os.path.join(remote_path, file))

    # 使用 SCP 将 norflash 文件夹中的所有文件上传到远程服务器
    for file in os.listdir(norflash_file_path):
        upload_files(client, os.path.join(norflash_file_path, file), os.path.join(remote_path, file))

    # 使用 SSH 执行 chmod 命令
    commands = [
        'mount -o remount,exec /tmp',
        'chmod 755 /tmp/get_test',
        'chmod 755 /tmp/set_test',
        'chmod 755 /tmp/peri_update_hal_test',
        'chmod 755 /tmp/update_all_reset.sh',
        'chmod 755 /tmp/update_all_no_reset.sh',
        'chmod 755 /tmp/ua_test',
        'chmod 755 /tmp/ua_test_all',
        'cd /tmp && export LD_LIBRARY_PATH=/app/lib64:${LD_LIBRARY_PATH} && ./update_all_reset.sh'
    ]
    for command in commands:
        stdin, stdout, stderr = client.exec_command(command)
        print(stdout.read().decode())
        print(stderr.read().decode())

    # 关闭连接
    client.close()

    boxed_text("MCU刷写完成")  # Note: 刷写完成后机器自动重启，无需手动掉电上电。


def flash_mpu(artifact_path):
    boxed_text(f"当前MPU版本为 {check_version(S32G)} \n开始刷MPU")
    target_path = find_target_dir(artifact_path, "finalrelease")

    command = f"./flash_s32g3_non_cur_plane.sh . --all {S32G}"

    # 使用subprocess在指定路径执行命令
    subprocess.run(f"cd {target_path} && {command}", shell=True, check=True)
    boxed_text("MPU刷写完成")


def flash_j5(artifact_path, core='A'):
    if core not in ['A', 'B']:
        raise ValueError("Invalid value for core. It can only be 'A' or 'B'.")
    if core == 'A':  # J5A
        ip = J5A
    else:  # J5B
        ip = J5B

    if not check_device_connection(ip):
        print("无法ping通目标设备，请检查网络连接")

    boxed_text(f"当前J5{core}版本为 {check_version(ip)} \n开始刷J5{core}")

    # 使用subprocess在指定路径执行命令
    target_path = find_target_dir(artifact_path, "packages")
    command = f"./j5_update.sh . --all {ip}"
    subprocess.run(f"cd {target_path} && {command}", shell=True, check=True)
    boxed_text(f"J5{core}刷写完成")


TARGETS_MAPPING = {
    'j5a': '1_journey5',
    'j5b': '2_journey5',
}


def parse_arguments():
    global S32G, J5A, J5B  # 为了在函数内部修改全局变量
    parser = argparse.ArgumentParser(description="Auto flash helper.")
    parser.add_argument("-a", "--artifact_path", required=True, help="Specify the artifact path.")
    parser.add_argument("-t", "--targets", required=True, nargs='+',
                        choices=['j5a', 'j5b', 'switch', 'mcu', 'mpu'],
                        help="Specify one or more targets by name. Options: {j5a, j5b, switch, mcu, mpu}")
    parser.add_argument("--type", default='t1',
                        help="Specify the type of the connection. Options: {t1, lan}")
    args = parser.parse_args()

    if args.type == 't1':  # 根据不同的连接类型，处理连接的网段
        S32G = '172.31.131.34'
        J5A = '172.31.131.35'
        J5B = '172.31.131.36'
    elif args.type == 'lan':
        S32G = '172.31.137.34'
        J5A = '172.31.137.50'
        J5B = '172.31.137.51'

    return args


def main():
    args = parse_arguments()
    # Convert to folder names if target is in mapping, else use original name
    if args.targets:
        args.targets = [TARGETS_MAPPING.get(target, target) for target in args.targets]

    artifact_path = args.artifact_path

    if not os.path.exists(artifact_path) or not os.path.isdir(artifact_path):
        print(f"Error: The path '{artifact_path}' does not exist or is not a directory.")
        exit(1)

    # 检查文件夹中是否存在需要的文件, 额外检查install文件夹
    folder_paths, missing_targets = check_existing_files(artifact_path, args.targets + ["install"])
    if len(missing_targets) > 0:
        print(f"Error: The following targets are missing from the artifact folder: {missing_targets}")
        exit(1)

    # 创建install文件夹的软链接，确保路径 $HOME/artifacts/install一定是最近刷的版本
    create_symlink(folder_paths["install"], os.path.join(os.path.expanduser("~"), "artifacts", "install"))

    if args.targets:
        # "mcu" target should be flashed first
        # 检查并移除所有 "mcu"
        mcu_found = "mcu" in args.targets
        while "mcu" in args.targets:
            args.targets.remove("mcu")

        # 检查并移除所有 "mpu"
        mpu_found = "mpu" in args.targets
        while "mpu" in args.targets:
            args.targets.remove("mpu")

        # 如果之前列表中有 "mcu"，则插入一个 "mcu"
        if mcu_found:
            args.targets.insert(0, "mcu")

        # 如果之前列表中有 "mpu"，并且 "mcu" 已经被插入，则插入一个 "mpu"
        if mpu_found and "mcu" in args.targets:
            args.targets.insert(1, "mpu")

        for target in args.targets:
            if target == "1_journey5":
                flash_j5(folder_paths["1_journey5"], core='A')
            elif target == "2_journey5":
                flash_j5(folder_paths["2_journey5"], core='B')
            elif target == "mcu":
                flash_mcu(folder_paths["s32g"])
            elif target == "mpu":
                flash_mpu(folder_paths["s32g"])
            elif target == "switch":
                print("Switch刷写, not implemented...")
            if target != args.targets[-1]:
                boxed_text(f"{target}刷写完成，等待10秒")
                time.sleep(10)

    boxed_text(f"版本刷写已完成\n请重启电源!")


if __name__ == '__main__':
    main()
