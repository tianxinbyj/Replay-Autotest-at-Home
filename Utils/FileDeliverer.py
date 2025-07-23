"""
Author: Bu Yujun
Date: 7/1/25
"""

import os
import sys
import time
import paramiko
from stat import S_ISDIR, S_ISREG


def get_remote_disk_space(ssh, remote_path):
    """获取远程目录所在磁盘的可用空间（字节）"""
    try:
        # 使用df命令获取磁盘信息
        cmd = f"df -B1 {remote_path} | tail -1 | awk '{{print $4}}'"
        stdin, stdout, stderr = ssh.exec_command(cmd)
        output = stdout.read().decode('utf-8').strip()
        exit_status = stdout.channel.recv_exit_status()

        if exit_status != 0:
            error = stderr.read().decode('utf-8')
            raise Exception(f"获取远程磁盘空间失败: {error}")

        return int(output)
    except Exception as e:
        print(f"获取远程磁盘空间时出错: {str(e)}")
        return -1


def calculate_local_folder_size(local_folder):
    """计算本地文件夹的总大小（字节）"""
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(local_folder):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            try:
                total_size += os.path.getsize(fp)
            except OSError:
                print(f"无法获取文件 {fp} 的大小")
    return total_size


def create_remote_folder(ssh, remote_base_dir, folder_name):
    """
    在远程服务器上创建同名文件夹，如果已存在则删除重建

    参数:
    ssh: SSH客户端连接
    remote_base_dir: 远程服务器上的基础目录
    folder_name: 要创建的文件夹名称
    """
    remote_folder_path = os.path.join(remote_base_dir, folder_name)
    try:
        # 检查目录是否存在，存在则删除
        stdin, stdout, stderr = ssh.exec_command(
            f"if [ -d '{remote_folder_path}' ]; then rm -rf '{remote_folder_path}'; fi")
        exit_status = stdout.channel.recv_exit_status()
        if exit_status != 0:
            error = stderr.read().decode('utf-8')
            raise Exception(f"删除已存在的远程文件夹失败: {error}")

        # 创建新目录
        stdin, stdout, stderr = ssh.exec_command(f"mkdir -p '{remote_folder_path}'")
        exit_status = stdout.channel.recv_exit_status()
        if exit_status != 0:
            error = stderr.read().decode('utf-8')
            raise Exception(f"创建远程文件夹失败: {error}")

        print(f"成功在远程服务器上创建文件夹: {remote_folder_path}")
        return remote_folder_path
    except Exception as e:
        print(f"创建远程文件夹时出错: {str(e)}")
        return None


def transfer_files(sftp, local_folder, remote_folder):
    """
    递归地将本地文件夹内容传输到远程文件夹

    参数:
    sftp: SFTP客户端连接
    local_folder: 本地文件夹路径
    remote_folder: 远程文件夹路径
    """
    try:
        total_files = sum(len(files) for _, _, files in os.walk(local_folder))
        transferred_files = 0

        for root, dirs, files in os.walk(local_folder):
            # 计算相对路径
            relative_path = os.path.relpath(root, local_folder)
            if relative_path == ".":
                relative_path = ""

            # 在远程创建对应的目录结构
            remote_dir = os.path.join(remote_folder, relative_path)
            try:
                sftp.stat(remote_dir)
            except FileNotFoundError:
                sftp.mkdir(remote_dir)

            # 传输文件
            for file in files:
                local_file_path = os.path.join(root, file)
                remote_file_path = os.path.join(remote_dir, file)

                # 显示进度
                transferred_files += 1
                progress = (transferred_files / total_files) * 100
                print(f"\r正在传输文件 {transferred_files}/{total_files} ({progress:.2f}%): {file}", end="")

                # 传输文件
                sftp.put(local_file_path, remote_file_path)

        print("\n文件传输完成")
        return True
    except Exception as e:
        print(f"\n文件传输过程中出错: {str(e)}")
        return False


def verify_transfer(sftp, local_folder, remote_folder):
    """
    验证本地和远程文件夹内容是否一致，并创建状态文件

    参数:
    sftp: SFTP客户端连接
    local_folder: 本地文件夹路径
    remote_folder: 远程文件夹路径
    """
    try:
        print("开始验证文件传输...")

        # 计算本地文件的总大小和数量
        local_files = {}
        total_local_size = 0
        for root, _, files in os.walk(local_folder):
            relative_path = os.path.relpath(root, local_folder)
            if relative_path == ".":
                relative_path = ""

            for file in files:
                local_file_path = os.path.join(root, file)
                file_size = os.path.getsize(local_file_path)
                total_local_size += file_size

                remote_file_path = os.path.join(remote_folder, relative_path, file)
                local_files[remote_file_path] = file_size

        # 获取远程文件的信息
        total_remote_size = 0
        remote_files_count = 0

        # 递归遍历远程目录
        def traverse_remote_dir(path):
            nonlocal total_remote_size, remote_files_count
            try:
                items = sftp.listdir_attr(path)
                for item in items:
                    item_path = os.path.join(path, item.filename)
                    if S_ISDIR(item.st_mode):
                        traverse_remote_dir(item_path)
                    elif S_ISREG(item.st_mode):
                        # 排除stats.txt文件本身
                        if item.filename != "stats.txt":
                            total_remote_size += item.st_size
                            remote_files_count += 1
            except Exception as e:
                print(f"遍历远程目录时出错: {str(e)}")

        traverse_remote_dir(remote_folder)

        # 验证文件数量和总大小
        local_files_count = len(local_files)
        size_mismatch = abs(total_local_size - total_remote_size)
        size_threshold = 1024  # 允许1KB的误差

        verification_result = False
        status_message = ""

        if local_files_count == remote_files_count and size_mismatch < size_threshold:
            verification_result = True
            status_message = f"验证成功: 本地 {local_files_count} 个文件, 远程 {remote_files_count} 个文件\n"
            status_message += f"本地总大小: {total_local_size} 字节, 远程总大小: {total_remote_size} 字节"
            print(status_message)
        else:
            status_message = f"验证失败: 本地 {local_files_count} 个文件, 远程 {remote_files_count} 个文件\n"
            status_message += f"本地总大小: {total_local_size} 字节, 远程总大小: {total_remote_size} 字节"
            print(status_message)

        # 创建stats.txt文件
        stats_file_path = os.path.join(remote_folder, "stats.txt")
        try:
            with sftp.file(stats_file_path, 'w') as f:
                f.write(f"transfer_{'OK' if verification_result else 'NOK'}\n")
                f.write(f"transfer_date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(status_message)
            print(f"状态文件已创建: {stats_file_path}")
        except Exception as e:
            print(f"创建状态文件时出错: {str(e)}")

        return verification_result
    except Exception as e:
        print(f"验证过程中出错: {str(e)}")
        # 创建失败状态文件
        stats_file_path = os.path.join(remote_folder, "stats.txt")
        try:
            with sftp.file(stats_file_path, 'w') as f:
                f.write(f"transfer_NOK\n")
                f.write(f"transfer_date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"错误信息: {str(e)}")
            print(f"错误状态文件已创建: {stats_file_path}")
        except Exception as inner_e:
            print(f"创建错误状态文件时出错: {str(inner_e)}")
        return False


def deliver_file(host, username, password, local_folder, remote_base_dir):
    # 配置信息 - 请根据实际情况修改
    config = {
        "host": host,
        'port': 22,  # SSH端口
        "username": username,
        "password": password,  # 密码，如使用密钥认证则设为None
        "local_folder": local_folder,  # 本地文件夹路径older",  # 本地文件夹路径
        "remote_base_dir": remote_base_dir,  # 远程文件夹路径
    }

    # 检查配置
    if not os.path.isdir(config['local_folder']):
        print(f"错误: 本地文件夹 '{config['local_folder']}' 不存在")
        sys.exit(1)

    try:
        # 创建SSH客户端
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # 连接到远程服务器
        print(f"正在连接到远程服务器 {config['host']}...")
        ssh.connect(
            config['host'],
            port=config['port'],
            username=config['username'],
            password=config['password']
        )
        print("连接成功")

        # 创建SFTP客户端
        sftp = ssh.open_sftp()

        # 获取本地文件夹名称
        folder_name = os.path.basename(os.path.normpath(config['local_folder']))

        # 计算本地文件夹大小
        print("正在计算本地文件夹大小...")
        local_folder_size = calculate_local_folder_size(config['local_folder'])
        required_space = local_folder_size * 1.5
        print(f"本地文件夹大小: {local_folder_size / (1024 ** 3):.2f} GB")
        print(f"所需最小空间: {required_space / (1024 ** 3):.2f} GB (1.5倍本地文件夹大小)")

        # 检查远程磁盘空间
        print("正在检查远程磁盘可用空间...")
        remote_available_space = get_remote_disk_space(ssh, config['remote_base_dir'])
        if remote_available_space < 0:
            print("无法获取远程磁盘空间信息，程序退出")
            ssh.close()
            sys.exit(1)

        print(f"远程磁盘可用空间: {remote_available_space / (1024 ** 3):.2f} GB")

        # 比较可用空间和所需空间
        if remote_available_space < required_space:
            space_shortage = required_space - remote_available_space
            print(f"❌ 错误: 远程磁盘空间不足，缺少 {space_shortage / (1024 ** 3):.2f} GB")
            ssh.close()
            sys.exit(1)
        else:
            print(
                f"✅ 远程磁盘空间充足，可用空间比所需空间多 {(remote_available_space - required_space) / (1024 ** 3):.2f} GB")

        # 创建远程文件夹（如果已存在则删除重建）
        remote_folder = create_remote_folder(ssh, config['remote_base_dir'], folder_name)
        if not remote_folder:
            print("无法创建远程文件夹，程序退出")
            ssh.close()
            sys.exit(1)

        # 传输文件
        transfer_success = transfer_files(sftp, config['local_folder'], remote_folder)

        if transfer_success:
            # 验证传输
            verification_success = verify_transfer(sftp, config['local_folder'], remote_folder)
            if verification_success:
                print("✅ 文件传输并验证成功")
            else:
                print("❌ 文件传输验证失败")
        else:
            print("❌ 文件传输失败")

        # 关闭连接
        sftp.close()
        ssh.close()

    except Exception as e:
        print(f"发生错误: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    config = {
        "host": "10.192.68.107",
        "username": "zhangliwei01",
        "password": "Pass1234",
        "local_folder": "/media/data/Q_DATA/debug_data/COMBINE_BAG/20250616-005-AEB/2025_06_16-09_06_16=2025_06_16-09_09_16/ROSBAG",
        "remote_base_dir": "/home/zhangliwei01/ZONE"  # 远程文件夹路径
    }

    deliver_file(*config.values())
