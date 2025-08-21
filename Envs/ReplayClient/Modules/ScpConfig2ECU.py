import os
import subprocess
import time
from pathlib import Path
import hashlib
import paramiko
from scp import SCPClient
import glob
from typing import Dict, List, Optional

from Envs.Master.Modules.PowerController.power_controller import *

from Envs.ReplayClient.Modules.VersionControl import VersionControl
from Utils.Libs import *
# from Utils.Libs import kill_tmux_session_if_exists

from Envs.ReplayClient.Modules.EcuFlasher import boxed_text

def create_boxed_prompt(text: str, color_code: str = "\033[94m",input_flag=True) -> str:
    """
    生成带大边框的彩色提示文字

    参数:
        text: 要显示的提示内容
        color_code: ANSI颜色码（默认蓝色）

    返回:
        带边框和颜色的提示字符串
    """

    RESET = "\033[0m"
    # 计算边框宽度（文字长度+2个内边距）
    box_width = len(text) * 2 + 4
    # 顶部边框
    top_border = f"{color_code}+" + "-" * box_width + f"+{RESET}"
    # 中间内容（带左右内边距）
    content_line = f"{color_code}|  {text}  |{RESET}"
    # 底部边框
    bottom_border = f"{color_code}+" + "-" * box_width + f"+{RESET}"
    if input_flag:
        # 组合成带换行的边框文字
        return f"\n{top_border}\n{content_line}\n{bottom_border}\n请输入: "
    else:
        # 组合成带换行的边框文字
        return f"\n{top_border}\n{content_line}\n{bottom_border}"


# 颜色选项（按需选择）
COLORS = {
    "red": "\033[91m",
    "green": "\033[92m",
    "yellow": "\033[93m",
    "blue": "\033[94m",
    "purple": "\033[95m"
}

def create_ssh_client(host: str, port: int, username: str, password: Optional[str] = None) -> paramiko.SSHClient:
    """创建SSH连接客户端"""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(
            hostname=host,
            port=port,
            username=username,
            password=password
        )
        print("✅ SSH连接成功")
        return ssh
    except Exception as e:
        print(f"❌ SSH连接失败: {str(e)}")
        raise


def run_remote_commands(ssh: paramiko.SSHClient, commands: List[str]) -> bool:
    """执行远程命令（第一阶段）"""
    print("\n===== 第一阶段：执行远程命令 =====")
    for cmd in commands:
        print(f"执行命令: {cmd}")
        stdin, stdout, stderr = ssh.exec_command(cmd)
        exit_status = stdout.channel.recv_exit_status()
        output = stdout.read().decode('utf-8').strip()
        error = stderr.read().decode('utf-8').strip()

        if exit_status != 0:
            print(f"❌ 命令执行失败: {error}")
            return False
        if output:
            print(f"输出: {output}")
    print("✅ 所有远程命令执行成功")
    return True


# =============================================
# MD5校验相关函数
# =============================================
def get_remote_md5(ssh: paramiko.SSHClient, file_path: str) -> Optional[str]:
    """获取单个远程文件的md5sum值"""
    try:
        cmd = f"md5sum {file_path} | awk '{{print $1}}'"
        # print('get_remote_md5::  ',cmd)
        stdin, stdout, stderr = ssh.exec_command(cmd)
        exit_status = stdout.channel.recv_exit_status()
        if exit_status != 0:
            print(f"❌ 获取{file_path}的md5失败: {stderr.read().decode('utf-8')}")
            return None
        return stdout.read().decode('utf-8').strip()
    except Exception as e:
        print(f"❌ 获取md5时出错: {str(e)}")
        return None

def scp_folder_to_remote(ssh: paramiko.SSHClient,
                         local_folder: str,
                         remote_folder: str,
                         recursive: bool = True) -> bool:
    """
    将本地文件夹通过SCP传输到远程服务器

    参数:
        ssh: 已建立的SSH连接对象
        local_folder: 本地文件夹路径（绝对路径或相对路径）
        remote_folder: 远程目标文件夹路径
        recursive: 是否递归传输子文件夹（默认True）

    返回:
        传输成功返回True，失败返回False
    """
    try:
        # 确保远程目标文件夹存在
        # ssh.exec_command(f"mkdir -p {remote_folder}")

        # 使用SCPClient传输文件夹
        with SCPClient(ssh.get_transport()) as scp:
            print(f"开始将本地文件夹 {local_folder} 传输到远程 {remote_folder}\n...")

            # 传输文件夹（recursive=True表示递归传输子文件夹）
            scp.put(local_folder,
                    remote_path=remote_folder,
                    recursive=recursive)

            print(f"✅ 文件夹传输完成：{local_folder} -> {remote_folder}")
            return True

    except Exception as e:
        print(f"❌ 文件夹传输失败：{str(e)}")
        return False


def get_local_file_md5(file_path: Path) -> str:
    md5_hash = hashlib.md5()
    with file_path.open("rb") as f:
        while chunk := f.read(4096):
            md5_hash.update(chunk)
    return md5_hash.hexdigest()


def run_remote_commands(ssh: paramiko.SSHClient, command) -> bool:
    """执行远程命令"""
    print(f"执行远程命令: {command}")
    stdin, stdout, stderr = ssh.exec_command(command)
    exit_status = stdout.channel.recv_exit_status()
    output = stdout.read().decode('utf-8').strip()
    error = stderr.read().decode('utf-8').strip()

    if exit_status != 0:
        print(f"❌ 命令执行失败: {error}")
        return False
    if output:
        print(f"输出: {output}")
    print("✅ 远程命令执行成功")
    return True


def get_max_sensor_log(ssh: paramiko.SSHClient) -> Optional[str]:
    """
    获取到当前sensor center文件夹下最大的log的文件名
    """
    remote_log_dir = '/log/app/sensor_center'
    command = f"ls -v -1 {remote_log_dir}/log_*.log | tail -n 1"

    stdin, stdout, stderr = ssh.exec_command(command)

    # 读取结果（最后一行即目标文件）
    largest_log = stdout.read().decode('utf-8').strip()
    error = stderr.read().decode('utf-8').strip()

    if error:
        print(f"命令执行错误: {error}")
        return None

    return largest_log if largest_log else ""

def check_sensor_log_last_5lines(ssh: paramiko.SSHClient) -> bool:
    """
    检查log文件的最后五行，若任意一行有检查到sensor fps = 0.00 认为回灌模式已经开启，返回True；否则返回False
    """
    remote_log_dir = '/log/app/sensor_center'
    command = f"ls -v -1 {remote_log_dir}/log_*.log | tail -n 1"

    stdin, stdout, stderr = ssh.exec_command(command)

    # 读取结果（最后一行即目标文件）
    largest_log = stdout.read().decode('utf-8').strip()
    if largest_log:
        tail_5lines_cmd = "tail -n 5 {}".format(largest_log)
        stdin_5lines, stdout_5lines, stderr_5lines = ssh.exec_command(tail_5lines_cmd)
        log_lines = stdout_5lines.read().decode('utf-8').splitlines()
        tail_error = stderr_5lines.read().decode('utf-8').strip()
        if tail_error:
            print(f"读取日志失败: {tail_error}")
            return False
        camera_pattern = re.compile(r"\[camera_(\d+)\]:\s*(\d+\.\d+)")

        # 4. 逐行解析并检查
        all_zere_count = 0
        for line_num, line in enumerate(log_lines, 1):
            # print(f"\n第{line_num}行日志: {line}")

            # 提取当前行中所有[camera_*]: 对应的数值
            matches = camera_pattern.findall(line)
            if not matches:
                print("未找到任何camera记录")
                continue

            # 检查每个camera的数值是否为0.00
            all_zero = True
            for camera_id, value in matches:
                if value != "0.00":
                    # print(f"  [camera_{camera_id}] 数值异常: {value}（不为0.00）")
                    all_zero = False
                else:
                    # print(f"  [camera_{camera_id}] 数值正常: {value}（为0.00）")
                    continue
            if all_zero:
                print(f"第{line_num}行所有camera数值均为0.00")
                all_zere_count += 1
        # 超过两次为True
        print('all_zero_count::' ,all_zere_count)
        if all_zere_count >=2:
            return True
        else:
            return False

    else:
        print("cannot find max sensor log file")
        return False


def close_terminal_by_title(title="StartReplayModel"):
    """
    根据终端标题关闭指定的终端窗口
    参数:
        title: 终端窗口的标题（部分匹配即可
    返回:
        bool: 成功关闭返回True，失败返回False
    """
    try:
        # 使用wmctrl -c 关闭匹配标题的窗口
        # -c 选项会关闭标题包含指定字符串的窗口
        std_in,std_out, std_err = subprocess.run(
            ['wmctrl', '-l'],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        print('wmctrl -l std_out',std_out)
        subprocess.run(
            ['wmctrl', '-c', title],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        print(f"已关闭标题包含[{title}]的终端窗口")
        return True
    except subprocess.CalledProcessError as e:
        print(f"关闭失败：未找到标题包含[{title}]的窗口（错误：{e.stderr.strip()}）")
        return False
    except Exception as e:
        print(f"执行出错：{str(e)}")
        return False


def open_replay_model(session="start_replay_model"):
    """
    运行tmux_ssh_v17_test.sh，开启ecu的回灌模式
    """
    # kill_tmux_session_if_exists(session) # 关闭  session start_replay_model ，并不会关闭嵌套的子tmux终端 my_ssh_session(启动回灌模式的一个tmux窗口)

    # 查找是否有这个session  start_replay_model
    # result = subprocess.run(['tmux', 'ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # if result.returncode != 0:
    #     print("Error listing tmux sessions:", result.stderr)
    #
    # find_session_flag = False
    # for line in result.stdout.splitlines():
    #     if session in line:
    #         find_session_flag = True
    #         break
    #
    # #没有 session start_replay_model时 创建这个session
    # if not find_session_flag:
    #     os.system(f'tmux new-session -s {session} -n tmux_ssh -d')
    # else:
    #     print("Already exists start_replay_model tmux session ,,,,,,,,")
    kill_tmux_session_if_exists("my_ssh_session")
    time.sleep(0.5)
    kill_tmux_session_if_exists("start_replay_model")
    time.sleep(1)
    os.system(f'tmux new-session -s {session} -n tmux_ssh -d')
    time.sleep(0.5)
    os.system(f'tmux send-keys -t {session}:tmux_ssh "if [ -n "$TMUX" ]; then unset TMUX; fi; bash {tmux_ssh_test_path}" C-m')

    terminal_title = session
    return terminal_title

    # time.sleep(10)

def replay_model_start_success(ssh: paramiko.SSHClient) -> bool:
    """
    打开网络回灌模式，且确定启动命令正常正常,通过检查网络回灌模式是否正常启动，查看相机的帧率是否都是0
    """
    # 打开回灌模式


    max_tru_start_times = 3
    try_count = 0
    while try_count < max_tru_start_times:

        if not try_count:
            open_replay_model()
        else:
            close_terminal_by_title()
            time.sleep(1)
            open_replay_model()

        time.sleep(20)
        try_count += 1
        if check_sensor_log_last_5lines(ssh):
            return True
        else:
            continue
    print(f"尝试 {max_tru_start_times} 次启动回灌模式均失败，请手动检查")
    return False



# def check_camera_internet_replay_model_start_success(ssh: paramiko.SSHClient):
#     """
#     检查网络回灌模式是否正常启动，查看相机的帧率是否都是0
#     """


class ChangeConfigVer:

    def __init__(self, local_config_folder_path: str,
                 remote_host="172.31.1.40",
                 remote_user="root",
                 remote_port=22,
                 remote_password=''
                 ) -> None:

        self.local_config_folder_path = local_config_folder_path
        if not Path(local_config_folder_path).exists():
            raise FileExistsError(f"指定参数文件夹路径不存在-{self.local_config_folder_path}")
        self.remote_host = remote_host
        self.remote_user = remote_user
        self.remote_port = remote_port
        self.remote_password = remote_password

        self._ssh = None

        self.local_config_md5 = {}
        self.remote_config_md5 = {}

        self.check_md5 = {}
        self.relative_bin_path = 'v2/bin/camera/installation/*.bin'
        self.relative_json_path = 'json/*.json'

        self.remote_v2_path = '/app/opt/sp/onboard/params/run_params/vehicles'
        self.remote_jsons_paths = ['/app_data/zone/camera/', '/app_data/zone/camera/release', '/app_data/zone/camera/production']

        # self.check_files = self.get_check_files()
        self.power_supply = VersionControl('EP39')
        # 如果初始化时，可编程自动电源没有上电，需要对电源进行上电控制
        try:
            if power_status := self.power_supply.power_ctrl_get_status():
                pass
            try:
                if power_status:
                    loop_time = 4

                    if not power_status['power_is_on']:
                        while loop_time:
                            loop_time -= 1
                            self.power_supply.power_ctrl_power_on()

                            time.sleep(3)
                            if self.power_supply.check_power_on_success():
                                break
                    if loop_time < 0:
                        raise EnvironmentError("反复上电检验失败，手动检查")
            except EnvironmentError  :
                raise EnvironmentError("上电成功校验失败")

        except Exception as EEE:
            if isinstance(EEE, EnvironmentError):
                raise EnvironmentError("上电成功校验失败,请手动检查")

            else:
                self.auto_power_flag = False
        else:
            self.auto_power_flag = True

    def create_ssh_client(self) -> paramiko.SSHClient:
        """创建SSH连接客户端"""
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(
                hostname=self.remote_host,
                port=self.remote_port,
                username=self.remote_user,
                password=self.remote_password,
            )
            print("✅ SSH连接成功--{}".format(self.remote_host))
            return ssh
        except Exception as e:
            print(f"❌ SSH连接失败: {str(e)}")
            raise Exception("❌ SSH连接失败")

    def get_ssh_client(self) -> paramiko.SSHClient:
        """获取有效的SSH连接（断开则重建）"""
        # 检查当前连接是否存在且有效
        if hasattr(self, '_ssh') and self._ssh is not None:
            try:
                # 发送一个测试命令（如echo）验证连接
                stdin, stdout, stderr = self._ssh.exec_command('echo "ping"', timeout=2)
                stdout.read()  # 读取输出确认命令执行成功
                return self._ssh
            except:
                # 连接无效，关闭旧连接
                self._ssh.close()
                self._ssh = None

        # 重建连接
        self._ssh = self.create_ssh_client()
        return self._ssh


    def check_local_files_and_get_md5(self):
        check_install_bin = glob.glob(str(Path(self.local_config_folder_path) / Path(self.relative_bin_path) ))
        check_jsons = glob.glob(str(Path(self.local_config_folder_path) / Path(self.relative_json_path)))

        if not  check_install_bin :
            raise FileNotFoundError("v2 文件夹下不存在对应的 /bin/camera/installation/*.bin 文件")
        elif len(check_install_bin) < 6:
            raise FileNotFoundError("v2 文件夹下缺少对应的 /bin/camera/installation/*.bin 文件，数量不足六个")
        if not check_jsons :
            raise FileNotFoundError("json文件夹下不存在对应的 *.json 文件")
        elif len(check_jsons) < 6:
            raise FileNotFoundError("json文件夹缺少对应的 *.json 文件，数量不足六个")

        check_files = check_install_bin + check_jsons

        for file_path in check_files:
            path_class = Path(file_path)
            file_base_name = path_class.name
            temp_md5 = get_local_file_md5(path_class)

            self.local_config_md5[file_base_name] = temp_md5

        return self.local_config_md5

    def mount_remote_app(self):
        """
        该动远程板子中的权限
        """
        self._ssh = self.get_ssh_client()
        time.sleep(0.1)
        mount_cmd = [
            "blockdev --setrw `mount | grep by-name/app_ | awk '{print $1}'`",
            "mount -o remount,rw /app",
            # 保存一下
            "sync"
        ]
        # 执行更换命令的权限
        for cmd in mount_cmd:
            self._ssh.exec_command(cmd)

    def rm_remote_v2(self):
        """
        删除远程的v2 文件夹
        """
        self._ssh = self.get_ssh_client()
        rm_v2_cmd = f'rm -r {Path(self.remote_v2_path) / "v2"}'
        print(rm_v2_cmd)
        try:
            self._ssh.exec_command(rm_v2_cmd)
        except Exception as del_v2_e:
            raise Exception(f"删除v2文件夹失败：{del_v2_e}")



    def scp_local_config_to_remote(self):
        """
        将本地config 文件传给远程remote 的 路径
        """
        #  用两种不同方式
        self._ssh = self.get_ssh_client()
        scp_folder_to_remote(self._ssh,str(Path(self.local_config_folder_path) / "v2") , str(Path(self.remote_v2_path)))

        # scp_json_cmds = [(f"sshpass -p '{self.remote_password}' scp {Path(self.local_config_folder_path) / self.relative_json_path} "
        #                   f"{self.remote_user}@{self.remote_host}:{single_path}") for single_path in self.remote_jsons_paths
        #                 for single_path in self.remote_jsons_paths]
        # json_success = []
        for single_json_path in self.remote_jsons_paths:
            print('start scp .json file')
            json_names = glob.glob(os.path.join(self.local_config_folder_path ,self.relative_json_path))
            for json_file_name in json_names:
                with SCPClient(self._ssh.get_transport()) as scp:
                    # 传输文件夹（recursive=True表示递归传输子文件夹）
                    scp.put(json_file_name,
                            remote_path=single_json_path,
                            recursive=False)
        #     json_cmd = ["sshpass",
        #     "-p", '',  # 密码为空字符串（如果实际有密码，替换为你的密码）
        #     "scp",
        #     str(Path(self.local_config_folder_path) / self.relative_json_path),
        #     f"{self.remote_user}@{self.remote_host}:{single_path}"
        # ]
        #     # 执行远程命令，返回元组 (stdin, stdout, stderr)
        #     # print('json_cmd:::',json_cmd)
        #     result = subprocess.run(
        #                         json_cmd,
        #                         stdout=subprocess.PIPE,
        #                         stderr=subprocess.PIPE,
        #                         text=True,  # 文本模式，输出为字符串
        #                         shell = True  # 关键：让 shell 处理 *.json 通配符
        #                     )
        #     # 获取退出码（subprocess的方式）
        #     exit_code = result.returncode  # 直接从CompletedProcess对象获
        #     # 获取输出
        #     stdout_output = result.stdout
        #     stderr_output = result.stderr
        #     if exit_code != 0:
        #         print(f"传输失败了！ \t {json_cmd}")
        #         print(stderr_output.decode('utf-8'),' -----‘’----- ', exit_code)
        #         return False
        #     else:
        #         print(stdout_output)
        #         json_success.append(exit_code)

        # 保存一下
        time.sleep(0.1)
        self._ssh.exec_command(f'sync')

    def check_remote_config_all_md5_equal_local(self):
        """
        检测远程installation的bin文件的md5码，检测是否与本地一致
        """
        self._ssh = self.get_ssh_client()
        if not self.local_config_md5:
            self.local_config_md5 = self.check_local_files_and_get_md5()
        for file_name in self.local_config_md5.keys():

            if file_name.endswith(".json"):
                remote_file_md5 = get_remote_md5(self._ssh,Path(self.remote_jsons_paths[0]) / file_name)
            elif file_name.endswith(".bin"):
                # bin 文件的远程路径 获取其md5
                # print(f"FFFFFFFFF:::::: {Path(self.remote_v2_path) / Path(self.relative_bin_path).parent / file_name}")
                remote_file_md5 = get_remote_md5(self._ssh, Path(self.remote_v2_path) / Path(self.relative_bin_path).parent / file_name)
            self.remote_config_md5[file_name] = remote_file_md5
            if remote_file_md5 != self.local_config_md5[file_name]:
                return False

        return True

    def replay_model_start_success(self) -> bool:
        """
        打开网络回灌模式，且确定启动命令正常正常,通过检查网络回灌模式是否正常启动，查看相机的帧率是否都是0
        """
        # 打开回灌模式
        self._ssh = self.get_ssh_client()

        max_tru_start_times = 3
        try_count = 0
        while try_count < max_tru_start_times:

            if not try_count:
                open_replay_model()
                time.sleep(1)
            else:
                # close_terminal_by_title()
                time.sleep(1)
                open_replay_model()

            time.sleep(20)
            try_count += 1
            if check_sensor_log_last_5lines(self._ssh):
                return True
            else:
                continue
        print(f"尝试 {max_tru_start_times} 次启动回灌模式均失败，请手动检查")
        return False

    def change_config_in_one(self,max_scp_times = 3):

        print("start check local config and get md5")
        self.check_local_files_and_get_md5()
        print("start mount app folder")
        self.mount_remote_app()
        print("start rm v2 folder")
        self.rm_remote_v2()
        print("start scp local config to remote")

        scp_times = 0

        while scp_times < max_scp_times:
            self.scp_local_config_to_remote()

            time.sleep(1)

            if self.check_remote_config_all_md5_equal_local():
                print("检查参数md5已经一致")
                break
            scp_times += 1
        # 多次重复scp参数失败
        if scp_times > max_scp_times:
            boxed_text("反复多次更换参数失败，请检查参数")
            return False
        # try:
        #     self.power_supply.power_ctrl_get_status()
        # except Exception as exc:
        #     print(exc)
        #     self.auto_power_flag = False
        # else:
        #     self.auto_power_flag = True
        if self.auto_power_flag:

            print("auto power off and power on ")
            self.power_supply.power_ctrl_power_off()

            time.sleep(3)
            self.power_supply.power_ctrl_power_on()
            time.sleep(1)


            power_off_on_success = self.power_supply.check_power_on_success()
            if power_off_on_success:
                print(create_boxed_prompt("参数更换成功，并重新上下电", COLORS["red"]))
                return True
            else:
                boxed_text("电源自动下上电失败，请检查")
                return False
        else: # 没有自动控制电源的时候
            # 带颜色的输入提示（自动包含重置）
            # 使用示例：生成带绿色边框的提示
            prompt = create_boxed_prompt("已经传输完所有参数（按回车用默认值）", COLORS["red"])
            user_input = input(prompt)

            if user_input == "":
                print("你按下了回车键")
            else:
                print(f"你输入的内容是：{user_input}")


            prompt2 = create_boxed_prompt("请确认电源已经重新上下电完成（按回车用默认值）", COLORS["red"])
            user_input2 = input(prompt2)
            if user_input2 == '1':

                print("你按下了 1 键")
            else:
                print(f"你输入的内容是：{user_input}")

            return True




# 命令行调用示例
if __name__ == "__main__":
    config_path = '/home/zhangliwei01/ZONE/v2_new_rear/1130_v2'
    ChangeConfigVer1 = ChangeConfigVer(config_path)
    # ChangeConfigVer1.change_config_in_one()
    # ssh = ChangeConfigVer1.create_ssh_client()
    # ffff = check_sensor_log_last_5lines(ssh)
    # print(ffff)
    # ChangeConfigVer2 = ChangeConfigVer(config_path)
    # change_c_flag = ChangeConfigVer2.change_config_in_one()
    # # if change_c_flag:
    # replay_flag = ChangeConfigVer2.replay_model_start_success()

    open_replay_model()

    # replay_model_start_success(ssh)