"""
Date: 2024/08/07
Author: Tian Loong
Class: VersionControl
"""
import glob
import os
import re
import subprocess
import sys
import time
from importlib.abc import Loader

import paramiko

# from pydantic_core import InitErrorDetails

from scp import SCPException

from wcwidth import wcswidth

from Envs.ReplayClient.Modules.EcuFlasher import flash_mcu, flash_mpu, flash_j5, main as auto_flash
from Envs.ReplayClient.Modules.EcuFlasher import parse_arguments, check_version, check_existing_files, upload_files, \
    find_target_dir, boxed_text
from Envs.ReplayClient.Interfaces.Api_Ecu30fpsConfig import ssh_pass, j5a_ssh_client
from Utils.Libs import kill_tmux_session_if_exists, project_path
from Envs.Master.Modules.PowerController.power_controller import *
from Envs.ReplayClient.Interfaces.Libs import get_project_path
from Envs.ReplayClient.Interfaces.Api_Ecu30fpsConfig import change_sensor_30fps, back_sensor_20fps

J5A = ''
J5B = ''
S32G = ''


def print_colored(text, color='bright_magenta'):
    # 定义颜色代码
    colors = {
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[37m",
        "bright_red": "\033[38;5;196m",
        "bright_green": "\033[38;5;46m",
        "bright_yellow": "\033[38;5;226m",
        "bright_blue": "\033[38;5;27m",
        "bright_magenta": "\033[38;5;201m",
        "bright_cyan": "\033[38;5;51m",
        "bright_orange": "\033[38;5;214m",
        "reset": "\033[0m"
    }

    # 获取对应颜色代码，如果找不到颜色则使用默认颜色
    color_code = colors.get(color, colors["reset"])

    # 打印带颜色的文本
    print(f"{color_code}{text}{colors['reset']}")


def boxed_text_colored(text):
    # Split the text into lines if it's multi-line
    lines = text.split('\n')
    # Determine the length of the longest line
    max_length = max(wcswidth(line) for line in lines)
    # Calculate the box width
    box_width = max_length + 4  # Add padding for spaces and box borders

    # Print the top border of the box
    print_colored("#" * box_width)

    # Print each line with padding and side borders
    for line in lines:
        line_width = wcswidth(line)
        padding = max_length - line_width
        left_padding = padding // 2
        right_padding = padding - left_padding
        centered_line = ' ' * left_padding + line + ' ' * right_padding
        print_colored(f'# {centered_line} #')

    # Print the bottom border of the box
    print_colored("#" * box_width)


def flash_firmware_():
    args = parse_arguments()
    args_list = ['-a', args.artifact_path] + ['-t'] + args.targets + ['--type', args.type]

    # Save the original sys.argv
    original_sys_argv = sys.argv

    # Replace sys.argv
    sys.argv = [sys.argv[0]] + args_list

    # Call the main function
    auto_flash()

    # Restore the original sys.argv
    sys.argv = original_sys_argv

    print('============================准备关闭刷写窗口============================')
    time.sleep(5)
    kill_tmux_session_if_exists('ses_flash')


def get_version(core='j5a'):
    """
    读取ecu中的当前的版本参数
    """
    if core in ['j5a', 'J5A']:
        curr_j5a_vers = check_version('172.31.131.35')
        return curr_j5a_vers
    elif core in ['j5b', 'J5B']:
        curr_j5b_vers = check_version('172.31.131.36')
        return curr_j5b_vers
    elif core in ['s32g', 'S32G']:
        curr_s32g_vers = check_version('172.31.131.34')
        return curr_s32g_vers


class VersionControl:

    def __init__(self, ecu_type='ES37'):

        self.ecu_type = ecu_type
        if ecu_type == 'ES37':
            self.j5b_addr = '172.31.131.36'

        elif ecu_type in ['1J5', '1j5']:
            pass

        elif ecu_type in ['j6e', 'J6E']:
            raise NotImplemented

        self.s32g_addr = '172.31.131.34'
        self.j5a_addr = '172.31.131.35'

        # 顺序依次为 s32g j5a j5b(无则为空字段)， J6E数据暂时未定
        self.curr_version_list = None

        self.record_old_version_list = None

        try:
            self.power_ctrl_inter = PowerSupplyObjectManager()
        except Exception as power_ctrl_err:
            print(power_ctrl_err)
            # raise power_ctrl_err
        # 如果板子是下电状态，需要给板子上电
        try:
            power_status = get_status(power_inter=self.power_ctrl_inter,
                                      serial_number=self.power_ctrl_inter.serial_numbers[0])
        except Exception as check_status_err:
            print(check_status_err)
        else:
            if not power_status['power_is_on']:
                print('the power is off ,please wait for power on')
                self.power_ctrl_power_on()
                time.sleep(8)
                if not self.check_power_on_success():
                    raise EnvironmentError('power on fail ,please check')
                else:
                    print('power on success')
            elif power_status['power_is_on']:
                print('the power is on now')

            # 将上电后的电源设置为remote模式,保证上下电操作执行
            self.switch_to_remote_mode()

        # 建立与板子的 ssh 链接
        j5a_ssh_host = '172.31.131.35'
        j5b_ssh_host = '172.31.131.36'
        ssh_port = 22
        ssh_user = 'root'
        ssh_pass = ''

        self.j5a_ssh_client = paramiko.SSHClient()
        self.j5b_ssh_client = paramiko.SSHClient()

        self.j5a_ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.j5b_ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # 建立连接
        self.record_ssh = {
            'j5a': None,
            'j5b': None
        }

        # self.open_ssh_connections()

    def power_ctrl_power_on(self):

        power_on_flag = power_on(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])

        return power_on_flag

    def power_ctrl_power_off(self):
        print('try to power off')
        power_off_flag = power_off(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])
        return power_off_flag

    def power_ctrl_get_status(self):
        power_ctrl_status = get_status(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])
        if power_ctrl_status:
            return power_ctrl_status
        else:
            return False

    def switch_to_panel_mode(self):
        boxed_text_colored('try to switch power to panel mode')
        switch_panel_flag = switch_to_panel_mode(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])
        return switch_panel_flag

    def switch_to_remote_mode(self):
        boxed_text_colored('try to switch power to remote mode')
        switch_remote_flag = switch_to_remote_mode(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])
        return switch_remote_flag

    def __del__(self):
        # 断开所有power ctrl 设备的连接
        self.power_ctrl_inter.close_all_instruments()

        for client in self.record_ssh.values():
            if client:
                client.close()

    def open_ssh_connections(self):
        """
        重置现有的 两个ssh 连接 ,如果已存在，则先关闭对应连接，再重新建立连接
        """
        # 建立与板子的 ssh 链接
        j5a_ssh_host = '172.31.131.35'
        j5b_ssh_host = '172.31.131.36'
        ssh_port = 22
        ssh_user = 'root'
        ssh_pass = ''

        # 无论是否建立链接 先关闭
        self.close_ssh_connection()
        try:
            self.j5a_ssh_client.connect(j5a_ssh_host, ssh_port, username=ssh_user, password=ssh_pass,
                                        timeout=10)
            self.j5b_ssh_client.connect(j5b_ssh_host, ssh_port, username=ssh_user, password=ssh_pass,
                                        timeout=10)
        except ConnectionError as EEEE:
            print('build connection failed')

            raise EEEE
        else:
            self.record_ssh = {
                'j5a': self.j5a_ssh_client,
                'j5b': self.j5b_ssh_client
            }
            self.j5a_ssh_client.exec_command('mount -o remount,rw /')
            self.j5b_ssh_client.exec_command('mount -o remount,rw /')

    def close_ssh_connection(self):
        """
        关闭当前的 两个 ssh 连接，如果没有链接 ，则不关闭
        通过self.record_ssh 来记录 ssh client 的链接
        """
        for ssh_host, client in self.record_ssh.items():
            if client and client.get_transport() and client.get_transport().is_active():
                client.close()
                print(f"{ssh_host} 连接已关闭")
                self.record_ssh[ssh_host] = None

    def flash_ecu(self, artificial_folder='/media/data/artif/', mcu=True, mpu=True, j5a=True, j5b=True):
        """
        刷写ecu的为新的版本,当前仅能刷写ES37 2J5参数
        artificial_folder: 刷写新版本的文件所在文件夹，
            下面包含flashing_images_zpd_dj5_1_journey5_20240815_030000
            flashing_images_zpd_dj5_2_journey5_20240815_030000
            install(可以没有此文件夹)、zpd_es37_s32g
            包含此四个刷写文件的文件夹
        """
        # global S32G, J5A, J5B  # 为了在函数内部修改全局变量
        # S32G = '172.31.131.34'
        # J5A = '172.31.131.35'
        # J5B = '172.31.131.36'
        ecu_type = self.ecu_type
        self.record_old_version_list = self.read_ecu_version()
        if ecu_type == 'ES37':
            flash_dict = {
                'mcu': mcu,
                'mpu': mpu,
                'j5a': j5a,
                'j5b': j5b,
            }
            flash_target_args = [key for key, val in flash_dict.items() if val]
            target_str = ' '.join(flash_target_args)

            # 获取到 Api_EcuFlasher.py 的文件夹所在路径
            Api_file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Interfaces')
            print(Api_file_path)

            cmd = f'cd {Api_file_path} && python3 Api_EcuFlasher.py -a {artificial_folder} -t {target_str}'
            print(cmd)
            subprocess.run(cmd, shell=True, check=True)

            # 第二种尝试使用sys.args 实现 flash ecu

            # args_list = ['-a', artificial_folder] + ['-t'] + flash_target_args + ['--type', 't1']
            # original_sys_argv = sys.argv
            #
            # # Replace sys.argv
            # sys.argv = [sys.argv[0]] + args_list
            #
            # # Call the main function
            # auto_flash()
            #
            # # Restore the original sys.argv
            # sys.argv = original_sys_argv
            #
            # print('============================准备关闭刷写窗口============================')
            # time.sleep(5)
            # kill_tmux_session_if_exists('ses_flash')


        elif ecu_type in ['1j5', '1J5']:
            raise NotImplemented
        elif ecu_type in ['j6e', 'J6E']:
            raise NotImplemented

    def read_ecu_version(self, ecu_type='ES37'):
        """
        读取当前参数的版本,
        ES37 返回一个list 分别为 s32g j5a j5b 的 版本号
        1J5 返回一个list 分别为 s32g j5a ‘空字符串’
        """
        if ecu_type == 'ES37':
            s32g_ver = check_version('172.31.131.34')
            j5a_ver = check_version('172.31.131.35')
            j5b_ver = check_version('172.31.131.36')
            return s32g_ver, j5a_ver, j5b_ver
        elif ecu_type in ['1j5', '1J5']:
            s32g_ver = check_version('172.31.131.34')
            j5a_ver = check_version('172.31.131.35')
            j5b_ver = ''
            return s32g_ver, j5a_ver, j5b_ver

        elif ecu_type in ['j6e', 'J6E']:
            raise NotImplemented

    def flash_new_camera_config(self):
        """
        向ecu中刷新新版本的相机参数，
        ecu_type: ecu的型号，目前只有 ES37 & 1J5 & J6E
        config_version : 相机参数的版本 如‘20230517’ 为str类型
        """
        if os.path.exists():
            pass
        if self.ecu_type == 'ES37':
            pass
        elif self.ecu_type == '1J5':
            pass
        elif self.ecu_type == 'J6E':
            pass
        else:
            print('ecu type must be ES37 or 1J5 or J6E')
            raise ValueError('ecu type must be ES37 or 1J5 or J6E')

        raise NotImplemented

    # TODO 检验该函数是否正确 需要修改
    def compare_ecu_version(self, old_version_list, new_version_list):
        """
        对比两个版本号的list 对应的版本号是否一致
        版本号的list 的顺序为 s32g j5a j5b 对应的版本号
        """
        if self.ecu_type == 'ES37':
            for i in range(len(old_version_list)):
                # TODO 如何对比需要修正
                if old_version_list[i] == new_version_list[i]:
                    print(f'新旧版本号比对无变化，刷机失败，请检查 {new_version_list}：{old_version_list}')
                    return False
                else:
                    print(f'版本对比有变化，{new_version_list[i]}：{old_version_list[i]}')
            print('刷写前后所有core版本均有变化')
            return True
        elif self.ecu_type == '1J5':
            raise NotImplemented

        elif self.ecu_type == 'J6E':
            raise NotImplemented

    # TODO 检验该函数是否按照预期进行
    def check_ecu_ver_eql_artif_date(self, artif_path='/media/data/artif', ecu_type='ES37'):
        """
        检查当前刷写后的版本，是否已artif_path 中的文件的日期一致，如果一致则返回True 否则返回 False + 对应的core 名字
        """
        check_ver_list = self.read_ecu_version(ecu_type)
        print(check_ver_list)
        artif_date = self.read_artif_folder_version(artif_path)
        for index, version in enumerate(check_ver_list):
            # 抓取version 中的 date
            temp_re_date = self.extract_timestamp_with_re(version)
            if temp_re_date == artif_date:
                continue
            else:
                # TODO 检验不够严禁，提取版本中的日期
                if index == 0:
                    print(f's32g 版本日期与路径中的不同 {artif_path} ， {version}：{artif_date}')
                    return False, 's32g'
                    # raise EnvironmentError(f's32g 版本日期与路径中的不同 {artif_path} ， {version}：{artif_date}')
                elif index == 1:
                    print(f'j5a 版本日期与路径中的不同 {artif_path} ， {version}：{artif_date}')
                    return False, 'j5a'
                    # raise EnvironmentError(f'j5a 版本日期与路径中的不同 {artif_path} ， {version}：{artif_date}')
                else:
                    print(f'j5b 版本日期与路径中的不同 {artif_path} ， {version}：{artif_date}')
                    return False, 'j5b'
                    # raise EnvironmentError()
        return True, None

    def read_artif_folder_version(self, artif_path='/media/data/artif/'):
        """
        读取当前 刷写folder artif_path 路径下的刷写文件的版本，确认j5a 和 j5b日期相同，并返回一个日期，
        否则raise 相应的 error
        """
        if self.ecu_type == 'ES37':
            j5a_folder = glob.glob(os.path.join(artif_path, '*1_journey5*'))
            j5b_folder = glob.glob(os.path.join(artif_path, '*2_journey5*'))
            install_exist = os.path.exists(os.path.join(artif_path, 'install'))
            s32g_folder = glob.glob(os.path.join(artif_path, '*s32*'))
            print(os.path.join(artif_path, '*s32g*'))

            #  检查 install 文件 和s32g 的刷写文件夹
            if not install_exist:
                print('不存在 install 文件夹')

            # 检查 s32g文件
            if s32g_folder:
                print(s32g_folder)
                for index, s32g_name in enumerate(s32g_folder):
                    if os.path.isdir(os.path.join(artif_path, s32g_name)):
                        print(os.path.join(artif_path, s32g_name))
                        print(f'抓取到s32g 刷写文件夹，请检查 {artif_path}')
                        break
                    elif index == len(s32g_folder) - 1:
                        raise FileNotFoundError(f'未找到对应的 s32g 刷写文件夹，请检查 {artif_path}')
            else:
                raise FileNotFoundError(f'未找到对应的 s32g 刷写文件夹，请检查 {artif_path}')

            # 检查j5a 和 j5b 的刷写文件夹
            if not (j5a_folder and j5b_folder):
                # glob 对 j5a 和j5b都没抓取到时
                raise FileNotFoundError(f'不存在对应的 j5的 刷写文件夹，请检查 {artif_path}')
            else:
                # 抓取到了对应的list，检查对应的文件夹
                if len(j5a_folder) > 2 or len(j5b_folder) > 2:
                    raise FileExistsError(f'存在多个版本的j5 刷写文件夹，请检查 {artif_path}')

                # 只抓取到了j5a 或 j5b的 tar.gz的压缩包时
                elif (len(j5a_folder) == 1 and j5a_folder[0].endswith('.tar.gz')) or (
                        len(j5b_folder) == 1 and j5b_folder[0].endswith('.tar.gz')):
                    raise EnvironmentError('该文件夹仅有 tar.gz 压缩包，请解压后重新重试刷写')
                else:
                    # 提取j5a 和 j5b 的folder 文件夹的名字
                    # 抓取到文件夹
                    if os.path.isdir(os.path.join(artif_path, j5a_folder[0])):
                        # 抓取 j5a文件夹的 date
                        j5a_match_res = self.extract_timestamp_with_re(j5a_folder[0])
                        if j5a_match_res:
                            j5a_date_time = j5a_match_res

                        else:
                            raise EnvironmentError(f'未查找到正确的刷写版本日趋，{j5a_folder[0]} ')
                    elif os.path.isdir(os.path.join(artif_path, j5a_folder[1])):
                        # 抓取 j5文件夹的 date
                        j5a_match_res = self.extract_timestamp_with_re(j5a_folder[1])
                        if j5a_match_res:
                            j5a_date_time = j5a_match_res

                        else:
                            raise EnvironmentError(f'未查找到正确的刷写版本日趋，{j5a_folder[1]} ')

                    else:
                        raise FileExistsError(f'不存在对应的 j5的 刷写文件夹，请检查 {artif_path} {j5a_folder}')

                    if os.path.isdir(os.path.join(artif_path, j5b_folder[0])):
                        # 抓取 j5b文件夹的 date
                        j5b_match_res = self.extract_timestamp_with_re(j5b_folder[0])
                        if j5b_match_res:
                            j5b_date_time = j5b_match_res

                        else:
                            raise EnvironmentError(f'未查找到正确的刷写版本日期，{j5b_folder[0]} ')
                    elif os.path.isdir(os.path.join(artif_path, j5b_folder[1])):
                        # 抓取 j5b文件夹的 date
                        j5b_match_res = self.extract_timestamp_with_re(j5b_folder[1])
                        if j5b_match_res:
                            j5b_date_time = j5b_match_res
                        else:
                            raise EnvironmentError(f'未查找到正确的刷写版本日趋，{j5b_folder[1]} ')

                    else:
                        raise FileExistsError(f'不存在对应的 j5的 刷写文件夹，请检查 {artif_path} {j5b_folder}')

                    # 比较j5a 和 j5b 的日期
                    if j5a_date_time == j5b_date_time:
                        return j5a_date_time
                    else:
                        raise EnvironmentError(f'j5a 与 j5b 的版本号不同，请检查刷写文件夹下的刷写文件： {artif_path} ')

    def check_ecu_ver_and_artif_ver(self, artif_path):
        """
        检查板子内的当前的版本 是否与 artif_path 文件夹中的版本是否一致
        相同则返回True ，不同则返回False
        """

    def extract_timestamp_with_re(self, ver_str):
        """
        使用正则表达式从给定的字符串中提取时间戳部分（YYYYMMDD_HHMMSS）。

        参数:
        s (str): 包含时间戳信息的字符串

        返回:
        str: 提取出的时间戳，格式为 'YYYYMMDD_HHMMSS'，如果未找到则返回None
        """
        # 定义一个正则表达式来匹配连续的14位数字，其中第8位和第9位之间有一个下划线  年月日_时分秒
        pattern = r'\d{8}_\d{6}'

        # 使用re.search()查找字符串中第一个匹配项
        match = re.search(pattern, ver_str)

        # 如果找到了匹配项，则返回匹配到的字符串，否则返回None
        return match.group(0) if match else None

    def get_power_status(self):
        """
        获取当前电源的信息，
        return :
        """
        power_status = get_status(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])

        return power_status

    def check_power_on_success(self):
        """
        通过电流判断，当前电源是否正常启动，检验板子是否正常启动
        """
        # TODO 这些参数都需要调试
        beyond_times = 0  # 电流超过  Ampere_limit 的 次数 ，不同ecu 的Ampere_limit不同
        loop_times = 80  # 循环检查的最大次数 ，每次循环检查电流后停顿0.5s
        flag_times = 35

        if self.ecu_type == "ES37":
            Ampere_limit = 3.1

            while loop_times > 0:
                time.sleep(0.75)
                loop_times -= 1  # 循环减少一次
                #  将 status 中的电流数值提取出来
                temp_status = get_status(self.power_ctrl_inter, self.power_ctrl_inter.serial_numbers[0])
                # print(temp_status)
                temp_ampere = temp_status['current_value']

                if temp_ampere >= Ampere_limit:
                    beyond_times += 1
                    print(loop_times, '----', beyond_times, ':::', temp_ampere)
                    # 大于 电流限定次数时 返回True
                    if beyond_times >= flag_times:
                        # reset config
                        return True
                else:
                    beyond_times = max(beyond_times - 3, 0)  # 如果低于 3.0,则 超过次数 -3,
                    print(loop_times, '----', beyond_times, ':::', temp_ampere)

            return False

        elif self.ecu_type == "ES18":
            raise NotImplemented
        elif self.ecu_type == "1J5":
            raise NotImplemented
        elif self.ecu_type == "J6E":
            raise NotImplemented

    def flash_check_all_in_one(self, artificial_folder='/media/data/artif/'):
        """
        将刷件的所有流程都放在同一个函数中，刷写、电源上下电、版本检查、版本对比等，检验刷件是否成功
        成功则返回True，不成功

        """
        self.open_ssh_connections()

        max_flash_time = 3  # 单次刷件最多次数

        if self.ecu_type == 'ES37':
            # 记录刷件前的版本
            old_version_list = self.read_ecu_version()
            # 先确认老版本的版本号一致，标记flag 是否可用
            if old_version_list[0] == old_version_list[1] == old_version_list[2]:
                old_version_corr_flag = True
                # 如过刷写文件夹下读取到的日期与当前板子版本一致，则不需要刷写，提示不需要刷写， return True
                try:
                    artif_ver_date = self.read_artif_folder_version()
                except Exception as EEE:
                    boxed_text_colored('当前板子中刷写版本号与刷写路径下版本一致，无需刷写')
                    return False
                else:
                    if self.extract_timestamp_with_re(old_version_list[0]) == artif_ver_date:
                        boxed_text_colored('当前板子中刷写版本号与刷写路径下版本一致，无需刷写')
                        return True
            else:
                old_version_corr_flag = False
            while max_flash_time > 0:

                try:
                    max_flash_time = max_flash_time - 1
                    self.flash_ecu(artificial_folder)
                except Exception as flash_err:
                    if max_flash_time != 0:
                        print(flash_err)
                        continue
                    else:
                        raise EnvironmentError(f'连续刷写三次板子报错， {flash_err}')
                # 执行刷写成功后，检查版本是否正确
                else:

                    # 成功运行刷写函数后
                    time.sleep(5)
                    # 下电
                    power_off_time = 0
                    max_power_off_time = 3
                    while True:
                        power_off_flag = self.power_ctrl_power_off()
                        if power_off_flag:
                            power_off_time = 0
                            break
                        else:
                            power_off_time += 1
                            if power_off_time == max_power_off_time:
                                power_off_time = 0
                                raise EnvironmentError('电源关闭三次为成功，请检查')
                            else:
                                time.sleep(0.5)
                                continue
                    time.sleep(1)
                    # 上电
                    max_power_on_time = 3
                    power_on_time = 0
                    while True:
                        power_on_flag = self.power_ctrl_power_on()
                        if power_on_flag:
                            time.sleep(2)
                            if self.check_power_on_success():

                                power_on_time = 0
                                break
                            else:
                                power_on_time += 1
                                if power_on_time == max_power_on_time:
                                    power_on_time = 0
                                    raise EnvironmentError('电源开启成功，三次检查ecu的电流大小均失败，请检查')
                                continue  # 电源开关失败， 再次重启
                        else:
                            power_on_time += 1
                            if power_on_time == max_power_on_time:
                                power_on_time = 0
                                raise EnvironmentError('电源开启三次均失败，请检查')
                            else:
                                continue
                    print('电源上电成功')
                    time.sleep(1)

                    # 重新建立 ssh 链接
                    self.open_ssh_connections()

                    # 读取新的版本日期
                    new_version_date_flag, core_errors = self.check_ecu_ver_eql_artif_date(artificial_folder)
                    if new_version_date_flag:
                        boxed_text(f'刷写成功，检查版本与 {artificial_folder} 中一致 ')
                        return True
                    else:
                        # 刷写三次板子仍旧 版本号不一致
                        if max_flash_time == 0:
                            raise EnvironmentError(f'刷写三次板子后，core中版本号仍旧不一致， {core_errors}')
                        else:
                            print(f'刷写板子后 core中版本号不一致， {core_errors}')
                            time.sleep(1)
                            continue

    def flash_camera_config_in_one(self):
        """
        将参数文件的内容scp拷贝进板子，再执行上下电操作，然后检验参数是否正确，
        成功 返回True ，不正确则重复3次数，三次均失败则 raise error 报错
        """
        print('start poen connection')
        self.open_ssh_connections()

        if self.ecu_type == "ES37":
            # 拷贝前的记录工作，与scp成功失败无关的，不需要进入循环

            # 获取到参数放置的文件夹
            es37_calib_folder = os.path.join(get_project_path(), 'Temp', 'es37_calib')
            calib_path_in_core = '/app_data/horizon/'

            # 用一个list 记录下来拷贝失败的camera
            scp_fail_camera = []

            if os.path.exists(os.path.join(es37_calib_folder, 'camera')):
                print('find camera path')
            else:
                raise FileNotFoundError(os.path.join(es37_calib_folder, 'camera'))
            # scp命令 的 str
            scp_cmd_a = f"sshpass -p '{ssh_pass}' scp -r {os.path.join(es37_calib_folder, 'camera')} root@172.31.131.35:{calib_path_in_core}"
            scp_cmd_b = f"sshpass -p '{ssh_pass}' scp -r {os.path.join(es37_calib_folder, 'camera')} root@172.31.131.36:{calib_path_in_core}"

            scp_times = 3

            while scp_times > 0:
                scp_times -= 1

                # # 重新打开ssh连接
                self.open_ssh_connections()

                # 删除 板子内原本的参数文件

                try:
                    self.j5a_ssh_client.exec_command('rm -r /app_data/horizon/camera')
                    self.j5b_ssh_client.exec_command('rm -r /app_data/horizon/camera')
                except Exception as del_calib_err:
                    print(del_calib_err)
                else:
                    std_del_in, std_del_out, std_del_err = self.j5a_ssh_client.exec_command('ls /app_data/horizon/')
                    print(std_del_out.read().decode())

                # run scp 的 cmd
                # scp_res_a = subprocess.run(["sshpass", "-p", "", "scp", "-r", f"{os.path.join(es37_calib_folder, 'camera')}", f"root@172.31.131.35:{calib_path_in_core}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,shell=True)
                # scp_res_b = subprocess.run(["sshpass", "-p", "", "scp", "-r", f"{os.path.join(es37_calib_folder, 'camera')}", f"root@172.31.131.36:{calib_path_in_core}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,shell=True)
                os.system(scp_cmd_a)
                os.system(scp_cmd_b)
                time.sleep(2)
                self.j5a_ssh_client.exec_command('sync')
                self.j5b_ssh_client.exec_command('sync')
                time.sleep(2)
                # print('scp_res_a and scp_res_b :_:_:_',scp_res_a, scp_res_a)
                scp_res_a = True
                scp_res_b = True
                if not (scp_res_a and scp_res_b):
                    if scp_times == 0:
                        raise SCPException('连续三次scp calib文件失败，退出')
                    else:
                        # 重新scp文件
                        print('scp文件失败,重新执行scp参数命令')
                        time.sleep(1)
                        continue
                else:
                    print('scp命令执行成功')
                    #重新上下电

                    # 检查 scp 进去的文件是否一致 ，检查某个特殊字段

                    # 1. 先读取原始文件的camera_0.json中的字段 ,并将其记录下来
                    calib_compare_dic = {}
                    camera_list = os.listdir(os.path.join(es37_calib_folder, 'camera'))
                    for camera_i in camera_list:
                        with open(os.path.join(es37_calib_folder, 'camera', camera_i, 'camera_0.json'),
                                  'r') as temp_json:
                            calib_data = json.load(temp_json)

                            # 确定读取 参数数值来作为验证值。选用 camera_x
                            calib_compare_dic[camera_i] = calib_data['camera_x']

                    # 再上电 、下电 、 重新建立ssh 连接
                    power_off_time = 0
                    max_power_off_time = 2
                    # 最多下电两次，失败则报错
                    while True:
                        power_off_flag = self.power_ctrl_power_off()
                        if power_off_flag:
                            power_off_time = 0
                            # 下电成功，退出上电循环
                            print('成功下电，退出循环')
                            break
                        else:
                            power_off_time += 1
                            if power_off_time == max_power_off_time:
                                power_off_time = 0
                                raise EnvironmentError('电源关闭三次为成功，请检查')
                            else:
                                time.sleep(0.5)
                                continue
                    # 下电后等待5s再上电
                    time.sleep(5)
                    # 上电  最多上电两次，失败则报错
                    max_power_on_time = 2
                    power_on_time = 0
                    while True:
                        power_on_flag = self.power_ctrl_power_on()
                        if power_on_flag:
                            time.sleep(10)
                            check_status = self.check_power_on_success()
                            print('check_status______________:', check_status)
                            if check_status:
                                power_on_time = 0
                                # 上电成功，退出上电循环
                                break
                            else:
                                power_on_time += 1
                                if power_on_time == max_power_on_time:
                                    power_on_time = 0
                                    raise EnvironmentError('电源开启成功，三次检查ecu的电流大小均失败，请检查')
                                continue  # 电源开关失败， 再次重启
                        else:
                            power_on_time += 1
                            if power_on_time == max_power_on_time:
                                power_on_time = 0
                                raise EnvironmentError('电源开启三次均失败，请检查')
                            else:
                                # 重新下电 最多循环三次
                                power_off_flag = self.power_ctrl_power_off()
                                if power_off_flag:
                                    time.sleep(2)
                                    continue
                                else:
                                    power_off_flag = self.power_ctrl_power_off()
                                    if power_off_flag:
                                        time.sleep(2)
                                        continue
                                    else:
                                        power_off_flag = self.power_ctrl_power_off()
                                        if power_off_flag:
                                            raise EnvironmentError('电源连续下电三次失败')
                    print('电源上电成功')

                    self.open_ssh_connections()  # 重建ssh 连接

                    # 读取板子内的参数的数值，并与记录值去做对比 camera_x

                    print('camera_list:::::::', camera_list)
                    for camera_i in camera_list:
                        a_stdin, a_stdout, a_stderr = self.j5a_ssh_client.exec_command(
                            f'cd /app_data/horizon/camera/{camera_i} && grep "camera_x" camera_0.json')
                        b_stdin, b_stdout, b_stderr = self.j5a_ssh_client.exec_command(
                            f'cd /app_data/horizon/camera/{camera_i} && grep "camera_x" camera_0.json')
                        a_output = a_stdout.read().decode()
                        b_output = b_stdout.read().decode()

                        j5a_exit_status = a_stdout.channel.recv_exit_status()
                        j5b_exit_status = b_stdout.channel.recv_exit_status()
                        print(f'J5AB exit status, {camera_i}___', j5a_exit_status, j5a_exit_status)
                        # 对比参数的命令，执行成功
                        if j5a_exit_status == 0 and j5b_exit_status == 0:
                            j5a_match = re.search(r'[-+]?\d*\.\d+', a_output)
                            j5b_match = re.search(r'[-+]?\d*\.\d+', b_output)
                            if j5a_match and j5b_match:
                                # 尝试用正则表达式去读取对应的数字
                                try:
                                    j5a_decimal_number = float(j5a_match.group())
                                    j5b_decimal_number = float(j5b_match.group())
                                except Exception as re_match_err:
                                    print(re_match_err)
                                    if camera_i not in scp_fail_camera:
                                        scp_fail_camera.append(camera_i)
                                    continue
                                else:
                                    if j5a_decimal_number == j5b_decimal_number and (
                                            abs(j5a_decimal_number - calib_compare_dic[camera_i]) <= 10e-12):
                                        print(f'camera_x,{camera_i}:', '参数一致')
                                        continue
                                    else:
                                        # 当由参数读取到的数值不一致时，说明拷贝参数出了问题
                                        print(f'camera_x,{camera_i}:', 'a:', j5a_decimal_number, 'b:',
                                              j5b_decimal_number, 'calib:', calib_compare_dic[camera_i])
                                        if camera_i not in scp_fail_camera:
                                            scp_fail_camera.append(camera_i)
                        else:
                            # 对比命令执行失败，说明scp命令执行失败了
                            print(
                                f"scp {camera_i} 参数失败，\nJ5A检查命令exit code :{j5a_exit_status} \nJ5B检查命令exit code:{j5b_exit_status}\nINFO:exit code 0为正确运行，其他为不正常运行")
                    # 一次scp校验文件后
                    if not scp_fail_camera:
                        # scp失败的 camera 数量为空
                        # return
                        boxed_text_colored('所有参数拷贝成功')
                        return True
                    else:

                        # scp_fail_camera 中 有拷贝失败的camera
                        boxed_text_colored(str(scp_fail_camera) + '\n校验参数失败')
                        # 清空错误的camera
                        scp_fail_camera = []
                        if scp_times == 0:
                            raise SCPException('连续三次scp calib文件失败，且校验拷贝文件出现错误，退出')
                        else:
                            #重新进入 拷贝参数的全流程循环
                            time.sleep(2)
                            continue

                    # 校验成功 ，说明参数拷贝成功

        elif self.ecu_type == '1J5':
            raise NotImplemented
        elif self.ecu_type == 'J6E':
            raise NotImplemented

    def change_sensor_30fps(self):

        change_sensor_30fps()
        #重新启动板子并确认上电成功
        power_off = self.power_ctrl_power_off()
        time.sleep(3)
        power_on = self.power_ctrl_power_on()

        if power_on or power_off:
            raise EnvironmentError('板子重新上下电时失败')
        print('restart power cmd finished， check power on status')
        time.sleep(3)

        power_on_stat = self.check_power_on_success()
        if power_on_stat:
            self.open_ssh_connections()
            return True
        else:
            raise EnvironmentError('板子重新上电失败')

    def back_sensor_20fps(self):

        back_sensor_20fps()
        #重新启动板子并确认上电成功
        power_off = self.power_ctrl_power_off()
        time.sleep(3)
        power_on = self.power_ctrl_power_on()

        if power_on or power_off:
            raise EnvironmentError('板子重新上下电时失败')
        print('restart power cmd finished， check power on status')
        time.sleep(3)

        power_on_stat = self.check_power_on_success()
        if power_on_stat:
            self.open_ssh_connections()
            return True
        else:
            raise EnvironmentError('板子重新上电失败')


if __name__ == '__main__':
    VC = VersionControl()
    # VC.get_power_status()
    VC.power_ctrl_power_off()
