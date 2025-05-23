"""  
@Author: BU YUJUN
@Date: 2024/7/10 上午9:11  
"""
import inspect
import json
import os
import socket
from datetime import datetime

from Utils.Libs import kill_process_by_port
from Utils.Libs import project_path, bench_config, variables


class UDPLogClient:

    def __init__(self, host=None, port=None):
        if not host:
            self.host = bench_config['Master']['ip']
        else:
            self.host = host
        if not port:
            self.port = variables['udp_port']['master_log']
        else:
            self.port = port

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_log(self, log_entry, log_stage='Undefined', to_server_flag=False):
        """
        发送日志到服务器。

        :param log_entry: 日志条目，可以是字符串或字典（将被转换为JSON）。
        :param log_stage: 日志产生的阶段。
        :param to_server_flag: 是否需要将此日志上传到云端。
        """

        if isinstance(log_entry, str):
            log_entry = json.dumps({'message': log_entry, 'log_stage': log_stage})
        elif isinstance(log_entry, dict):
            log_entry['log_stage'] = log_stage
            log_entry = json.dumps(log_entry)
        else:
            return
        # 将to_server_flag和日志条目一起发送
        log_message = f"{to_server_flag}|{log_entry}"

        self.udp_socket.sendto(log_message.encode(), (self.host, self.port))


class RotatingFileHandler:

    def __init__(self, log_dir=None, filename_prefix='ReplayTestLog'):
        """
        初始化RotatingFileHandler。

        :param log_dir: 日志文件存放的目录
        :param filename_prefix: 日志文件名前缀，默认为'udp_log'
        """
        if not log_dir:
            self.log_dir = os.path.join(project_path, 'Docs', 'Logs')
        else:
            self.log_dir = log_dir
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        self.filename_prefix = filename_prefix
        self.current_log_file = None  # 当前日志文件对象
        self.last_rotation_time = None  # 上一次日志文件轮换时间
        self.rotation_interval = 4 * 60 * 60  # 日志文件轮换间隔，默认为4小时（秒）
        self.open_log_file()  # 打开（或创建）日志文件

    def open_log_file(self):
        """
        根据当前时间打开（或创建）一个新的日志文件。
        """
        timestamp = datetime.now().strftime('%Y-%m-%d_%H')  # 获取当前时间并格式化
        filename = f'{self.filename_prefix}_{timestamp}.log'  # 构造日志文件名
        filepath = os.path.join(self.log_dir, filename)  # 拼接完整的日志文件路径

        # 如果当前没有日志文件对象，或者到了轮换时间，则创建/打开一个新的日志文件
        if self.current_log_file is None or self.should_rotate_log_file(timestamp):
            if self.current_log_file:
                self.current_log_file.close()  # 关闭当前的日志文件
            self.current_log_file = open(filepath, 'a', encoding='utf-8')  # 打开新的日志文件以追加模式写入
            self.last_rotation_time = datetime.now()  # 更新上一次轮换时间
            print(f'日志文件已轮换: {filepath}')  # 打印轮换信息

    def should_rotate_log_file(self, current_timestamp):
        """
        检查是否到了轮换日志文件的时间。

        :param current_timestamp: 当前时间戳（已格式化）
        :return: 是否需要轮换日志文件
        """
        if self.last_rotation_time is None:
            return True  # 如果没有上一次轮换时间，则立即轮换

        # 解析当前时间戳为datetime对象
        current_time = datetime.strptime(current_timestamp, '%Y-%m-%d_%H')
        last_rotation_hour = self.last_rotation_time.hour  # 获取上一次轮换时的小时数
        current_hour = current_time.hour  # 获取当前小时数

        # 检查自上一次轮换以来是否已经过去了4小时，或者小时数是否发生了变化
        time_difference = (datetime.now() - self.last_rotation_time).total_seconds()
        return time_difference >= self.rotation_interval or current_hour != last_rotation_hour

    def write(self, log_entry):
        """
        将日志条目写入当前日志文件。

        :param log_entry: 要写入的日志条目
        """
        current_time = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')  # 获取并格式化当前时间
        self.open_log_file()  # 确保日志文件已打开，并根据需要轮换
        log_text = f'{current_time}({len(log_entry)}) {log_entry}\n'
        print(log_text[0:-1])
        self.current_log_file.write(log_text)  # 写入日志条目和时间戳
        self.current_log_file.flush()  # 刷新缓冲区，确保日志立即写入磁盘

    def close(self):
        """
        关闭当前日志文件。
        """
        if self.current_log_file:
            self.current_log_file.close()  # 关闭日志文件对象
            self.current_log_file = None  # 将当前日志文件对象设置为None


class UDPLogServer:

    def __init__(self, host=None, port=None, log_dir=None, filename_prefix='HilTest_log'):
        if not host:
            self.host = bench_config['Master']['ip']
        else:
            self.host = host
        if not port:
            self.port = variables['udp_port']['master_log']
        else:
            self.port = port

        kill_process_by_port(self.port)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.host, self.port))
        self.file_handler = RotatingFileHandler(log_dir, filename_prefix)
        self.running = True

    def start(self):
        """启动服务器接收日志。"""
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(16384)  # 接收数据
                log_message = data.decode()
                self.process_log_entry(log_message)
            except OSError as e:
                if e.errno == 10038:  # WSAENOTSOCK，即套接字已关闭
                    print('套接字已关闭，退出接收循环。')
                    break
                else:
                    raise

    def process_log_entry(self, log_message):
        """处理接收到的日志条目。"""
        to_server_flag, log_entry = log_message.split('|', 1)  # 分割to_server_flag和日志条目
        to_server_flag = to_server_flag in ['True', '1']  # 转换to_server_flag为布尔值

        if log_entry.startswith('{') and log_entry.endswith('}'):  # 判断是否为JSON
            log_entry = json.loads(log_entry)  # 转换为字典
            log_entry_str = json.dumps(log_entry, ensure_ascii=False)  # 重新转换为字符串以去除可能的额外空格等
        else:
            log_entry_str = log_entry  # 已经是字符串，直接使用

        # 写入日志文件（不包含to_server_flag）
        self.file_handler.write(log_entry_str)

        # 根据to_server_flag决定是否上传日志到云端
        if to_server_flag:
            self.upload_log_to_cloud(log_entry_str)

    def upload_log_to_cloud(self, log_entry_str):
        """模拟上传日志到云端。"""
        pass
        # url = 'http://your-cloud-log-service.com/upload'  # 替换为你的云端日志服务URL
        # headers = {'Content-Type': 'application/json'}  # 根据需要设置请求头
        # try:
        #     response = requests.post(url, data=log_entry_str, headers=headers)
        #     print(f'Uploaded log to cloud. Status code: {response.status_code}')
        # except requests.RequestException as e:
        #     print(f'Failed to upload log to cloud: {e}')

    def stop(self):
        """停止服务器。"""
        self.running = False
        kill_process_by_port(self.port)
        self.udp_socket.close()
        self.file_handler.close()


log_client = UDPLogClient()


def send_log(some_instance=None, log='Hello World!'):
    if some_instance is not None:
        log_stage = f'{some_instance.__class__.__name__}.{inspect.currentframe().f_back.f_code.co_name}'
    else:
        log_stage = inspect.currentframe().f_back.f_code.co_name

    log_client.send_log(log, log_stage)
