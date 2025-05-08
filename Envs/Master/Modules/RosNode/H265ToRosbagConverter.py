"""  
@Author: BU YUJUN
@Date: 2025/4/7 14:04  
"""

import argparse
import os
import shutil

import pandas as pd
import rclpy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage
import rosbag2_py
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata


class H265ToRosbagConverter:

    def __init__(self, topic, h265_path, timestamp_path, rosbag_path):

        # 初始化ROS 2上下文（无节点）
        rclpy.init()

        if os.path.exists(rosbag_path):
            shutil.rmtree(rosbag_path)

        # 配置输出.db3文件
        self.writer = SequentialWriter()
        storage_options = StorageOptions(
            uri=rosbag_path,
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.writer.open(storage_options, converter_options)

        # 创建Topic信息
        topic_metadata = TopicMetadata(
            name=topic,
            type="sensor_msgs/msg/CompressedImage",
            serialization_format="cdr"
        )
        self.writer.create_topic(topic_metadata)

        # 打开转换后的H.265文件
        self.h265_path = h265_path
        self.topic = topic

        # 打开时间辍文件
        self.time_index = pd.read_csv(timestamp_path, index_col=False)

    def parse_h265_frames(self):
        """解析H.265裸流文件，分离出各帧"""
        with open(self.h265_path, 'rb') as f:
            data = f.read()

        frames = []
        start = 0

        # 查找NAL单元起始码 (0x00000001)
        while True:
            # 查找下一个起始码
            pos = data.find(b'\x00\x00\x00\x01', start + 1)
            if pos == -1:
                # 添加最后一帧
                frames.append(data[start:])
                break
            # 添加当前帧
            frames.append(data[start:pos])
            start = pos

        return frames

    def process_h265_file(self):
        for i, frame in enumerate(self.parse_h265_frames()):
            self._write_to_bag(frame, i)

        print(f"Processed {i} frames to rosbag2.")

    def _write_to_bag(self, nal_data, frame_id):
        if frame_id < len(self.time_index):
            time_stamp = self.time_index.at[frame_id, 'time_stamp']
            msg = CompressedImage()
            msg.header.stamp.sec = int(time_stamp)
            msg.header.stamp.nanosec = int(time_stamp*1e9 - int(time_stamp)*1e9)
            msg.header.frame_id = f'frame_{frame_id}'
            msg.format = 'h265'
            msg.data = bytes(nal_data)
            print(time_stamp, len(msg.data.tolist()), msg.data.tolist()[:20])

            # 写入数据库
            self.writer.write(
                self.topic,
                serialize_message(msg),
                int(time_stamp*1e9),
            )

    def close(self):
        del self.writer
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="process raw data")
    parser.add_argument("-t", "--topic", type=str, required=True, help="topic")
    parser.add_argument("-f", "--h265_path", type=str, required=True, help="h265_path")
    parser.add_argument("-s", "--timestamp_path", type=str, required=True, help="timestamp_path")
    parser.add_argument("-r", "--rosbag_path", type=str, required=True, help="rosbag_path")
    args = parser.parse_args()

    converter = H265ToRosbagConverter(args.topic, args.h265_path, args.timestamp_path, args.rosbag_path)
    try:
        converter.process_h265_file()  # 替换为你的输入文件
    finally:
        converter.close()