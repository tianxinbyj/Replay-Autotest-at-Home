"""  
@Author: BU YUJUN
@Date: 2025/4/7 14:04  
"""
# !/usr/bin/env python3

import argparse
import os
import shutil
import subprocess

import pandas as pd
import rclpy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage
import av
import rosbag2_py
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata


class H265ToRosbagConverter:

    def __init__(self, topic, h265_path, timestamp_path, rosbag_path):
        # 初始化参数
        self.h265_path = h265_path  # 输入视频文件
        self.timestamp_path = timestamp_path
        self.rosbag_path = rosbag_path  # 输出bag文件夹
        self.topic = topic

        if os.path.exists(self.rosbag_path):
            shutil.rmtree(self.rosbag_path)

        # 创建rosbag并写入帧数据
        self.create_rosbag_with_timestamps()

    def create_rosbag_with_timestamps(self):
        print("Creating ROS bag with original timestamps...")

        # 设置bag存储选项
        storage_options = StorageOptions(
            uri=self.rosbag_path,
            storage_id='sqlite3'
        )

        # 设置转换选项
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        # 创建写入器
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        # 创建Topic信息
        topic_metadata = TopicMetadata(
            name=self.topic,
            type="sensor_msgs/msg/CompressedImage",
            serialization_format="cdr"
        )
        writer.create_topic(topic_metadata)

        # 打开转换后的H.265文件
        with open(self.h265_path, 'rb') as f:
            h265_data = f.read()
        codec = av.CodecContext.create('hevc', 'r')
        packets = codec.parse(h265_data)

        # 打开时间辍文件
        time_index = pd.read_csv(self.timestamp_path, index_col=False)

        for i, packet in enumerate(packets):
            if i == len(packets):
                break

            # 创建CompressedImage消息
            img_msg = CompressedImage()

            # 设置原始时间戳
            timestamp_sec = time_index.loc[i, 'time_stamp']
            img_msg.header.stamp.sec = int(timestamp_sec)
            img_msg.header.stamp.nanosec = int((timestamp_sec - int(timestamp_sec)) * 1e9)
            img_msg.header.frame_id = str(i)

            img_msg.format = "h265"
            img_msg.data = bytes(packet)  # 使用原始H.265数据包

            # 写入bag (使用原始时间戳作为ROS消息时间)
            writer.write(
                topic_metadata.name,
                serialize_message(img_msg),
                int(timestamp_sec * 1e9)  # 转换为纳秒
            )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="process raw data")
    parser.add_argument("-t", "--topic", type=str, required=True, help="topic")
    parser.add_argument("-f", "--h265_path", type=str, required=True, help="h265_path")
    parser.add_argument("-s", "--timestamp_path", type=str, required=True, help="timestamp_path")
    parser.add_argument("-r", "--rosbag_path", type=str, required=True, help="rosbag_path")
    args = parser.parse_args()

    rclpy.init()
    try:
        converter = H265ToRosbagConverter(args.topic, args.h265_path, args.timestamp_path, args.rosbag_path)
    finally:
        rclpy.shutdown()