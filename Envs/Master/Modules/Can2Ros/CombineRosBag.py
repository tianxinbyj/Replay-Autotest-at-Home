#!/usr/bin/env python3
import argparse
import os
import shutil
import time

from rclpy.time import Time
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata

def merge_rosbags(input_bags, output_bag):
    # 初始化读取器和写入器
    if os.path.exists(output_bag):
        shutil.rmtree(output_bag)
    readers = []
    for bag_path in input_bags:
        reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)
        readers.append(reader)

    # 创建输出bag
    writer = SequentialWriter()
    writer_storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    writer.open(writer_storage_options, ConverterOptions('', ''))

    # 收集所有主题
    all_topics = {}
    for reader in readers:
        for topic in reader.get_all_topics_and_types():
            if topic.name not in all_topics:
                all_topics[topic.name] = topic
                writer.create_topic(topic)
            elif topic.type != all_topics[topic.name].type:
                raise ValueError(f"Topic {topic.name} has conflicting types: "
                                f"{topic.type} vs {all_topics[topic.name].type}")

    # 初始化每个reader的消息队列
    next_messages = []
    for reader in readers:
        if reader.has_next():
            (topic, data, t) = reader.read_next()
            next_messages.append((Time(nanoseconds=t), topic, data, reader))
        else:
            next_messages.append(None)

    # 按时间顺序合并消息
    while any(msg is not None for msg in next_messages):
        # 找出最早的消息
        earliest_time = None
        earliest_index = -1
        for i, msg in enumerate(next_messages):
            if msg is not None:
                if earliest_time is None or msg[0] < earliest_time:
                    earliest_time = msg[0]
                    earliest_index = i

        # 写入最早的消息
        if earliest_index != -1:
            time, topic, data, reader = next_messages[earliest_index]
            writer.write(topic, data, time.nanoseconds)

            # 获取下一条消息
            if reader.has_next():
                next_topic, next_data, next_t = reader.read_next()
                next_messages[earliest_index] = (Time(nanoseconds=next_t), next_topic, next_data, reader)
            else:
                next_messages[earliest_index] = None

    print(f"合并完成，输出文件: {output_bag}")

def main():
    parser = argparse.ArgumentParser(description='Merge ROS2 bags with time alignment')
    parser.add_argument('-i', '--input', nargs='+', required=True, help='Input ROS2 bag paths')
    parser.add_argument('-o', '--output', required=True, help='Output ROS2 bag path')
    args = parser.parse_args()

    # 检查输入文件是否存在
    for bag in args.input:
        if not os.path.exists(bag):
            print(f"错误: 输入文件 {bag} 不存在")
            return

    # 检查输出目录是否存在
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    try:
        merge_rosbags(args.input, args.output)
    except Exception as e:
        print(f"合并过程中发生错误: {str(e)}")

if __name__ == '__main__':
    main()