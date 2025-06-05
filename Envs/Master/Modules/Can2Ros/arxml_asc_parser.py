import glob
import os
import datetime
import shutil

import cantools
import can
import sys
import argparse
from pathlib import Path

import pandas as pd

CAN_NAME_TO_CAN_FILE = {
    "ADAS_LocalCANFD1": "ADAS1",
    "ADAS_LocalCANFD2": "ADAS2",
    "ADAS_LocalCANFD3": "ADAS3",
    "BKPCANFD": "BKP",
    "CHCANFD": "CH",
    "IMU": "IMU",
    "IPS_Local_CANFD": "IPS",
    "PTCANFD": "PT",
    "SFCANFD": "Safety"
}
def all_dicts_contain_key(lst, key):
    return all(key in d for d in lst)


def parse_arguments():
    """解析命令行参数（支持信号列表）"""
    parser = argparse.ArgumentParser(description='根据ARXML文件解析ASC文件中的多个信号')
    parser.add_argument('-a', '--arxml', required=True, help='ARXML文件路径')
    parser.add_argument('-i', '--input', required=True, help='ASC输入文件路径')
    parser.add_argument('-o', '--output', help='CSV输出文件路径，默认为stdout')
    parser.add_argument('-s', '--signals', required=True, nargs='+', help='要解析的信号名列表（空格分隔）')
    return parser.parse_args()


def load_dbc_from_arxml(arxml_path):
    """从ARXML文件加载DBC对象"""
    try:
        db = cantools.database.load_file(arxml_path, strict=False)
        print(f"成功加载ARXML文件: {arxml_path}", file=sys.stderr)
        return db
    except Exception as e:
        print(f"加载ARXML文件失败: {e}", file=sys.stderr)
        sys.exit(1)

def add_frame_name_to_signal(db, signal_list_df):
    """查找信号名列表中每个信号所属的CAN总线（仅保留首次匹配）"""

    # 构建信号到消息的映射（信号名 -> CAN ID -> 消息对象）
    local_signal_names = signal_list_df.iloc[:, 0].tolist()
    local_signal_names = ['I' + s for s in local_signal_names]
    signal_with_frame_name = {signal: '' for signal in local_signal_names}
    signal_to_messages = {}
    for message in db.messages:
        if message.contained_messages:
            for sub_message in message.contained_messages:
                for signal in sub_message.signals:
                    if signal.name in local_signal_names:
                        bus_id = getattr(message, "bus_name", "Unknown")
                        if bus_id not in signal_to_messages.keys():
                            signal_to_messages[bus_id] = {}
                        if message.name not in signal_to_messages[bus_id].keys():
                            signal_to_messages[bus_id][message.name] = []
                        local_signal_names.remove(signal.name)
                        signal_with_frame_name[signal.name] = message.name
        if message.signals:
            for signal in message.signals:
                if signal.name in local_signal_names:
                    signal_with_frame_name[signal.name] = message.name
                    local_signal_names.remove(signal.name)

    # 打印未找到的信号
    if len(local_signal_names) != 0 :
        print(f"未找到以下信号: {', '.join(local_signal_names)}")

    return signal_with_frame_name


def find_signal_can_bus(db, signal_names):
    """查找信号名列表中每个信号所属的CAN总线（仅保留首次匹配）"""
    local_signal_names = set(signal_names)
    signal_to_messages = {}

    def process_signal(signal, message, parent_frame_id=None):
        if signal.name in local_signal_names:
            bus_id = getattr(message, "bus_name", "Unknown")
            can_id = message.frame_id

            # 初始化总线层级
            signal_to_messages.setdefault(bus_id, {})
            # 初始化消息层级
            if parent_frame_id is None:
                signal_to_messages[bus_id].setdefault(can_id, [])
                signal_to_messages[bus_id][can_id].append(signal.name)
            else:
                signal_to_messages[bus_id].setdefault(can_id, [])
                signal_to_messages[bus_id][can_id].append(signal.name)

            local_signal_names.discard(signal.name)

    for message in db.messages:
        if message.contained_messages:
            for sub_message in message.contained_messages:
                for signal in sub_message.signals:
                    process_signal(signal, message, sub_message.frame_id)
        if message.signals:
            for signal in message.signals:
                process_signal(signal, message)

    # 打印未找到的信号
    if local_signal_names:
        print(f"未找到以下信号: {', '.join(local_signal_names)}")

    return signal_to_messages


def parse_asc_file(asc_path, db, signal_names_with_frame_name, target_signal_info):
    """解析ASC文件并提取多个信号"""
    signal_names = list(signal_names_with_frame_name.keys())

    data_key = ['timestamp'] + signal_names
    raw_data = {signal: [] for signal in data_key}

    try:
        for can_bus in target_signal_info.keys():
            asc_files = glob.glob(os.path.join(asc_path, CAN_NAME_TO_CAN_FILE[can_bus[0:-8]], "*.asc"))
            if len(asc_files) != 1:
                raise ValueError("文件夹中.asc文件数量不为1！")
            with can.ASCReader(asc_files[0]) as asc_reader:
                message_count = 0
                for msg in asc_reader:
                    message_count += 1
                    # 记录当前消息中解析到的信号
                    updated_signals = set()
                    if msg.arbitration_id in target_signal_info[can_bus].keys():
                        last_values = {signal: None for signal in signal_names}
                        # 添加时间戳
                        raw_data['timestamp'].append(msg.timestamp)
                        try:
                            frame_name = signal_names_with_frame_name[target_signal_info[can_bus][msg.arbitration_id][0]]
                            decoded = db.decode_message(frame_name, msg.data, decode_containers=True)

                            if isinstance(decoded, dict):
                                # decoded = db.decode_message(frame_name, msg.data)
                                for signal_name in target_signal_info[can_bus][msg.arbitration_id]:
                                    try:
                                        signal_value = decoded.get(signal_name, None)

                                        # 处理枚举值（若为NamedSignalValue，提取数值）
                                        if isinstance(signal_value, cantools.database.namedsignalvalue.NamedSignalValue):
                                            signal_value = signal_value.value
                                        # 更新该信号的上一个值
                                        last_values[signal_name] = signal_value
                                        updated_signals.add(signal_name)

                                    except Exception as e:
                                        print(f"解码信号 '{signal_name}' 失败 - CAN ID: 0x{msg.arbitration_id:X}, 错误: {str(e)}",
                                              file=sys.stderr)
                            else:
                                # decoded = db.decode_message(frame_name, msg.data, decode_containers=True)
                                for message in decoded:
                                    for signal_name in message[1]:
                                        if signal_name in target_signal_info[can_bus][msg.arbitration_id]:
                                            try:
                                                signal_value = message[1][signal_name]

                                                # 处理枚举值（若为NamedSignalValue，提取数值）
                                                if isinstance(signal_value,
                                                              cantools.database.namedsignalvalue.NamedSignalValue):
                                                    signal_value = signal_value.value
                                                # 更新该信号的上一个值
                                                last_values[signal_name] = signal_value
                                                updated_signals.add(signal_name)

                                            except Exception as e:
                                                print(
                                                    f"解码信号 '{signal_name}' 失败 - CAN ID: 0x{msg.arbitration_id:X}, 错误: {str(e)}",
                                                    file=sys.stderr)

                        except Exception as e:
                            print(f"解码0x{msg.arbitration_id:X}失败, 错误: {str(e)}",
                                  file=sys.stderr)

                        # 为所有信号添加值（更新的信号用新值，未更新的信号用上一次的值）
                        for s in signal_names:
                            raw_data[s].append(last_values[s])
                print(f"ASC文件{asc_files[0]}解析完成: {message_count} 条消息", file=sys.stderr)
    except Exception as e:
        print(f"解析ASC文件失败: {e}", file=sys.stderr)
        sys.exit(1)
    data = data_processing(raw_data)
    return data


def save_to_csv(df, output_path):
    """将结果写入CSV（每行一个信号记录）"""
    if not os.path.exists(os.path.dirname(output_path)):
        os.mkdir(os.path.dirname(output_path))
    if os.path.exists(output_path):
        os.remove(output_path)
    if df.empty:
        print("警告: 未找到任何有效信号数据", file=sys.stderr)
        return
    df.to_csv(output_path, index=False, encoding="utf-8")
    print(f"结果已保存到 {output_path}，共 {len(df)} 条记录", file=sys.stderr)


def data_processing(raw_data):
    # 直接使用原始数据创建DataFrame，避免中间副本
    df = pd.DataFrame(raw_data)

    # 先处理时间戳，避免后续操作影响索引
    df.sort_values('timestamp', inplace=True, ignore_index=True)

    # 找出全为NaN的列
    signals_no_value = []
    for col in df.columns:
        if col == 'timestamp':
            continue
        if df[col].isna().all():
            signals_no_value.append(col)

    # 一次性删除所有全NaN的列，减少内存碎片
    if signals_no_value:
        df.drop(columns=signals_no_value, inplace=True)
        print(f"以下信号在文件内没有找到: {signals_no_value}")

    # 对剩余列进行填充
    for col in df.columns:
        if col == 'timestamp':
            continue
        # 向前填充NaN
        df[col].ffill(inplace=True)
        # 填充最前面可能存在的NaN
        df[col].fillna(0, inplace=True)

    return df


def asc_parser(data='TestData/CAN_Trace', csv_path='/home/hp/ZHX/read_can_signal/CAN_Signal.csv', arxml='arxml/20240315-cgy-ES37_IPD_V6.0.arxml'):
    """主函数"""
    signal_list_df = pd.read_csv(csv_path, header=None, index_col=0)  # 仅读取第一列
    # arxml = 'arxml/20240315-cgy-ES37_IPD_V6.0.arxml'
    # data = 'TestData/CAN_Trace'
    # signal_list = ['IVehSpdAvg', 'IACCSysSts', 'ILDWLKASwReq', 'IVehSpdAvgCRC', 'IIpsSts']
    signal_list = signal_list_df[1].to_dict()
    now = datetime.datetime.now()
    timestamp_str = now.strftime("%Y-%m-%d_%H-%M-%S")
    asc_data_path = os.path.join(os.path.dirname(data), 'ASCParseData')
    if os.path.exists(asc_data_path):
        shutil.rmtree(asc_data_path)
    os.makedirs(asc_data_path)
    output = os.path.join(asc_data_path, f'{os.path.basename(data)}.csv')


    # 检查文件是否存在
    arxml_path = Path(arxml)
    input_path = Path(data)

    if not arxml_path.exists():
        print(f"错误: ARXML文件不存在: {arxml_path}", file=sys.stderr)
        sys.exit(1)

    if not input_path.exists():
        print(f"错误: 输入ASC文件不存在: {input_path}", file=sys.stderr)
        sys.exit(1)

    # 加载数据库
    db = load_dbc_from_arxml(arxml_path)
    t = find_signal_can_bus(db, signal_list.keys())

    # 解析ASC文件
    print(f"正在解析ASC文件: {input_path}", file=sys.stderr)
    results = parse_asc_file(data, db, signal_list, t)

    # 输出结果
    save_to_csv(results, output)
    return output


if __name__ == "__main__":
    asc_parser()
    # csv_path = 'output.csv'
    # signal_list_df = pd.read_csv(csv_path)  # 仅读取第一列
    # db = load_dbc_from_arxml('arxml/20240315-cgy-ES37_IPD_V6.0.arxml')
    # t = add_frame_name_to_signal(db, signal_list_df)
    # df = pd.DataFrame({
    #     '列1': list(t.keys()),
    #     '列2': list(t.values())
    # })
    # # signal_list_df = signal_list_df.drop_duplicates()
    # df.to_csv(csv_path, index=False, header=False)
    # db = load_dbc_from_arxml('arxml/20240315-cgy-ES37_IPD_V6.0.arxml')
    # t = find_signal_can_bus__(db, ['IBrkSysHillStAstSts'])
    # with can.ASCReader('/home/hp/ZHX/can_signal_test/arxml/CAN_Trace/CHCANFD/n000001_2025-01-15-14-23-21-872_CH.asc') as asc_reader:
    #     message_count = 0
    #     for msg in asc_reader:
    #         message_count += 1
    #         if msg.arbitration_id == 423:
    #             message_def = db.get_message_by_frame_id('BrkSysHillStAstSts')
    #             decoded = db.decode_message(message_def.name, msg.data, allow_truncated=True)
    #             print(decoded)
    # t = find_signal_can_bus(db, ['IVehSpdAvg', 'ITrShftLvrPosV', 'ITrShftLvrPos'])
    # parse_asc_file('arxml/CAN_Trace', db, ['IVehSpdAvg', 'ITrShftLvrPosV', 'ITrShftLvrPos'])