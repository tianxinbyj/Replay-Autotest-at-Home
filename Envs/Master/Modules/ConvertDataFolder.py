import glob
import os
import re
import shutil


def extract_date_time_from_filename(filename):
    """
    从MKV文件名中提取日期时间信息并格式化为指定格式

    参数:
    filename -- MKV文件名

    返回:
    格式化后的日期时间字符串，格式为YYYYMMDD_HHMMSS
    """
    try:
        # 方法1: 使用正则表达式匹配日期时间模式
        # 模式解释: 匹配类似2025-05-29-13-19-34的日期时间部分
        filename = filename.split('.')[0]
        pattern = r'(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})'
        match = re.search(pattern, filename)

        if match:
            year, month, day, hour, minute, second = match.groups()
            # 格式化为YYYYMMDD_HHMMSS
            return f"{year}{month}{day}_{hour}{minute}{second}"

        # 如果正则表达式未匹配到，尝试方法2: 基于字符串分割
        # 假设文件名中包含类似2025-05-29-13-19-34的部分
        parts = filename.split('-')
        if len(parts) >= 6 and all(p.isdigit() for p in parts[:6]):
            year, month, day, hour, minute, second = parts[:6]
            return f"{year}{month}{day}_{hour}{minute}{second}"

        # 如果以上方法都失败，返回None或错误信息
        return "无法从文件名中提取日期时间信息"

    except Exception as e:
        return f"处理过程中发生错误: {str(e)}"


def create_data_folder(file_path: str):
    folders_under_can_trace = ['ADAS1', 'ADAS2', 'ADAS3', 'BKP', 'CH', 'IMU', 'IPS', 'PT', 'Safety']
    folders_under_can_images = ['CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT', 'CAM_FISHEYE_BACK',
                                'CAM_FISHEYE_FRONT', 'CAM_FISHEYE_LEFT', 'CAM_FISHEYE_RIGHT', 'CAM_FRONT_30',
                                'CAM_FRONT_120', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT']
    can_trace_folder_path = os.path.join(file_path, 'CAN_Trace')
    images_folder_path = os.path.join(file_path, 'Images')
    all_folder_path = {}
    for folder in folders_under_can_trace:
        all_folder_path[folder] = os.path.join(can_trace_folder_path, folder)
        if not os.path.exists(all_folder_path[folder]):
            os.makedirs(all_folder_path[folder], exist_ok=False)
    for folder in folders_under_can_images:
        all_folder_path[folder] = os.path.join(images_folder_path, folder)
        if not os.path.exists(all_folder_path[folder]):
            os.makedirs(all_folder_path[folder], exist_ok=False)
    # if not os.path.exists(config_folder_path):
    #     os.makedirs(config_folder_path, exist_ok=False)
    return all_folder_path

def get_all_folder_in_target_dir(src_dir: str):
    folder_list = []
    for root, dirs, files in os.walk(src_dir):
        for dir_name in dirs:
            folder_list.append(os.path.join(root, dir_name))
    return folder_list

def get_all_file_in_target_path(src_dir: str):
    can_trace_folder_path = get_all_folder_in_target_dir(os.path.join(src_dir, 'CAN_Trace'))
    images_folder_path = get_all_folder_in_target_dir(os.path.join(src_dir, 'Images'))
    raw_data_all_folder_path = {}
    for folder in can_trace_folder_path:
        raw_data_all_folder_path[os.path.basename(folder)] = glob.glob(os.path.join(folder, '*'))
    for folder in images_folder_path:
        raw_data_all_folder_path[os.path.basename(folder)] = glob.glob(os.path.join(folder, '*'))
    return raw_data_all_folder_path


def copy_data_file_to_target(raw_data_all_folder_path: dict, all_folder_path: dict, file_path: str):
    tag_num = file_path.split('_')[-1]
    for folder_name, folder_path in all_folder_path.items():
        for file in raw_data_all_folder_path[folder_name]:
            file_name = os.path.basename(file)
            if file_name.startswith(tag_num):
                shutil.move(file, folder_path)

def reverse_data_folder(src_dir, dst_dir):
    front_120_files_name = glob.glob(os.path.join(src_dir, 'Images', 'CAM_FRONT_120', '*'))
    folder_num = len(front_120_files_name)
    if folder_num != 0:
        folder_start_name = extract_date_time_from_filename(os.path.basename(front_120_files_name[0]))
        raw_data_all_folder_path = get_all_file_in_target_path(
            src_dir=src_dir
        )
        for i in range(folder_num):
            folder_end_name = os.path.basename(front_120_files_name[i]).split('_')[0]
            file_path = os.path.join(dst_dir, f"{folder_start_name}_{folder_end_name}")
            all_folder_path = create_data_folder(
                file_path=file_path
            )
            copy_data_file_to_target(raw_data_all_folder_path, all_folder_path, file_path=file_path)
            if os.path.exists(os.path.join(src_dir, file_path, 'Config')):
                shutil.rmtree(os.path.join(src_dir, file_path, 'Config'))
            shutil.copytree(os.path.join(src_dir, 'Config'), os.path.join(src_dir, file_path, 'Config'))
    if os.path.exists(os.path.join(src_dir, 'Config')):
        shutil.rmtree(os.path.join(src_dir, 'Config'))
    if os.path.exists(os.path.join(src_dir, 'Images')):
        shutil.rmtree(os.path.join(src_dir, 'Images'))
    if os.path.exists(os.path.join(src_dir, 'CAN_Trace')):
        shutil.rmtree(os.path.join(src_dir, 'CAN_Trace'))
    if os.path.exists(os.path.join(src_dir, 'Lidar')):
        shutil.rmtree(os.path.join(src_dir, 'Lidar'))
    if os.path.exists(os.path.join(src_dir, 'Tag')):
        shutil.rmtree(os.path.join(src_dir, 'Tag'))



# 示例用法
if __name__ == "__main__":
    # 示例 1：使用完整文件路径
    reverse_data_folder(
        src_dir="/home/hp/temp/data",
        dst_dir = ""
    )