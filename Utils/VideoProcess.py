"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import os
import re
import shutil
import subprocess
import sys

import pandas as pd

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Utils.Libs import project_path


def image2video(image_folder, fps, video, width=None, height=None):
    if width is None:
        cmd = f'ffmpeg -r {fps} -i "{image_folder}"/img%05d.jpg -c:v libx265 -b:v 2000k -preset fast -crf 28 "{video}"'
    else:
        width = round(width) + round(width) % 2
        height = round(height) + round(height) % 2
        cmd = f'ffmpeg -framerate {fps} -i "{image_folder}"/img%05d.jpg -s {width}x{height} -b:v 2M -crf 28 -pix_fmt yuv420p "{video}"'

    print(cmd)
    os.system(cmd)


def video2image(video, fps, image_folder):
    cmd = f'ffmpeg -i "{video}" -f image2 -vf fps={fps} -qscale:v 2 "{image_folder}"/img%05d.jpg'
    os.system(cmd)


def extract_frame(video_path, frame_number, pic_path):
    # 使用OpenCV读取视频
    import cv2
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit()

    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    success, frame = cap.read()
    if success:
        cv2.imwrite(pic_path, frame)

    cap.release()
    return pic_path


def extract_frames(video_path, frame_list, pic_folder=None):
    if pic_folder is None:
        pic_folder = os.path.join(project_path, 'Temp')

    if os.path.exists(pic_folder):
        shutil.rmtree(pic_folder)
    os.makedirs(pic_folder)

    # 使用OpenCV读取视频
    import cv2
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit()

    saved_images = []

    for frame_count in frame_list:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_count)
        ret, frame = cap.read()
        if ret:
            file_id = os.path.join(pic_folder, f'{frame_count}.jpg')
            cv2.imwrite(file_id, frame)
            saved_images.append(file_id)

    cap.release()  # 释放视频文件资源
    return pic_folder


def parse_video(video_path):
    import cv2

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit()

    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = 0
    while True:
        success, _ = cap.read()
        if not success:
            break
        frame_count += 1

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    duration = frame_count / fps
    cap.release()

    print(fps, duration)
    return fps, duration


def convert_video_h265(video_path, target_fps, h265_path=None):
    if not h265_path:
        h265_path = os.path.join(project_path, 'Temp', f'{os.path.basename(video_path).split(".")[0]}.h265')

    video_convert_command = [
        'ffmpeg',
        '-i', video_path,  # 输入文件
        '-c:v', 'libx265',  # 使用H.265编码
        '-pix_fmt', 'yuv420p',  # 像素格式为yuv420p
        '-r', f'{target_fps}',  # 输出帧率15Hz
        '-x265-params', f'keyint={target_fps}:min-keyint={target_fps}:bframes=0',  # 每秒一个I帧，无B帧
        '-vcodec', 'hevc',  # 视频编码器
        '-an',  # 去除音频
        '-f', 'hevc',  # 强制输出格式为HEVC
        '-bsf:v', 'hevc_mp4toannexb',  # 比特流过滤器，确保NAL单元分隔符正确
        '-y',  # 覆盖输出文件
        h265_path
    ]

    try:
        subprocess.run(video_convert_command, check=True)
        print("Video conversion completed successfully.")
        return h265_path
    except subprocess.CalledProcessError as e:
        print(f"Video conversion failed: {e}")
        return None


def gen_h265_timestamp(h265_path, timestamp_path=None):
    if not timestamp_path:
        timestamp_path = os.path.join(project_path, 'Temp', f'{os.path.basename(h265_path).split(".")[0]}.csv')

    cmd = [
        "ffmpeg",
        "-i", h265_path,
        "-vf", "showinfo",
        "-f", "null",
        "-"
    ]
    process = subprocess.Popen(
        cmd,
        stderr=subprocess.PIPE,  # FFmpeg 输出到 stderr
        universal_newlines=True  # 确保文本模式
    )

    # 优化后的正则表达式（精确匹配实际输出格式）
    pattern = re.compile(
        r"n:\s*(\d+).*?pts_time:([0-9.e+-]+).*?type:([IBP])\b"
    )

    col = ['frame_index', 'time_stamp', 'frame_type']
    rows = []
    for line in process.stderr:
        match = pattern.search(line)
        if match:
            frame_index = int(match.group(1))
            pts_time = float(match.group(2))
            frame_type = match.group(3)
            rows.append([frame_index, pts_time, frame_type])

    pd.DataFrame(rows, columns=col).to_csv(timestamp_path, index=False)
    return timestamp_path


def verify_all_delimiters(file_path):
    """
    验证所有NAL单元分隔符是否为0x00000001
    :return: True如果全部符合要求，False如果有不符合的
    """
    with open(file_path, 'rb') as f:
        data = f.read()

    pos = 0
    all_valid = True

    while pos < len(data) - 4:
        # 找到下一个起始码
        next_pos = data.find(b'\x00\x00\x00\x01', pos)
        if next_pos == -1:
            next_pos = data.find(b'\x00\x00\x01', pos)
            if next_pos != -1:
                print(f"在位置 {next_pos} 发现不符合要求的分隔符: 0x000001")
                all_valid = False
            break

        # 检查当前位置到找到位置之间是否有3字节起始码
        between = data[pos:next_pos]
        if b'\x00\x00\x01' in between:
            invalid_pos = pos + between.find(b'\x00\x00\x01')
            print(f"在位置 {invalid_pos} 发现不符合要求的分隔符: 0x000001")
            all_valid = False

        pos = next_pos + 4

    if all_valid:
        print("所有NAL单元分隔符都是0x00000001，符合要求")
    else:
        print("发现不符合要求的分隔符")

    return all_valid


def normalize_h265_startcodes(input_file, output_file):
    """
    将H.265裸流文件的所有NAL单元分隔符统一为 b'\x00\x00\x00\x01'
    """
    with open(input_file, 'rb') as f:
        data = f.read()

    # NAL单元起始码可以是 00 00 01 或 00 00 00 01
    start_code_3 = b'\x00\x00\x01'
    start_code_4 = b'\x00\x00\x00\x01'

    i = 0
    nal_units = []

    while i < len(data):
        # 检查是否匹配4字节起始码
        if i + 4 <= len(data) and data[i:i+4] == start_code_4:
            nal_start = i + 4
            i += 4
        # 检查是否匹配3字节起始码
        elif i + 3 <= len(data) and data[i:i+3] == start_code_3:
            nal_start = i + 3
            i += 3
        else:
            i += 1
            continue

        # 查找下一个起始码，确定当前NAL单元结束位置
        j = i
        while j < len(data):
            if j + 4 <= len(data) and data[j:j+4] == start_code_4:
                break
            if j + 3 <= len(data) and data[j:j+3] == start_code_3:
                break
            j += 1

        nal_unit = data[nal_start:j]
        nal_units.append(nal_unit)
        i = j

    # 将所有NAL单元用 b'\x00\x00\x00\x01' 拼接
    new_data = start_code_4.join([b""] + nal_units)

    # 写入新文件
    with open(output_file, 'wb') as f:
        f.write(new_data)

    print(f"转换完成！新文件已保存为: {output_file}")


if __name__ == '__main__':

    # video_path = '/home/byj/ZONE/debug/origin60.mkv'
    h265_path = '/home/byj/ZONE/debug/origin60.h265'
    # convert_video_h265(video_path, 15, h265_path)
    #
    # timestamp_path = '/home/byj/ZONE/debug/origin60.csv'
    # gen_h265_timestamp(h265_path, timestamp_path)

    output_file = '/home/byj/ZONE/debug/output_normalized.h265'  # 输出文件（所有分隔符统一为 00 00 00 01）
    # normalize_h265_startcodes(h265_path, output_file)
    verify_all_delimiters(output_file)
