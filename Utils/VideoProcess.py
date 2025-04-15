"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import os
import re
import shutil
import subprocess
import sys

import av
import pandas as pd
import numpy as np


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


def convert_video_h265(video_path, target_fps):
    h265_path = os.path.join(project_path, 'Temp', f'{os.path.basename(video_path).split(".")[0]}.h265')

    video_convert_command = [
        'ffmpeg',
        '-i', video_path,
        '-c:v', 'libx265',  # H.265编码
        '-pix_fmt', 'yuv420p',  # 像素格式
        '-r', str(target_fps),  # 目标帧率
        '-x265-params', f'keyint={target_fps}:min-keyint={target_fps}',  # 每秒一个I帧
        '-preset', 'medium',  # 编码速度与质量平衡
        '-crf', '23',  # 恒定质量模式，值越小质量越高（18通常是无损或接近无损）
        '-f', 'hevc',  # 输出格式为HEVC裸流
        '-an',  # 不处理音频
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


def gen_h265_timestamp(h265_path):
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


if __name__ == '__main__':

    video_path = '/home/byj/ZONE/debug/origin60.mkv'
    convert_video_h265(video_path, 15)

    h265_path = '/home/byj/ZONE/PythonProject/Replay-Autotest-at-Home/Temp/origin60.h265'
    gen_h265_timestamp(h265_path)
    with open(h265_path, 'rb') as f:
        h265_data = f.read()

    # 使用FFmpeg解析H.265裸流
    codec = av.CodecContext.create('hevc', 'r')

    # 解析H.265数据包
    packets = codec.parse(h265_data)

    for i, packet in enumerate(packets):
        dd = np.frombuffer(packet, dtype=np.uint8).tolist()
        print(len(dd), dd[:30])
        # print('=======================')