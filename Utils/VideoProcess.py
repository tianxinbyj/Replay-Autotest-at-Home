"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import os
import shutil
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Utils.Libs import project_path


def image2video(image_folder, fps, video):
    cmd = f'ffmpeg -r {fps} -i "{image_folder}"/img%05d.jpg "{video}"'
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


if __name__ == '__main__':
    # 示例用法
    video_path = '/media/data/NI_Data/20240528_150013/Images/CAM_FRONT_120/n000002-2024-05-28-15-05-13-887_CAM_FRONT_120.mkv'
    # print(get_video_length(video_path))
