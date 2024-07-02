"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import os
import sys


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


if __name__ == '__main__':
    # 示例用法
    video_path = '/media/data/NI_Data/20240528_150013/Images/CAM_FRONT_120/n000002-2024-05-28-15-05-13-887_CAM_FRONT_120.mkv'
    pic_path = 'frame.jpg'
    extract_frame(video_path, 100, pic_path)
