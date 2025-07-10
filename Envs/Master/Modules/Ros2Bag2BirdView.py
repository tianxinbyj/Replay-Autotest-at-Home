import copy
import glob
import json
import os

import shutil

from pathlib import Path

import cv2
from pandas.io.common import file_path_to_url
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg

from Envs.ReplayClient.Modules.BirdEyeView import euler2matrix, vector2matrix, matrix2euler, \
    DistortCameraObject, BirdEyeView
from Utils.Libs import get_project_path
import numpy as np
from scipy.spatial.transform import Rotation


class Ros2Bag2BirdView:

    def __init__(self, workspace, filepath, camera_config_path):
        self.timestamp_topic = '/Camera/SorroundFront/H265'
        self.topic_info ={
            '/Camera/FrontWide/H265':  'sensor_msgs/msg/CompressedImage',
            '/Camera/SorroundRight/H265': 'sensor_msgs/msg/CompressedImage',
            '/Camera/SorroundLeft/H265': 'sensor_msgs/msg/CompressedImage',
            '/Camera/SorroundFront/H265': 'sensor_msgs/msg/CompressedImage',
            '/Camera/SorroundRear/H265': 'sensor_msgs/msg/CompressedImage',
            '/Camera/Rear/H265': 'sensor_msgs/msg/CompressedImage'
        }
        self.cameras = []
        self.skip_frames = {key.split('/')[2]: 0 for key, value in self.topic_info.items()}
        self.total_frames = {key.split('/')[2]: 0 for key, value in self.topic_info.items()}
        self.timestamp = []
        self.image_count = 0
        self.file_path = filepath
        self.rosbag_path = os.path.join(self.file_path, 'ROSBAG', 'COMBINE')
        self.bev_path = os.path.join(self.file_path, 'PICTURE', 'BIRD_VIEW')
        self.jpg_path = os.path.join(self.file_path, 'PICTURE', 'FRAME')
        if len(glob.glob(os.path.join(camera_config_path, '*calibration.json'))) != 0:
            self.calibration_json = glob.glob(os.path.join(camera_config_path, '*calibration.json'))[0]
        else:
            self.calibration_json = None
        if len(glob.glob(os.path.join(camera_config_path, 'front.json'))) != 0:
            self.camera_json_path = camera_config_path
        else:
            self.camera_json_path = None
        self.bev_obj = self.getBevObj(self.calibration_json, self.camera_json_path)
        if not os.path.exists(self.rosbag_path):
            raise FileNotFoundError(f"{self.rosbag_path}不存在")
        self.h265_path = os.path.join(self.file_path, 'ROSBAG', 'H265')
        self.picture_path = os.path.join(self.file_path, 'PICTURE')
        self.struct_dict = None
        self.last_timestamp = None
        self.frame_id_saver = None
        self.time_saver = None
        self.workspace = workspace
        self.typestore = get_typestore(Stores.LATEST)
        self.struct_abbr_corr = {}
        msg_list = []
        for root, dirs, files in os.walk(self.workspace):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        for root, dirs, files in os.walk('/opt/ros/rolling'):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        for pathstr in msg_list:
            msg_path = Path(pathstr)
            msg_def = msg_path.read_text(encoding='utf-8')
            temp = get_types_from_msg(msg_def, self.getMsgType(msg_path))
            if list(temp.keys())[0] not in self.typestore.types.keys():
                self.typestore.register(temp)
        print(self.typestore.types.keys())

    def getMsgType(self, path: Path) -> str:
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    def getBevObj(self, calibration_json, camera_json_path):
        camera_parameter_folder = os.path.join(get_project_path(), 'Docs', 'Resources', 'camera_parameter')
        bird_json_file = os.path.join(camera_parameter_folder, 'bird_virtual', 'bird.json')
        if camera_json_path is None:
            distort_camera_dict = {}
            camera_vs = {
                'CAM_FRONT_120': ['front', 100],
                'CAM_BACK': ['rear', 101],
                'CAM_FISHEYE_FRONT': ['fisheye_front', 101],
                'CAM_FISHEYE_BACK': ['fisheye_rear', 101],
                'CAM_FISHEYE_LEFT': ['fisheye_left', 101],
                'CAM_FISHEYE_RIGHT': ['fisheye_right', 101],
            }

            G = np.eye(4)
            G[0:3, 0:3] = Rotation.from_euler('zyx', [0, -np.pi / 2, 0]).as_matrix() @ \
                          Rotation.from_euler('zyx', [0, 0, np.pi / 2]).as_matrix()  # pitch为90度，避免万向节锁，分两次转

            with open(calibration_json, 'r') as f:
                cameras = json.load(f)['camera']
            for origin_camera in cameras:
                # 拿6路相机参数，如果cameras里面的文件包含不止六路参数需要做判断再拿
                if origin_camera['name'] not in camera_vs:
                    continue
                camera_name = camera_vs[origin_camera['name']][0]
                template_json = os.path.join(camera_parameter_folder, 'json_calib', f'{camera_name}.json')
                with open(template_json, 'r') as f:
                    temp_camera = json.load(f)

                # 相机标定坐标系认为是固定参数，从4号车读取
                # 求外参数
                world2calib = euler2matrix(*temp_camera['vcs']['rotation'],
                                           *temp_camera['vcs']['translation'])
                calib2camera = np.linalg.inv(world2calib) @ vector2matrix(origin_camera['extrinsic_param']['rotation'],
                                                                          origin_camera['extrinsic_param']['translation'])
                roll, pitch, yaw, camera_x, camera_y, camera_z = matrix2euler(calib2camera @ G)

                # 求内参
                intrinsic_param = origin_camera['intrinsic_param']
                camera_model = intrinsic_param['camera_model']
                size = (intrinsic_param['camera_width'], intrinsic_param['camera_height'])
                extrinsic = np.linalg.inv(vector2matrix(origin_camera['extrinsic_param']['rotation'],
                                                        origin_camera['extrinsic_param']['translation']))
                intrinsic = np.zeros((3, 4))
                intrinsic[:, 0:3] = np.array(intrinsic_param['camera_matrix'])
                distort = np.array(intrinsic_param['distortion_coeffcients'])
                if camera_model == 'opencv_omni':
                    xi = np.array([float(intrinsic_param['xi'])])
                else:
                    xi = 0
                fov_range = [temp_camera['vcs']['rotation'][2] + yaw - np.deg2rad(temp_camera['fov']) / 2,
                             temp_camera['vcs']['rotation'][2] + yaw + np.deg2rad(temp_camera['fov']) / 2]
                camera_par = {
                    'size': size,
                    'extrinsic': extrinsic,
                    'intrinsic': intrinsic,
                    'distort': distort,
                    'xi': xi,
                    'fov_range': fov_range,
                }
                distort_camera = DistortCameraObject(camera_par=camera_par, camera_model=camera_model)
                distort_camera_dict[origin_camera['name']] = distort_camera

            BEV = BirdEyeView(bird_json_file=bird_json_file,
                              distort_camera_dict=distort_camera_dict,
                              bev_type='6_camera')
            self.cameras = list(distort_camera_dict.keys())
        else:
            camera_json_files = {
                'CAM_FRONT_120': [os.path.join(camera_json_path, 'front.json'), 'opencv_pinhole'],
                'CAM_BACK': [os.path.join(camera_json_path, 'rear.json'), 'opencv_pinhole'],
                'CAM_FISHEYE_FRONT': [os.path.join(camera_json_path, 'fisheye_front.json'), 'opencv_fisheye'],
                'CAM_FISHEYE_BACK': [os.path.join(camera_json_path, 'fisheye_rear.json'), 'opencv_fisheye'],
                'CAM_FISHEYE_LEFT': [os.path.join(camera_json_path, 'fisheye_left.json'), 'opencv_fisheye'],
                'CAM_FISHEYE_RIGHT': [os.path.join(camera_json_path, 'fisheye_right.json'), 'opencv_fisheye'],
            }
            BEV = BirdEyeView(bird_json_file=bird_json_file,
                              camera_json_files=camera_json_files,
                              bev_type='6_camera')
            self.cameras = list(camera_json_files.keys())

        return BEV

    def extract_h265_raw_streams(self):
        if os.path.exists(self.h265_path):
            shutil.rmtree(self.h265_path)
        os.makedirs(self.h265_path)
        for topic in self.topic_info.keys():
            h265_name = topic.split('/')[2]
            self.extract_h265_raw_stream(self.rosbag_path, os.path.join(self.h265_path, f'{h265_name}.h265'), topic)
            if topic == self.timestamp_topic:
                self.gen_timestamp(self.rosbag_path, topic)
        self.cal_frame_skip(self.rosbag_path, self.topic_info.keys())



    def extract_frames_from_h265(self):
        if os.path.exists(self.picture_path):
            shutil.rmtree(self.picture_path)
        os.makedirs(self.picture_path)
        if not os.path.exists(self.h265_path):
            raise FileNotFoundError(f"{self.h265_path}不存在")
        file_paths = []
        for root, directories, files in os.walk(self.h265_path):
            for filename in files:
                # 构建文件的绝对路径
                file_path = os.path.join(root, filename)
                file_paths.append(file_path)
        if len(file_paths) != len(self.topic_info):
            raise FileNotFoundError(f"{self.h265_path}内仅有{len(file_paths)}个文件")
        self.extract_frames(file_paths, self.picture_path)
        # self.test_extract_frames(file_paths, self.picture_path)
        if os.path.exists(self.h265_path):
            shutil.rmtree(self.h265_path)


    def gen_timestamp(self, bag_path, image_topic):
        """从ROS2的.db3文件中提取CompressedImage消息并保存为H.265裸流文件"""
        with Reader(bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == image_topic:
                    msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
                    self.timestamp.append(msg.header.stamp.sec * 1e9+ msg.header.stamp.nanosec)


    def cal_frame_skip(self, bag_path, topic_list):
        topic_start_time = self.skip_frames
        topic_list_copy = copy.deepcopy(list(topic_list))
        with Reader(bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in topic_list_copy:
                    msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
                    timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                    topic_start_time[connection.topic.split('/')[2]] = timestamp
                    topic_list_copy.remove(connection.topic)
                    if len(topic_list_copy) == 0:
                        break
        max_start_time = max(topic_start_time.values())
        for key, value in topic_start_time.items():
            self.skip_frames[key] = int((max_start_time - value) / 1e8)
        if self.timestamp_topic.split('/')[2] in self.skip_frames and self.skip_frames[self.timestamp_topic.split('/')[2]] != 0:
            self.timestamp = self.timestamp[self.skip_frames[self.timestamp_topic.split('/')[2]]:]



    def extract_h265_raw_stream(self, bag_path, output_h265_path, image_topic):
        """从ROS2的.db3文件中提取CompressedImage消息并保存为H.265裸流文件"""
        # 打开H.265裸流文件以写入二进制数据
        with open(output_h265_path, 'wb') as out_file:
            # 读取ROS2 bag文件
            with Reader(bag_path) as reader:
                count = 0
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == image_topic:
                        msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
                        # 将压缩图像数据直接写入文件
                        out_file.write(msg.data)
                        count += 1

                        if count % 100 == 0:
                            print(f"已处理 {count} 帧")
        self.total_frames[image_topic.split('/')[2]] = count
        print(f"处理完成! 共写入 {count} 帧到 {output_h265_path}")

    def extract_frames(self, video_name_list, output_dir):
        """
        从视频文件中提取每一帧并保存为图片

        参数:
        video_name_list (list): 输入视频文件路径
        output_dir (str): 输出图片目录
        prefix (str): 输出图片前缀
        format (str): 输出图片格式 (jpg, png等)
        quality (int): 图片质量 (0-100, 仅适用于jpg)
        """
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)

        caps = {}
        for video_name in video_name_list:

            # 打开视频文件
            cap = cv2.VideoCapture(video_name)

            if not cap.isOpened():
                print(f"无法打开视频文件: {video_name}")
                return
            h265_name = os.path.basename(video_name).split('.')[0]
            caps[h265_name] = cap
            # 获取视频信息
            print(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            total_frames =  self.total_frames[h265_name.split('.')[0]]
            fps = cap.get(cv2.CAP_PROP_FPS)
            print(f"视频信息: {total_frames} 帧, {fps:.2f} FPS")

        # 提取并保存每一帧
        frame_count = 0
        skip_frame_max = max(self.skip_frames.values())
        file_name = {
            'FrontWide': 'CAM_FRONT_120',
            'Rear': 'CAM_BACK',
            'SorroundFront': 'CAM_FISHEYE_FRONT',
            'SorroundRear': 'CAM_FISHEYE_BACK',
            'SorroundLeft': 'CAM_FISHEYE_LEFT',
            'SorroundRight': 'CAM_FISHEYE_RIGHT',
        }
        bev_folder = self.bev_path
        if os.path.exists(bev_folder):
            shutil.rmtree(bev_folder)
        os.makedirs(bev_folder)
        for i in range(len(self.timestamp)):
            if os.path.exists(self.jpg_path):
                shutil.rmtree(self.jpg_path)
            os.makedirs(self.jpg_path)
            images = {camera: None for camera in self.cameras}
            for name, cap in caps.items():
                skipped= self.skip_initial_frames(cap, name, frame_count)
                # skipped = False
                if skipped:
                    print(f"跳过了{name}摄像头的第{frame_count}帧")
                    continue
                elif frame_count > skip_frame_max:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    images[file_name[name]] = frame
            if all(value is not None for value in images.values()):
                # 拼图
                bev_jpg_path = self.bev_obj.getBev(images, bev_folder, rect_pts=None)
                os.rename(bev_jpg_path, os.path.join(os.path.dirname(bev_jpg_path), f'{self.timestamp[frame_count-skip_frame_max-1]:.0f}_BEV.jpg'))
            shutil.rmtree(self.jpg_path)
            # 显示进度
            if (frame_count-skip_frame_max-1) % 100 == 0:
                print(f"已处理: {frame_count-skip_frame_max-1}/{min(self.total_frames.values())}帧 ")

            frame_count += 1
        for name, cap in caps.items():
            cap.release()

    def skip_initial_frames(self, cap, camera_name, frame_count):
        """根据摄像头类型和帧计数决定是否跳过当前帧"""
        # 定义每种摄像头需要跳过的帧

        # 默认跳过所有帧（针对未定义的摄像头类型）
        # frames_to_skip = skip_frames.get(camera_name, float('inf'))

        # 如果当前帧计数小于等于需要跳过的帧数，则跳过
        if camera_name in self.skip_frames.keys() and self.skip_frames[camera_name] != 0 and frame_count + 1 <= self.skip_frames[camera_name]:
            ret, frame = cap.read()  # 读取并丢弃帧
            return True  # 返回True表示跳过了帧

        return False  # 返回False表示没有跳过帧


    def test_extract_frames(self, video_name_list, output_dir, prefix="frame", format="jpg", quality=95):
        """
        debug用

        参数:
        video_name_list (list): 输入视频文件路径
        output_dir (str): 输出图片目录
        prefix (str): 输出图片前缀
        format (str): 输出图片格式 (jpg, png等)
        quality (int): 图片质量 (0-100, 仅适用于jpg)
        """
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)

        caps = {}
        for video_name in video_name_list:

            # 打开视频文件
            cap = cv2.VideoCapture(video_name)

            if not cap.isOpened():
                print(f"无法打开视频文件: {video_name}")
                return
            h265_name = os.path.basename(video_name).split('.')[0]
            caps[h265_name] = cap
            # 获取视频信息
            print(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            print(f"视频信息: {total_frames} 帧, {fps:.2f} FPS")

        # 提取并保存每一帧
        frame_count = 0
        skip_frame_max = max(self.skip_frames.values())
        file_name = {
            'FrontWide': 'CAM_FRONT_120',
            'Rear': 'CAM_BACK',
            'SorroundFront': 'CAM_FISHEYE_FRONT',
            'SorroundRear': 'CAM_FISHEYE_BACK',
            'SorroundLeft': 'CAM_FISHEYE_LEFT',
            'SorroundRight': 'CAM_FISHEYE_RIGHT',
        }
        bev_folder = os.path.join(self.bev_path)
        if os.path.exists(bev_folder):
            shutil.rmtree(bev_folder)
        os.makedirs(bev_folder)
        time_stamp = '0'
        while True:
            if frame_count >= len(self.timestamp):
                break
            for name, cap in caps.items():
                skipped= self.skip_initial_frames(cap, name, frame_count)
                # skipped = False
                if skipped:
                    print(f"跳过了{name}摄像头的第{frame_count}帧")
                    continue
                elif frame_count > skip_frame_max:
                    time_stamp = str(self.timestamp[frame_count - skip_frame_max - 1])
                    ret, frame = cap.read()
                    if not ret:
                        break
                    # 构建输出文件名
                    if 1701329127834000128 < int(float(time_stamp)) < 1701329129834000128:
                        if not os.path.exists(os.path.join(self.jpg_path, time_stamp)):
                            os.makedirs(os.path.join(self.jpg_path, time_stamp))
                        frame_filename = f"{file_name[name]}.{format}"
                        frame_full_path = os.path.join(self.jpg_path, time_stamp, frame_filename)
                        # 根据格式设置保存参数
                        if format.lower() == "jpg":
                            cv2.imwrite(frame_full_path, frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                        else:
                            cv2.imwrite(frame_full_path, frame)

            if os.path.exists(os.path.join(self.jpg_path, time_stamp)):
                file_num = glob.glob(os.path.join(self.jpg_path, time_stamp, "*.jpg")) + glob.glob(os.path.join(self.jpg_path, time_stamp, "*.JPG"))
                if len(file_num) == 6:
                    # 拼图
                    images = {}
                    for camera in self.cameras:
                        images[camera] = os.path.join(self.jpg_path, time_stamp, f'{camera}.jpg')
                    bev_jpg_path = self.bev_obj.getBev(images, os.path.join(self.jpg_path, time_stamp), rect_pts=None)
                    # os.rename(bev_jpg_path, os.path.join(os.path.dirname(bev_jpg_path), f'{self.timestamp[str(frame_count)]:.0f}_BEV.jpg'))
                elif len(file_num) == 0:
                    print(f"处理完成! 共提取 {frame_count} 帧，保存在 {output_dir}")
                    break
                # shutil.rmtree(self.jpg_path)
                # 显示进度
                if frame_count % 100 == 0 or frame_count == total_frames - 1:
                    print(f"已处理: {frame_count}/{total_frames} 帧 ({frame_count / total_frames * 100:.2f}%)")

            frame_count += 1
        for name, cap in caps.items():
            cap.release()


if __name__ == '__main__':
    file_path_list = ['/home/hp/temp/20240123_145155_n000003']
    for file_path in file_path_list:
        file_path = '/home/hp/TESTDATA/20250529_102834_n000003'
        install_path = '/home/hp/artifacts/ZPD_EP39/4.3.0_RC11/install'
        # bev_folder = os.path.join(file_path, 'PICTURE', 'BIRD_VIEW')
        # bev_file_num = len(glob.glob(os.path.join(bev_folder, "*.jpg")) + glob.glob(os.path.join(bev_folder, "*.JPG")))
        r = Ros2Bag2BirdView(install_path, file_path, '/home/hp/config/json')
        r.extract_h265_raw_streams()
        # print(len(r.timestamp))
        # r.extract_h265_raw_stream('/home/hp/temp/20250324_144918_n000001/ROSBAG/COMBINE', '/home/hp/ZHX/read_can_signal/h265', '/Camera/FrontWide/H265')
        r.extract_frames_from_h265()
