#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : BirdEyeView.py
# @Time         : 4/12/23 3:13 PM
# @Author      : Bu Yujun

import json
import os
import shutil

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from Envs.Master.Modules.Libs import get_project_path
camera_parameter_folder = os.path.join(get_project_path(), 'Docs', 'Resources', 'camera_parameter')


# 根据内外参计算世界坐标系到相机坐标系
def world2camera(intrinsic, extrinsic, x, y, z):
    res = extrinsic @ np.array([[x], [y], [z], [1]])
    res = intrinsic @ res / res[2][0]
    return res[0][0], res[1][0]


def changeCvDataShape(data, direction='1*N*mtoN*1*m'):
    shape = data.shape
    target = np.zeros((shape[1], shape[0], shape[2]))
    if direction == '1*N*mtoN*1*m':
        for i, d in enumerate(data[0]):
            target[i, 0, :] = d
    else:
        for i, d in enumerate(data):
            target[0, i, :] = d[0]
    return target


def get_RPY(roll, pitch, yaw, x, y, z):
    Rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll), -np.sin(roll), 0],
        [0, np.sin(roll), np.cos(roll), 0],
        [0, 0, 0, 1]])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1]])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    T = np.array([
        [0, 0, 0, x],
        [0, 0, 0, y],
        [0, 0, 0, z],
        [0, 0, 0, 0],
    ])

    return Rx @ Ry @ Rz + T


def euler2matrix(roll, pitch, yaw, x, y, z, seq='XYZ'):
    R = np.eye(4)
    R[0:3, 3] = [x, y, z]
    x_index = seq.lower().index('x')
    y_index = seq.lower().index('y')
    z_index = seq.lower().index('z')
    eulers = [0, 0, 0]
    eulers[x_index] = roll
    eulers[y_index] = pitch
    eulers[z_index] = yaw
    R[0:3, 0:3] = Rotation.from_euler(seq, eulers).as_matrix()
    return R


def vector2matrix(rotation, translation):
    R = np.eye(4)
    R[0:3, 3] = translation
    R[0:3, 0:3] = Rotation.from_rotvec(rotation).as_matrix()
    return R


def matrix2euler(R, seq='XYZ'):
    x = R[0, 3]
    y = R[1, 3]
    z = R[2, 3]
    eulers = Rotation.from_matrix(R[0:3, 0:3]).as_euler(seq)
    x_index = seq.lower().index('x')
    y_index = seq.lower().index('y')
    z_index = seq.lower().index('z')
    return eulers[x_index], eulers[y_index], eulers[z_index], x, y, z


def rotationMatrixToEulerAngles(R):
    def isRotationMatrix(R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    assert (isRotationMatrix(R))

    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


class ConvertJsonFile:

    def __init__(self, calibration_json, output_folder):
        self.j5A_folder = os.path.join(output_folder, '100')
        if not os.path.exists(self.j5A_folder):
            os.mkdir(self.j5A_folder)
        self.j5B_folder = os.path.join(output_folder, '101')
        if not os.path.exists(self.j5B_folder):
            os.mkdir(self.j5B_folder)

        camera_vs = {
            'CAM_FRONT_120': ['front', 100, 'opencv_pinhole'],
            'CAM_FRONT_LEFT': ['frontleft', 100, 'opencv_pinhole'],
            'CAM_FRONT_RIGHT': ['frontright', 100, 'opencv_pinhole'],
            'CAM_BACK_LEFT': ['rearleft', 100, 'opencv_pinhole'],
            'CAM_BACK_RIGHT': ['rearright', 100, 'opencv_pinhole'],
            'CAM_FRONT_30': ['front_30fov', 101, 'opencv_pinhole'],
            'CAM_BACK': ['rear', 101, 'opencv_pinhole'],
            'CAM_FISHEYE_FRONT': ['fisheye_front', 101, 'opencv_fisheye'],
            'CAM_FISHEYE_BACK': ['fisheye_rear', 101, 'opencv_fisheye'],
            'CAM_FISHEYE_LEFT': ['fisheye_left', 101, 'opencv_fisheye'],
            'CAM_FISHEYE_RIGHT': ['fisheye_right', 101, 'opencv_fisheye'],
        }

        G = np.eye(4)
        G[0:3, 0:3] = Rotation.from_euler('zyx', [0, -np.pi / 2, 0]).as_matrix() @ \
                      Rotation.from_euler('zyx', [0, 0, np.pi / 2]).as_matrix()  # pitch为90度，避免万向节锁，分两次转
        with open(calibration_json, 'r') as f:
            cameras = json.load(f)['camera']
        for origin_camera in cameras:
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

            # 求内参和畸变
            target_model = camera_vs[origin_camera['name']][-1]
            intrinsic_param = origin_camera['intrinsic_param']
            camera_model = intrinsic_param['camera_model']
            size = (intrinsic_param['camera_width'], intrinsic_param['camera_height'])
            extrinsic = np.linalg.inv(vector2matrix(origin_camera['extrinsic_param']['rotation'],
                                                    origin_camera['extrinsic_param']['translation']))
            intrinsic = np.zeros((3, 4))
            intrinsic[:, 0: 3] = np.array(intrinsic_param['camera_matrix'])
            distort = np.array(intrinsic_param['distortion_coeffcients'])
            if camera_model == 'opencv_omni':
                xi = np.array([np.float32(intrinsic_param['xi'])])
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

            if camera_model != target_model:
                distort_camera = DistortCameraObject(camera_par=camera_par, camera_model=camera_model)
                objectPoints, imagePoints = distort_camera.getProjectPoints()

                error, CameraMatrix, distort = distort_camera.calibrateCamera(objectPoints, imagePoints,
                                                                              target_model=target_model)
                distort = list(distort.reshape(1, -1)[0])
            else:
                error = 0
                distort = camera_par['distort'].tolist() + [0] * 10
                CameraMatrix = intrinsic[:, 0: 3]

            if 'eye' not in camera_name.lower():
                distort = distort[:8]
            else:
                distort = distort[:4]

            center_u = CameraMatrix[0][2]
            center_v = CameraMatrix[1][2]
            focal_u = CameraMatrix[0][0]
            focal_v = CameraMatrix[1][1]

            fov = temp_camera['fov']
            image_height = temp_camera['image_height']
            image_width = temp_camera['image_width']
            type_ = temp_camera['type']
            valid_height = temp_camera['valid_height']
            vendor = temp_camera['vendor']
            version = temp_camera['version']
            vcs = temp_camera['vcs']

            new_camera = {
                'camera_x': camera_x,
                'camera_y': camera_y,
                'camera_z': camera_z,
                'center_u': center_u,
                'center_v': center_v,
                'distort': distort,
                'focal_u': focal_u,
                'focal_v': focal_v,
                'fov': fov,
            }

            if __name__ == '__main__':
                print(origin_camera['name'], round(error, 6))

            if 'eye' not in camera_name.lower():
                new_camera['intern'] = temp_camera['intern']
                new_camera['loc'] = temp_camera['loc']
                new_camera['mask'] = temp_camera['mask']

            new_camera['image_height'] = image_height
            new_camera['image_width'] = image_width
            new_camera['pitch'] = pitch
            new_camera['roll'] = roll
            new_camera['type'] = type_
            new_camera['valid_height'] = valid_height
            new_camera['vcs'] = vcs
            new_camera['vendor'] = vendor
            new_camera['version'] = version
            new_camera['yaw'] = yaw

            path = os.path.join(output_folder, str(camera_vs[origin_camera['name']][1]),
                                '{:s}.json'.format(camera_name))
            with open(path, 'w+') as f:
                json.dump(new_camera, f, indent=2)


class ConvertJsonFile2:

    def __init__(self, calibration_json, output_folder):
        self.j5A_folder = os.path.join(output_folder, '100')
        if not os.path.exists(self.j5A_folder):
            os.mkdir(self.j5A_folder)
        self.j5B_folder = os.path.join(output_folder, '101')
        if not os.path.exists(self.j5B_folder):
            os.mkdir(self.j5B_folder)

        camera_vs = {
            'CAM_FRONT_120': ['front','opencv_pinhole'],
            'CAM_BACK': ['rear', 'opencv_pinhole'],
            'CAM_FISHEYE_FRONT': ['fisheye_front', 'opencv_fisheye'],
            'CAM_FISHEYE_BACK': ['fisheye_rear', 'opencv_fisheye'],
            'CAM_FISHEYE_LEFT': ['fisheye_left', 'opencv_fisheye'],
            'CAM_FISHEYE_RIGHT': ['fisheye_right', 'opencv_fisheye'],
        }

        G = np.eye(4)
        G[0:3, 0:3] = Rotation.from_euler('zyx', [0, -np.pi / 2, 0]).as_matrix() @ \
                      Rotation.from_euler('zyx', [0, 0, np.pi / 2]).as_matrix()  # pitch为90度，避免万向节锁，分两次转
        with open(calibration_json, 'r') as f:
            cameras = json.load(f)['camera']
        for origin_camera in cameras:
            if origin_camera['name'] not in camera_vs:
                continue
            camera_name = camera_vs[origin_camera['name']][0]

            # 相机标定坐标系认为是固定参数，从4号车读取
            # 求外参数
            calib2camera = vector2matrix(origin_camera['extrinsic_param']['rotation'],
                                         origin_camera['extrinsic_param']['translation'])

            roll, pitch, yaw, camera_x, camera_y, camera_z = matrix2euler(calib2camera @ G, seq='ZYX')
            print(camera_name, roll, pitch, yaw, camera_x, camera_y, camera_z)

            # # 求内参和畸变
            # target_model = camera_vs[origin_camera['name']][-1]
            # intrinsic_param = origin_camera['intrinsic_param']
            # camera_model = intrinsic_param['camera_model']
            # size = (intrinsic_param['camera_width'], intrinsic_param['camera_height'])
            # extrinsic = np.linalg.inv(vector2matrix(origin_camera['extrinsic_param']['rotation'],
            #                                         origin_camera['extrinsic_param']['translation']))
            # intrinsic = np.zeros((3, 4))
            # intrinsic[:, 0: 3] = np.array(intrinsic_param['camera_matrix'])
            # distort = np.array(intrinsic_param['distortion_coeffcients'])
            # if camera_model == 'opencv_omni':
            #     xi = np.array([np.float32(intrinsic_param['xi'])])
            # else:
            #     xi = 0
            # fov_range = [temp_camera['vcs']['rotation'][2] + yaw - np.deg2rad(temp_camera['fov']) / 2,
            #              temp_camera['vcs']['rotation'][2] + yaw + np.deg2rad(temp_camera['fov']) / 2]
            # camera_par = {
            #     'size': size,
            #     'extrinsic': extrinsic,
            #     'intrinsic': intrinsic,
            #     'distort': distort,
            #     'xi': xi,
            #     'fov_range': fov_range,
            # }
            #
            # if camera_model != target_model:
            #     distort_camera = DistortCameraObject(camera_par=camera_par, camera_model=camera_model)
            #     objectPoints, imagePoints = distort_camera.getProjectPoints()
            #
            #     error, CameraMatrix, distort = distort_camera.calibrateCamera(objectPoints, imagePoints,
            #                                                                   target_model=target_model)
            #     distort = list(distort.reshape(1, -1)[0])
            # else:
            #     error = 0
            #     distort = camera_par['distort'].tolist() + [0] * 10
            #     CameraMatrix = intrinsic[:, 0: 3]
            #
            # if 'eye' not in camera_name.lower():
            #     distort = distort[:8]
            # else:
            #     distort = distort[:4]
            #
            # center_u = CameraMatrix[0][2]
            # center_v = CameraMatrix[1][2]
            # focal_u = CameraMatrix[0][0]
            # focal_v = CameraMatrix[1][1]
            #
            # fov = temp_camera['fov']
            # image_height = temp_camera['image_height']
            # image_width = temp_camera['image_width']
            # type_ = temp_camera['type']
            # valid_height = temp_camera['valid_height']
            # vendor = temp_camera['vendor']
            # version = temp_camera['version']
            # vcs = temp_camera['vcs']
            #
            # new_camera = {
            #     'camera_x': camera_x,
            #     'camera_y': camera_y,
            #     'camera_z': camera_z,
            #     'center_u': center_u,
            #     'center_v': center_v,
            #     'distort': distort,
            #     'focal_u': focal_u,
            #     'focal_v': focal_v,
            #     'fov': fov,
            # }
            #
            # if __name__ == '__main__':
            #     print(origin_camera['name'], round(error, 6))
            #
            # if 'eye' not in camera_name.lower():
            #     new_camera['intern'] = temp_camera['intern']
            #     new_camera['loc'] = temp_camera['loc']
            #     new_camera['mask'] = temp_camera['mask']
            #
            # new_camera['image_height'] = image_height
            # new_camera['image_width'] = image_width
            # new_camera['pitch'] = pitch
            # new_camera['roll'] = roll
            # new_camera['type'] = type_
            # new_camera['valid_height'] = valid_height
            # new_camera['vcs'] = vcs
            # new_camera['vendor'] = vendor
            # new_camera['version'] = version
            # new_camera['yaw'] = yaw
            #
            # path = os.path.join(output_folder, str(camera_vs[origin_camera['name']][1]),
            #                     '{:s}.json'.format(camera_name))
            # with open(path, 'w+') as f:
            #     json.dump(new_camera, f, indent=2)


class CameraObject:

    # 从json_file中读取，是指从地平线标准格式的相机json构造相机
    # 也可以从参数传入构造一个新相机
    # 这里的外参矩阵实际上为【逆外参】
    def __init__(self, json_file=None, yaml_file=None, size=None, extrinsic=None, intrinsic=None, distort=None, xi=None,
                 fov_range=None):
        camera_rotation = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
        if json_file:
            with open(json_file, 'r', encoding='utf8') as fp:
                self.json_data = json.load(fp)
            self.size = (self.json_data['image_width'], self.json_data['image_height'])
            if 'vcs' in self.json_data:
                self.extrinsic = camera_rotation @ self.cal_RTMatrix_horizon()
                camera_orientation = self.json_data['vcs']['rotation'][2] + self.json_data['yaw']
            else:
                self.extrinsic = camera_rotation @ self.cal_RTMatrix_json()
                camera_orientation = self.json_data['yaw']
            self.intrinsic = np.array([
                [self.json_data['focal_u'], 0, self.json_data['center_u'], 0],
                [0, self.json_data['focal_v'], self.json_data['center_v'], 0],
                [0, 0, 1, 0]])
            self.distort = np.array(self.json_data['distort'])
            self.fov_range = [
                camera_orientation - np.deg2rad(self.json_data['fov'] / 2),
                camera_orientation + np.deg2rad(self.json_data['fov'] / 2),
            ]
            print(f"+++++++++{json_file}: {self.fov_range}+++++++++")
            self.xi = 0
        elif yaml_file:
            with open(yaml_file, 'r', encoding='utf8') as fp:
                self.yaml_data = yaml.safe_load(fp)
            self.size = (self.yaml_data['image_width'], self.yaml_data['image_height'])
            self.extrinsic = camera_rotation @ self.cal_RTMatrix()
            self.intrinsic = np.array([
                [self.yaml_data['focal_x'], 0, self.yaml_data['center_u'], 0],
                [0, self.yaml_data['focal_y'], self.yaml_data['center_v'], 0],
                [0, 0, 1, 0]])
            self.distort = np.array(self.yaml_data['distort'])
            camera_orientation = self.yaml_data['yaw']
            self.fov_range = [
                camera_orientation - np.deg2rad(self.yaml_data['fov'] / 2),
                camera_orientation + np.deg2rad(self.yaml_data['fov'] / 2),
            ]
            self.xi = 0
        else:
            self.size = size
            self.extrinsic = extrinsic  # 4 X 4
            self.intrinsic = intrinsic  # 3 x 4
            self.distort = distort  # 1 X n
            self.xi = xi
            self.fov_range = fov_range
            print(f"+++++++++{self.fov_range}+++++++++")
        self.fov = self.fov_range[-1] - self.fov_range[0]
        self.camera_location = np.linalg.inv(self.extrinsic)[0:3, 3]

    def cal_RTMatrix_horizon(self):
        # 世界坐标系到标定坐标系
        world2calib = euler2matrix(
            *self.json_data['vcs']['rotation'],
            *self.json_data['vcs']['translation']
        )

        # 标定坐标系到相机坐标系
        calib2camera = euler2matrix(
            self.json_data['roll'],
            self.json_data['pitch'],
            self.json_data['yaw'],
            self.json_data['camera_x'],
            self.json_data['camera_y'],
            self.json_data['camera_z'],
        )

        return np.linalg.inv(world2calib @ calib2camera)

    def cal_RTMatrix_json(self):
        world2camera = euler2matrix(
            self.json_data['roll'],
            self.json_data['pitch'],
            self.json_data['yaw'],
            self.json_data['camera_x'],
            self.json_data['camera_y'],
            self.json_data['camera_z'],
            seq='ZYX',
        )
        return np.linalg.inv(world2camera)

    def cal_RTMatrix(self):
        world2camera = euler2matrix(
            self.yaml_data['roll'],
            self.yaml_data['pitch'],
            self.yaml_data['yaw'],
            self.yaml_data['pos_x'],
            self.yaml_data['pos_y'],
            self.yaml_data['pos_z'],
            seq='ZYX'
        )

        return np.linalg.inv(world2camera)

    def world2camera(self, x, y, z):
        return world2camera(self.intrinsic, self.extrinsic, x, y, z)


class DistortCameraObject:

    # 从json_file中读取，是指从地平线标准格式的相机json构造相机
    # 也可以从参数传入构造一个新相机
    # 从json进入的方式构造相机可能有问题，暂时停止使用
    def __init__(self, json_file=None, yaml_file=None, camera_par=None, camera_model='opencv_pinhole'):
        if json_file:
            self.camera_object = CameraObject(json_file=json_file)
        elif yaml_file:
            self.camera_object = CameraObject(yaml_file=yaml_file)
        else:
            self.camera_object = CameraObject(**camera_par)

        self.camera_model = camera_model
        size = self.camera_object.size
        if camera_model == 'opencv_fisheye':
            newCameraMatrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K=self.camera_object.intrinsic[:, :3],
                D=self.camera_object.distort,
                image_size=size,
                R=np.eye(3),
                balance=0,
                new_size=size
            )
            self.mapX, self.mapY = cv2.fisheye.initUndistortRectifyMap(
                K=self.camera_object.intrinsic[:, :3],
                D=self.camera_object.distort,
                R=np.eye(3),
                P=newCameraMatrix,
                size=size,
                m1type=cv2.CV_16SC2
            )
        elif camera_model == 'opencv_pinhole':
            newCameraMatrix, self.roi = cv2.getOptimalNewCameraMatrix(
                cameraMatrix=self.camera_object.intrinsic[:, :3],
                distCoeffs=self.camera_object.distort,
                imageSize=size,
                alpha=1,
                newImgSize=size,
                centerPrincipalPoint=False
            )
            self.mapX, self.mapY = cv2.initUndistortRectifyMap(
                cameraMatrix=self.camera_object.intrinsic[:, :3],
                distCoeffs=self.camera_object.distort,
                R=np.eye(3),
                newCameraMatrix=newCameraMatrix,
                size=size,
                m1type=cv2.CV_16SC2
            )
            newCameraMatrix[0][-1] = newCameraMatrix[0][-1] - self.roi[0]
            newCameraMatrix[1][-1] = newCameraMatrix[1][-1] - self.roi[1]
        else:
            # opencv_omni
            newCameraMatrix = np.array([[size[0] / 4, 0, size[0] / 2],
                                        [0, size[0] / 4, size[0] / 2],
                                        [0, 0, 1]])
            try:
                self.mapX, self.mapY = cv2.omnidir.initUndistortRectifyMap(
                    K=self.camera_object.intrinsic[:, :3],
                    D=self.camera_object.distort,
                    xi=self.camera_object.xi,
                    R=np.eye(3),
                    P=newCameraMatrix,
                    size=size,
                    mltype=cv2.CV_16SC2,
                    flags=cv2.omnidir.RECTIFY_PERSPECTIVE
                )
            except:
                self.mapX, self.mapY = cv2.omnidir.initUndistortRectifyMap(
                    K=self.camera_object.intrinsic[:, :3],
                    D=self.camera_object.distort,
                    xi=self.camera_object.xi,
                    R=np.eye(3),
                    P=newCameraMatrix,
                    size=size,
                    m1type=cv2.CV_16SC2,
                    flags=cv2.omnidir.RECTIFY_PERSPECTIVE
                )

        self.new_intrinsic = np.zeros((3, 4))
        self.new_intrinsic[:, 0: 3] = newCameraMatrix
        # print('roi: ', self.roi)
        # print('old intrinsic:')
        # print(self.camera_object.intrinsic)
        # print('intrinsic after de_distorted:')
        # print(newCameraMatrix)
        # print('intrinsic after cropped:')
        # print(self.new_intrinsic)
        # print('===================================')

    def getProjectPoints(self):
        # 创造不同的相机姿态
        camera_rotation = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
        extrinsics = []
        calibrate_point_groups = []
        for camera_z in [-2, 0, 2]:
            for camera_y in [-2, 0, 2]:
                for camera_x in [0, -2]:
                    roll = 0
                    pitch = np.arctan(camera_z / (camera_x - 6))
                    yaw = np.arctan(camera_y / (camera_x - 6))
                    # print(yaw)
                    RPY = get_RPY(roll, pitch, yaw, camera_x, camera_y, camera_z)
                    extrinsics.append(camera_rotation @ np.linalg.inv(RPY))
                    # 构造标定点，尽量在fov范围内
                    calibrate_points = []
                    for c_z in np.arange(-6, 6.1, 0.75):
                        for c_y in np.arange(-6, 6.1, 0.75):
                            c_x = 6
                            calibrate_points.append([c_x, c_y, c_z])
                    calibrate_point_groups.append(calibrate_points)

        objectPointsForExtrinsics = []
        imagePointsForExtrinsics = []
        for extrinsic, calibrate_points in zip(extrinsics, calibrate_point_groups):
            tvec = extrinsic[0:3, 3]
            rvec = cv2.Rodrigues(extrinsic[0:3, 0:3])[0]
            objectPoints = np.array([calibrate_points], dtype=np.float32)
            if self.camera_model == 'opencv_fisheye':
                imagePoints, _ = cv2.fisheye.projectPoints(
                    objectPoints=objectPoints,
                    rvec=rvec,
                    tvec=tvec,
                    K=self.camera_object.intrinsic[:, :3],
                    D=self.camera_object.distort,
                    alpha=0,
                )
            elif self.camera_model == 'opencv_pinhole':
                imagePoints, _ = cv2.projectPoints(
                    objectPoints=objectPoints,
                    rvec=rvec,
                    tvec=tvec,
                    cameraMatrix=self.camera_object.intrinsic[:, :3],
                    distCoeffs=self.camera_object.distort,
                )
                # opencv_pinhole.projectPoints输出的imagePoints为 (N * 1 * 2），需要转化为统一的 (1 * N * 2）
                imagePoints = changeCvDataShape(imagePoints, direction='N*1*mto1*N*m')
            else:
                # opencv_omni
                imagePoints, _ = cv2.omnidir.projectPoints(
                    objectPoints=objectPoints,
                    rvec=rvec,
                    tvec=tvec,
                    K=self.camera_object.intrinsic[:, :3],
                    xi=self.camera_object.xi[0],
                    D=self.camera_object.distort,
                )

            # 只保存投射完在视野内的点
            objpts, imgpts = [], []
            for i, objectPoint in enumerate(objectPoints[0]):
                if 50 < imagePoints[0][i][0] < self.camera_object.size[0] - 50 and 50 < imagePoints[0][i][1] < \
                        self.camera_object.size[1] - 50:
                    objpts.append(objectPoints[0][i, :].tolist())
                    imgpts.append(imagePoints[0][i, :].tolist())

            # print('视野中存在{:d}个标定点'.format(len(objpts)))
            objectPointsForExtrinsics.append(np.float32(objpts))
            imagePointsForExtrinsics.append(np.float32(imgpts))

        return objectPointsForExtrinsics, imagePointsForExtrinsics

    def calibrateCamera(self, objectPointsForExtrinsics, imagePointsForExtrinsics, target_model):
        calibration_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100000, 1e-10)
        if target_model == 'opencv_fisheye':
            # opencv_fisheye.calibrate需要的数据为(N * 2），需要转化为统一的 (N * 1 * 2）
            new_objectPointsForExtrinsics = []
            new_imagePointsForExtrinsics = []
            for objectPoint in objectPointsForExtrinsics:
                temp = np.zeros((objectPoint.shape[0], 1, 3), dtype=np.float32)
                temp[:, 0, :] = objectPoint
                new_objectPointsForExtrinsics.append(temp)
            for imagePoint in imagePointsForExtrinsics:
                temp = np.zeros((imagePoint.shape[0], 1, 2), dtype=np.float32)
                temp[:, 0, :] = imagePoint
                new_imagePointsForExtrinsics.append(temp)

            ret, CameraMatrix, distort, rvecs, tvecs = cv2.fisheye.calibrate(
                objectPoints=new_objectPointsForExtrinsics,
                imagePoints=new_imagePointsForExtrinsics,
                image_size=self.camera_object.size,
                K=None,
                D=None,
                rvecs=None,
                tvecs=None,
                flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW,
                criteria=calibration_criteria
            )
        else:
            # opencv_pinhole
            ret, CameraMatrix, distort, rvecs, tvecs = cv2.calibrateCamera(
                objectPoints=objectPointsForExtrinsics,
                imagePoints=imagePointsForExtrinsics,
                imageSize=self.camera_object.size,
                cameraMatrix=self.camera_object.intrinsic[:, :3],
                distCoeffs=self.camera_object.distort,
                rvecs=None,
                tvecs=None,
                flags=cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_USE_INTRINSIC_GUESS,
                criteria=calibration_criteria
            )

        return ret, CameraMatrix, distort

    def de_distort(self, image_path):
        if isinstance(image_path, str):
            image = cv2.imread(image_path)
        else:
            image = image_path
        image = cv2.remap(image, self.mapX, self.mapY, cv2.INTER_LINEAR)
        w, h = self.camera_object.size
        if self.camera_model == 'opencv_pinhole':
            x, y, w, h = self.roi
            image = image[y: y + h, x: x + w]
        return image, w, h

    def world2camera(self, x, y, z):
        return world2camera(self.new_intrinsic, self.camera_object.extrinsic, x, y, z)

    def world2camera_with_distort(self, x, y, z, cut_flag=True):
        extrinsic = self.camera_object.extrinsic
        tvec = extrinsic[0:3, 3]
        rvec = cv2.Rodrigues(extrinsic[0:3, 0:3])[0]
        pts = [x, y, z]
        objectPoints = np.array([[pts]], dtype=np.float32)

        if self.camera_model == 'opencv_fisheye':
            imagePoints, _ = cv2.fisheye.projectPoints(
                objectPoints=objectPoints,
                rvec=rvec,
                tvec=tvec,
                K=self.camera_object.intrinsic[:, :3],
                D=self.camera_object.distort,
                alpha=0,
            )

        elif self.camera_model == 'opencv_pinhole':
            imagePoints, _ = cv2.projectPoints(
                objectPoints=objectPoints,
                rvec=rvec,
                tvec=tvec,
                cameraMatrix=self.camera_object.intrinsic[:, :3],
                distCoeffs=self.camera_object.distort,
            )
            # opencv_pinhole.projectPoints输出的imagePoints为 (N * 1 * 2），需要转化为统一的 (1 * N * 2）
            imagePoints = changeCvDataShape(imagePoints, direction='N*1*mto1*N*m')

        else:
            # opencv_omni
            imagePoints, _ = cv2.omnidir.projectPoints(
                objectPoints=objectPoints,
                rvec=rvec,
                tvec=tvec,
                K=self.camera_object.intrinsic[:, :3],
                xi=self.camera_object.xi[0],
                D=self.camera_object.distort,
            )

        u = round(imagePoints[0][0][0])
        v = round(imagePoints[0][0][1])

        if cut_flag:
            if 20 < u < self.camera_object.size[0] - 20 and 20 < v < self.camera_object.size[1] - 20:
                return u, v, self.camera_object.size[0], self.camera_object.size[1]
            else:
                return None

        else:
            return u, v


class BirdEyeView:

    # 从json_file中读取，是指从地平线标准格式的相机json构造相机
    # 也可以直接将DistortCameraObject实例字典传入
    def __init__(self, bird_json_file, camera_json_files=None, distort_camera_dict=None, bev_type='fisheye'):
        self.bev_camera = CameraObject(bird_json_file)
        if camera_json_files:
            self.origin_camera = {camera: DistortCameraObject(json_file=json_file[0], camera_model=json_file[1])
                                  for camera, json_file in camera_json_files.items()}
            cameras = camera_json_files.keys()
        else:
            self.origin_camera = {camera: distort_camera
                                  for camera, distort_camera in distort_camera_dict.items()}
            cameras = distort_camera_dict.keys()
        self.bev_type = bev_type

        self.calibration_pts = [[10, 10], [10, -10], [-10, -10], [-10, 10]]
        self.weight_flag = False
        self.del_matrix_dic = {}
        bev_height = self.bev_camera.size[1]
        bev_width = self.bev_camera.size[0]
        self.weight_matrix = np.zeros((bev_height, bev_width))

        for camera in cameras:
            fov_1, fov_2 = self.origin_camera[camera].camera_object.fov_range
            del_matrix = np.zeros((bev_height, bev_width))
            print("正在计算{:s}的删除矩阵".format(camera))
            # camera in bev
            camera_in_bev = self.bev_camera.world2camera(*self.origin_camera[camera].camera_object.camera_location)
            print('bev中，相机位置在', camera, camera_in_bev)

            for row_index, row in enumerate(del_matrix):
                for col_index, col in enumerate(row):
                    pos = self.uv2pos(bev_height, bev_width, row_index, col_index, *camera_in_bev)
                    if fov_1 < pos < fov_2:
                        del_matrix[row_index][col_index] = 1
                    elif fov_1 - 2 * np.pi < pos < fov_2 - 2 * np.pi:
                        del_matrix[row_index][col_index] = 1
                    elif fov_1 + 2 * np.pi < pos < fov_2 + 2 * np.pi:
                        del_matrix[row_index][col_index] = 1
                    else:
                        del_matrix[row_index][col_index] = 0
            self.del_matrix_dic[camera] = del_matrix

    # 像素坐标转矩阵内角度坐标
    def uv2pos(self, bh, bw, v, u, camera_u, camera_v):
        point_x = u - bw / 2
        point_y = bh / 2 - v
        camera_x = camera_u - bw / 2
        camera_y = bh / 2 - camera_v
        point_x = point_x - camera_x
        point_y = point_y - camera_y
        pos = 100
        # 第一象限
        if point_x >= 0 and point_y > 0:
            pos = -np.arctan(point_x / point_y)
        # 第二象限
        if point_x < 0 < point_y:
            pos = -np.arctan(point_x / point_y)
        # 第三象限
        if point_x < 0 and point_y < 0:
            pos = np.pi - np.arctan(point_x / point_y)
        # 第四象限
        if point_x >= 0 > point_y:
            pos = np.pi - np.arctan(point_x / point_y)
        if point_x > 0 and point_y == 0:
            pos = - np.pi / 2
        if point_x < 0 and point_y == 0:
            pos = np.pi / 2
        return pos

    # 图片融合矩阵获取（二图重叠则 0.5 三图则0.33）
    def get_weight_matrix(self):
        for camera in self.origin_camera.keys():
            self.weight_matrix += self.del_matrix_dic[camera]
        for row_index, row in enumerate(self.weight_matrix):
            for col_index, col in enumerate(row):
                if col != 0:
                    self.weight_matrix[row_index][col_index] = 1 / col

    # 获取修正后的删除矩阵（原删除矩阵为1 而此像素无图 则置0）
    def correct_matrix(self, camera, del_matrix, bird_image):
        print("正在修正{:s}的删除矩阵".format(camera))
        for row_index, row in enumerate(del_matrix):
            for col_index, col in enumerate(row):
                if col == 1:
                    if bird_image[row_index][col_index][0] == 0:
                        self.del_matrix_dic[camera][row_index][col_index] = 0

    def getBev(self, images, folder, rect_pts=None):
        if rect_pts is None:
            rect_pts = []
        bev_height = self.bev_camera.size[1]
        bev_width = self.bev_camera.size[0]
        bird_image_merge = np.zeros((bev_height, bev_width, 3))
        bird_image_merge_addweight = np.zeros((bev_height, bev_width, 3))
        bird_image_dic = {}

        for camera, image in images.items():
            de_image, w, h = self.origin_camera[camera].de_distort(image)
            # 去畸变结果
            cv2.imwrite(os.path.join(folder, f'De-distort_{camera}_{w}x{h}.jpg'), de_image)
            ori_pts, bev_pts = [], []
            for pt in self.calibration_pts:
                ori_pts.append(self.origin_camera[camera].world2camera(*pt, 0))
                bev_pts.append(self.bev_camera.world2camera(*pt, 0))

            ori_pts = np.array(ori_pts, dtype=np.float32)
            bev_pts = np.array(bev_pts, dtype=np.float32)
            M = cv2.getPerspectiveTransform(ori_pts, bev_pts)
            bird_image = cv2.warpPerspective(de_image, M, (self.bev_camera.size[0], self.bev_camera.size[1]),
                                             cv2.INTER_LINEAR)
            # BEV结果（双重)
            # cv2.imwrite(os.path.join(folder, f'BEV_{camera}_raw.jpg'), bird_image)

            # 第一次运行时先求得各个相机修正后的删除矩阵和图像融合矩阵
            if not self.weight_flag:
                self.correct_matrix(camera, self.del_matrix_dic[camera], bird_image)
            bird_image_dic[camera] = bird_image

        if not self.weight_flag:
            self.weight_flag = True
            self.get_weight_matrix()

        for camera, bird_image in bird_image_dic.items():
            b = bird_image[:, :, 0] * self.del_matrix_dic[camera]
            g = bird_image[:, :, 1] * self.del_matrix_dic[camera]
            r = bird_image[:, :, 2] * self.del_matrix_dic[camera]
            bird_image = cv2.merge((b, g, r))
            # BEV结果（单重)
            cv2.imwrite(os.path.join(folder, f'BEV_{camera}_clear.jpg'), bird_image)

            bird_image_merge += bird_image
            # add weight
            b_merge = b * self.weight_matrix
            g_merge = g * self.weight_matrix
            r_merge = r * self.weight_matrix
            bird_image_addweight = cv2.merge((b_merge, g_merge, r_merge))
            bird_image_merge_addweight += bird_image_addweight

        bev_rect_pts = []
        for pt in rect_pts:
            bev_rect_pts.append(self.bev_camera.world2camera(*pt, 0))
        if bev_rect_pts:
            uvps = np.array(bev_rect_pts, np.int32).reshape((-1, 1, 2))
            cv2.polylines(bird_image_merge_addweight, [uvps], True, (48, 48, 255), 3)

        # 最终的融合图片
        cv2.imwrite(os.path.join(folder, f'BEV_{self.bev_type}.jpg'), bird_image_merge_addweight)
        # cv2.imwrite(os.path.join(folder, f'BEV_ALL.jpg'), bird_image_merge)
        return os.path.join(folder, f'BEV_{self.bev_type}.jpg')


def kunyi_bev(folder, calibration_json, bev_type='fisheye', rect_pts=None):
    bird_json_file = os.path.join(camera_parameter_folder, 'bird_virtual', 'bird.json')
    distort_camera_dict = {}

    camera_vs = {
        'CAM_FRONT_120': ['front', 100],
        'CAM_FRONT_LEFT': ['frontleft', 100],
        'CAM_FRONT_RIGHT': ['frontright', 100],
        'CAM_BACK_LEFT': ['rearleft', 100],
        'CAM_BACK_RIGHT': ['rearright', 100],
        'CAM_FRONT_30': ['front_30fov', 101],
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
        if bev_type == 'fisheye':
            if 'eye' not in origin_camera['name'].lower() or '30' in origin_camera['name'].lower():
                continue
        else:
            if 'eye' in origin_camera['name'].lower() or '30' in origin_camera['name'].lower():
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
                      bev_type=bev_type)
    images = {}
    for camera in distort_camera_dict.keys():
        if bev_type == 'fisheye':
            if 'eye' in camera.lower() and '30' not in camera.lower():
                images[camera] = os.path.join(folder, f'{camera}.jpg')
        else:
            if 'eye' not in camera.lower() and '30' not in camera.lower():
                images[camera] = os.path.join(folder, f'{camera}.jpg')

    bev_folder = os.path.join(folder, 'Kunyi', bev_type)
    if os.path.exists(bev_folder):
        shutil.rmtree(bev_folder)
    os.makedirs(bev_folder)

    BEV.getBev(images, bev_folder, rect_pts)


def horizon_bev(folder, bev_type='fisheye'):
    bird_json_file = os.path.join(camera_parameter_folder, 'bird_virtual', 'bird.json')

    camera_json_files = {
        'CAM_FRONT_120': ['../Config/Camera/replay/2J5/100/front.json', 'opencv_pinhole'],
        'CAM_FRONT_LEFT': ['../Config/Camera/replay/2J5/100/frontleft.json', 'opencv_pinhole'],
        'CAM_FRONT_RIGHT': ['../Config/Camera/replay/2J5/100/frontright.json', 'opencv_pinhole'],
        'CAM_BACK_LEFT': ['../Config/Camera/replay/2J5/100/rearleft.json', 'opencv_pinhole'],
        'CAM_BACK_RIGHT': ['../Config/Camera/replay/2J5/100/rearright.json', 'opencv_pinhole'],
        'CAM_BACK': ['../Config/Camera/replay/2J5/101/rear.json', 'opencv_pinhole'],
        'CAM_FISHEYE_FRONT': ['../Config/Camera/replay/2J5/101/fisheye_front.json', 'opencv_fisheye'],
        'CAM_FISHEYE_BACK': ['../Config/Camera/replay/2J5/101/fisheye_rear.json', 'opencv_fisheye'],
        'CAM_FISHEYE_LEFT': ['../Config/Camera/replay/2J5/101/fisheye_left.json', 'opencv_fisheye'],
        'CAM_FISHEYE_RIGHT': ['../Config/Camera/replay/2J5/101/fisheye_right.json', 'opencv_fisheye'],
    }

    images = {
        'CAM_FRONT_120': '/home/buyujun/ZONE/Data/BEV/CAM_FRONT_120.jpg',
        'CAM_FRONT_LEFT': '/home/buyujun/ZONE/Data/BEV/CAM_FRONT_LEFT.jpg',
        'CAM_FRONT_RIGHT': '/home/buyujun/ZONE/Data/BEV/CAM_FRONT_RIGHT.jpg',
        'CAM_BACK_LEFT': '/home/buyujun/ZONE/Data/BEV/CAM_BACK_LEFT.jpg',
        'CAM_BACK_RIGHT': '/home/buyujun/ZONE/Data/BEV/CAM_BACK_RIGHT.jpg',
        'CAM_BACK': '/home/buyujun/ZONE/Data/BEV/CAM_BACK.jpg',
        'CAM_FISHEYE_FRONT': '/home/buyujun/ZONE/Data/BEV/CAM_FISHEYE_FRONT.jpg',
        'CAM_FISHEYE_BACK': '/home/buyujun/ZONE/Data/BEV/CAM_FISHEYE_BACK.jpg',
        'CAM_FISHEYE_LEFT': '/home/buyujun/ZONE/Data/BEV/CAM_FISHEYE_LEFT.jpg',
        'CAM_FISHEYE_RIGHT': '/home/buyujun/ZONE/Data/BEV/CAM_FISHEYE_RIGHT.jpg',
    }

    if bev_type == 'fisheye':
        new_camera_json_files = {camera: value for camera, value in camera_json_files.items() if
                                 'eye' in camera.lower()}
        new_images = {camera: value for camera, value in images.items() if 'eye' in camera.lower()}
    else:
        new_camera_json_files = {camera: value for camera, value in camera_json_files.items() if
                                 'eye' not in camera.lower()}
        new_images = {camera: value for camera, value in images.items() if 'eye' not in camera.lower()}

    BEV = BirdEyeView(bird_json_file=bird_json_file,
                      camera_json_files=new_camera_json_files,
                      bev_type=bev_type)

    bev_folder = os.path.join(folder, 'Horizon', bev_type)
    if os.path.exists(bev_folder):
        shutil.rmtree(bev_folder)
    os.makedirs(bev_folder)

    BEV.getBev(new_images, bev_folder)


def transfer_2j5_2_1j5(json_folder, yaml_folder):
    def euler2matrix2(roll, pitch, yaw, x, y, z):
        R = np.eye(4)
        R[0:3, 3] = [x, y, z]
        R[0:3, 0:3] = Rotation.from_euler('XYZ', [roll, pitch, yaw]).as_matrix()
        # R[0:3, 0:3] = Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
        return R

    def matrix2euler2(R):
        x = R[0, 3]
        y = R[1, 3]
        z = R[2, 3]
        yaw, pitch, roll = Rotation.from_matrix(R[0:3, 0:3]).as_euler('ZYX')
        # roll, pitch, yaw = Rotation.from_matrix(R[0:3, 0:3]).as_euler('XYZ')
        return roll, pitch, yaw, x, y, z

    mapping = {
        0: '100/frontright',
        1: '100/frontleft',
        2: '101/rear',
        3: '100/rearleft',
        4: '100/rearright',
        5: '100/front',
        6: '101/front_30fov',
        7: '101/fisheye_front',
        8: '101/fisheye_right',
        9: '101/fisheye_rear',
        10: '101/fisheye_left',
    }

    for num in mapping.keys():

        json_2j5 = os.path.join(json_folder, f'{mapping[num]}.json')
        with open(json_2j5, 'r') as f:
            camera_config = json.load(f)

        world2calib = euler2matrix2(*camera_config['vcs']['rotation'],
                                    *camera_config['vcs']['translation'])
        calib2camera = euler2matrix2(camera_config['roll'], camera_config['pitch'], camera_config['yaw'],
                                     camera_config['camera_x'], camera_config['camera_y'], camera_config['camera_z'])

        roll, pitch, yaw, x, y, z = matrix2euler2(world2calib @ calib2camera)

        if num <= 6:
            yaml_lines = [
                'cam_{:d}_image_width: {:d}\n'.format(num, camera_config['image_width']),
                'cam_{:d}_image_height: {:d}\n'.format(num, camera_config['image_height']),
                'cam_{:d}_focal_x: {:.16e}\n'.format(num, camera_config['focal_u']),
                'cam_{:d}_focal_y: {:.16e}\n'.format(num, camera_config['focal_v']),
                'cam_{:d}_center_u: {:.16e}\n'.format(num, camera_config['center_u']),
                'cam_{:d}_center_v: {:.16e}\n'.format(num, camera_config['center_v']),
                'cam_{:d}_pos_x: {:.16e}\n'.format(num, float(x)),
                'cam_{:d}_pos_y: {:.16e}\n'.format(num, float(y)),
                'cam_{:d}_pos_z: {:.16e}\n'.format(num, float(z)),
                'cam_{:d}_pitch: {:.16e}\n'.format(num, float(pitch)),
                'cam_{:d}_roll: {:.16e}\n'.format(num, float(roll)),
                'cam_{:d}_yaw: {:.16e}\n'.format(num, float(yaw)),
                'cam_{:d}_vehicleWheelBase: {:.16e}\n'.format(num, 2.95),
                'cam_{:d}_version: "{:s}"\n'.format(num, "230527 1414"),
                'vehicle: 2\n',
                'cam_{:d}_distort_k1: {:.16e}\n'.format(num, camera_config['distort'][0]),
                'cam_{:d}_distort_k2: {:.16e}\n'.format(num, camera_config['distort'][1]),
                'cam_{:d}_distort_p1: {:.16e}\n'.format(num, camera_config['distort'][2]),
                'cam_{:d}_distort_p2: {:.16e}\n'.format(num, camera_config['distort'][3]),
                'cam_{:d}_distort_k3: {:.16e}\n'.format(num, camera_config['distort'][4]),
                'cam_{:d}_distort_k4: {:.16e}\n'.format(num, camera_config['distort'][5]),
                'cam_{:d}_distort_k5: {:.16e}\n'.format(num, camera_config['distort'][6]),
                'cam_{:d}_distort_k6: {:.16e}\n'.format(num, camera_config['distort'][7]),
                'intri_flag: 0\n',
            ]
        else:
            yaml_lines = [
                'cam_{:d}_image_width: {:d}\n'.format(num, camera_config['image_width']),
                'cam_{:d}_image_height: {:d}\n'.format(num, camera_config['image_height']),
                'cam_{:d}_focal_x: {:.16e}\n'.format(num, camera_config['focal_u']),
                'cam_{:d}_focal_y: {:.16e}\n'.format(num, camera_config['focal_v']),
                'cam_{:d}_center_u: {:.16e}\n'.format(num, camera_config['center_u']),
                'cam_{:d}_center_v: {:.16e}\n'.format(num, camera_config['center_v']),
                'cam_{:d}_pos_x: {:.16e}\n'.format(num, float(x)),
                'cam_{:d}_pos_y: {:.16e}\n'.format(num, float(y)),
                'cam_{:d}_pos_z: {:.16e}\n'.format(num, float(z)),
                'cam_{:d}_pitch: {:.16e}\n'.format(num, float(pitch)),
                'cam_{:d}_roll: {:.16e}\n'.format(num, float(roll)),
                'cam_{:d}_yaw: {:.16e}\n'.format(num, float(yaw)),
                'cam_{:d}_vehicleWheelBase: {:.16e}\n'.format(num, 2.95),
                'cam_{:d}_version: "{:s}"\n'.format(num, "230527 1414"),
                'vehicle: 2\n',
                'cam_{:d}_distort_k1: {:.16e}\n'.format(num, camera_config['distort'][0]),
                'cam_{:d}_distort_k2: {:.16e}\n'.format(num, camera_config['distort'][1]),
                'cam_{:d}_distort_k3: {:.16e}\n'.format(num, camera_config['distort'][2]),
                'cam_{:d}_distort_k4: {:.16e}\n'.format(num, camera_config['distort'][3]),
                'intri_flag: 0\n',
            ]
        with open(os.path.join(yaml_folder, f'camera_{num}.yaml'), 'w', encoding='utf-8') as f:
            f.writelines(yaml_lines)

    with open(os.path.join(yaml_folder, f'cam_description.yaml'), 'w', encoding='utf-8') as f:
        yaml.dump({
            0: 'CAM_FRONT_RIGHT',
            1: 'CAM_FRONT_LEFT',
            2: 'CAM_BACK',
            3: 'CAM_BACK_LEFT',
            4: 'CAM_BACK_RIGHT',
            5: 'CAM_FRONT_120',
            6: 'CAM_FRONT_30',
            7: 'CAM_FISHEYE_FRONT',
            8: 'CAM_FISHEYE_RIGHT',
            9: 'CAM_FISHEYE_BACK',
            10: 'CAM_FISHEYE_LEFT',
        }, f, encoding='utf-8', allow_unicode=True, sort_keys=False)


def transfer_es39_2_1j5(json_folder, yaml_folder):
    mapping = {
        'CAM_FRONT_RIGHT': 'frontright',
        'CAM_FRONT_LEFT': 'frontleft',
        'CAM_BACK': 'rear',
        'CAM_BACK_LEFT': 'rearleft',
        'CAM_BACK_RIGHT': 'rearright',
        'CAM_FRONT_120': 'front',
        'CAM_FRONT_30': 'front_30fov',
        'CAM_FISHEYE_FRONT': 'fisheye_front',
        'CAM_FISHEYE_RIGHT': 'fisheye_right',
        'CAM_FISHEYE_BACK': 'fisheye_rear',
        'CAM_FISHEYE_LEFT': 'fisheye_left',
    }

    for camera_name in mapping.keys():

        json_es39 = os.path.join(json_folder, f'{mapping[camera_name]}.json')
        if os.path.exists(json_es39):
            with open(json_es39, 'r') as f:
                camera_config = json.load(f)

            yaml_dict = {
                'image_width': camera_config['image_width'],
                'image_height': camera_config['image_height'],
                'focal_x': camera_config['focal_u'],
                'focal_y': camera_config['focal_v'],
                'center_u': camera_config['center_u'],
                'center_v': camera_config['center_v'],
                'pos_x': camera_config['camera_x'],
                'pos_y': camera_config['camera_y'],
                'pos_z': camera_config['camera_z'],
                'pitch': camera_config['pitch'],
                'roll': camera_config['roll'],
                'yaw': camera_config['yaw'],
                'distort': camera_config['distort'],
                'fov': camera_config['fov'],
            }

            yaml_file = os.path.join(yaml_folder, f'{camera_name}.yaml')

            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_dict, f, encoding='utf-8', allow_unicode=True, sort_keys=False)


def transfer_es39_2_2j5(json_folder_3s39, json_folder_2j5):

    mapping = {
        'fisheye_front': '7',
        'fisheye_rear': '6',
        'fisheye_left': '8',
        'fisheye_right': '9',
        'front': '5',
        'rear': '4',
    }
    temp_2j5_folder = '/home/zhangliwei01/ZONE/horizon/camera'

    for camera_name in mapping.keys():

        json_es39 = os.path.join(json_folder_3s39, f'{camera_name}.json')
        if os.path.exists(json_es39):
            with open(json_es39, 'r') as f:
                es39_camera = json.load(f)

            template_json = os.path.join(temp_2j5_folder, f'camera-{mapping[camera_name]}', 'camera_0.json')
            with open(template_json, 'r') as f:
                temp_camera = json.load(f)

            # es39的外参
            world2camera_es39 = Rotation.from_euler('ZYX', [
                es39_camera['yaw'], es39_camera['pitch'], es39_camera['roll']
            ]).as_matrix()

            # 2j5的外参
            world2calib_2j5 = Rotation.from_euler('XYZ', [
                temp_camera['vcs']['rotation'][0],
                temp_camera['vcs']['rotation'][1],
                temp_camera['vcs']['rotation'][2],
            ]).as_matrix()
            calib2camera_2j5 = np.linalg.inv(world2calib_2j5) @ world2camera_es39
            # yaw, pitch, roll = Rotation.from_matrix(calib2camera_2j5).as_euler('ZYX')
            roll, pitch, yaw = Rotation.from_matrix(calib2camera_2j5).as_euler('XYZ')

            x = es39_camera['camera_x'] - temp_camera['vcs']['translation'][0]
            y = es39_camera['camera_y'] - temp_camera['vcs']['translation'][1]
            z = es39_camera['camera_z'] - temp_camera['vcs']['translation'][2]

            folder_camera_2j5 = os.path.join(json_folder_2j5, f'camera-{mapping[camera_name]}')
            if os.path.exists(folder_camera_2j5):
                shutil.rmtree(folder_camera_2j5)
            os.makedirs(folder_camera_2j5)

            camera_json_2j5 = os.path.join(folder_camera_2j5, 'camera_0.json')
            camera_dict = {
                'base_calib_done': 0,
                'binning_camera_update': 0,
                'calib_done_ts': 0,
                'calib_src': 0,
                'camera_intrinsic_changed': 0,
                'camera_x': x,
                'camera_y': y,
                'camera_z': z,
                'center_u': es39_camera['center_u'],
                'center_v': es39_camera['center_v'],
                'check_state': 1,
                'distort': es39_camera['distort'],
                'error_code': '',
                'focal_u': es39_camera['focal_u'],
                'focal_v': es39_camera['focal_v'],
                'fov': temp_camera['fov'],
                'image_height': es39_camera['image_height'],
                'image_width': es39_camera['image_width'],
                'intric_paramater_from': temp_camera['intric_paramater_from'],
                'is_default': 1,
                'pitch': pitch,
                'roll': roll,
                'sensor_desc': '',
                'serial_num': temp_camera['serial_num'],
                'type': temp_camera['type'],
                'valid_height': temp_camera['valid_height'],
                'vcs': temp_camera['vcs'],
                'vendor': temp_camera['vendor'],
                'version': temp_camera['version'],
                'yaw': yaw,
            }

            with open(camera_json_2j5, 'w+') as f:
                json.dump(camera_dict, f, indent=2)


def transfer_1j5_2_es39(yaml_folder, json_folder):
    """
    将 生成出的1j5 格式的yaml 文件生成 用于es39软件(ES37 2J5 板子) 回灌使用的参数格式
    """
    mapping = {
        'rear': 'camera_2',
        'front': 'camera_5',
        'front_30fov': 'camera_6',
        'fisheye_front': 'camera_7',
        'fisheye_right': 'camera_8',
        'fisheye_rear': 'camera_9',
        'fisheye_left': 'camera_10',
    }

    fov_dict = {
        'rear': 60.0,
        'front': 120.0,
        'front_30fov': 30.0,
        'fisheye_front': 190.0,
        'fisheye_right': 190.0,
        'fisheye_rear': 190.0,
        'fisheye_left': 190.0,
    }
    for pos_camera_name , yaml_camera_num in mapping.items(): # front : camera_5
        print('pos_camera_name , yaml_camera_num' ,pos_camera_name , yaml_camera_num)
        json_camera_dict = {
            "calib_src": 1,
            "camera_intrinsic_changed": 0,
            "need_calibration_flag": False,
            "vendor": "",
            "version": 1,
        }

        with open(os.path.join(yaml_folder, f'{yaml_camera_num}.yaml'), 'r') as camera_yaml:

            camera_yaml_data = yaml.safe_load(camera_yaml)
            camera_num = 'cam_' + yaml_camera_num.split('_')[-1] + '_'

            if 'fisheye' in pos_camera_name:
                json_camera_dict['type'] = 1
                json_camera_dict['vin'] = 'unknown'
                json_camera_dict['camera_x'] = camera_yaml_data[camera_num + 'pos_x']
                json_camera_dict['camera_y'] = camera_yaml_data[camera_num + 'pos_y']
                json_camera_dict['camera_z'] = camera_yaml_data[camera_num + 'pos_z']
                json_camera_dict['center_u'] = camera_yaml_data[camera_num + 'center_u']
                json_camera_dict['center_v'] = camera_yaml_data[camera_num + 'center_v']
                json_camera_dict['focal_u'] = camera_yaml_data[camera_num + 'focal_x']
                json_camera_dict['focal_v'] = camera_yaml_data[camera_num + 'focal_y']
                json_camera_dict['fov'] = fov_dict[pos_camera_name]
                json_camera_dict['image_height'] = camera_yaml_data[camera_num + 'image_height']
                json_camera_dict['image_width'] = camera_yaml_data[camera_num + 'image_width']
                json_camera_dict['pitch'] = camera_yaml_data[camera_num + 'pitch']
                json_camera_dict['roll'] = camera_yaml_data[camera_num + 'roll']
                json_camera_dict['yaw'] = camera_yaml_data[camera_num + 'yaw']
                json_camera_dict['distort'] = [
                    camera_yaml_data[camera_num + 'distort_k1'],
                    camera_yaml_data[camera_num + 'distort_k2'],
                    camera_yaml_data[camera_num + 'distort_k3'],
                    camera_yaml_data[camera_num + 'distort_k4']
                ]
                json_camera_dict['valid_height'] = [1280, 1280] # 所有鱼眼相机都是 [1280, 1280]

            else:
                json_camera_dict['type'] = 0
                json_camera_dict['vin'] = ''
                json_camera_dict['camera_x'] = camera_yaml_data[camera_num + 'pos_x']
                json_camera_dict['camera_y'] = camera_yaml_data[camera_num + 'pos_y']
                json_camera_dict['camera_z'] = camera_yaml_data[camera_num + 'pos_z']
                json_camera_dict['center_u'] = camera_yaml_data[camera_num + 'center_u']
                json_camera_dict['center_v'] = camera_yaml_data[camera_num + 'center_v']
                json_camera_dict['focal_u'] = camera_yaml_data[camera_num + 'focal_x']
                json_camera_dict['focal_v'] = camera_yaml_data[camera_num + 'focal_y']
                json_camera_dict['fov'] = fov_dict[pos_camera_name]
                json_camera_dict['image_height'] = camera_yaml_data[camera_num + 'image_height']
                json_camera_dict['image_width'] = camera_yaml_data[camera_num + 'image_width']
                json_camera_dict['pitch'] = camera_yaml_data[camera_num + 'pitch']
                json_camera_dict['roll'] = camera_yaml_data[camera_num + 'roll']
                json_camera_dict['yaw'] = camera_yaml_data[camera_num + 'yaw']
                json_camera_dict['distort'] = [
                    camera_yaml_data[camera_num + 'distort_k1'],
                    camera_yaml_data[camera_num + 'distort_k2'],
                    camera_yaml_data[camera_num + 'distort_p1'],
                    camera_yaml_data[camera_num + 'distort_p2'],
                    camera_yaml_data[camera_num + 'distort_k3'],
                    camera_yaml_data[camera_num + 'distort_k4'],
                    camera_yaml_data[camera_num + 'distort_k5'],
                    camera_yaml_data[camera_num + 'distort_k6']
                ]
                if pos_camera_name == 'front':
                    json_camera_dict['valid_height'] = [2160, 2160]
                elif pos_camera_name == 'rear':
                    json_camera_dict['valid_height'] = [1280, 1280]
                elif pos_camera_name == 'front_30fov':
                    json_camera_dict['valid_height'] = [2160, 2160]

        with open(os.path.join(json_folder, f'{pos_camera_name}.json'), 'w+') as camera_json:
            json.dump(json_camera_dict, camera_json, indent=2)


def transfer_q_2_es39(q_config_path, json_folder):
    """
    将 生成出的q格式的json 文件生成 用于es39软件(ES37 2J5 板子) 回灌使用的参数格式
    """
    mapping = {
        'rear': 'CAM_PBQ_REAR',
        'front': 'CAM_PBQ_FRONT_WIDE',
        'fisheye_front': 'CAM_PBQ_FRONT_FISHEYE',
        'fisheye_right': 'CAM_PBQ_RIGHT_FISHEYE',
        'fisheye_rear': 'CAM_PBQ_REAR_FISHEYE',
        'fisheye_left': 'CAM_PBQ_LEFT_FISHEYE',
    }

    fov_dict = {
        'rear': 60.0,
        'front': 120.0,
        'fisheye_front': 196.0,
        'fisheye_right': 196.0,
        'fisheye_rear': 196.0,
        'fisheye_left': 196.0,
    }

    with open(q_config_path, 'r', encoding='utf-8') as file:
        q_config = json.load(file)
    q_cameras = q_config['carInfo']['runParams']['v2VehicleParams']['cameras']

    for z_camera_name, q_camera_name in mapping.items(): # front : camera_5

        json_camera_dict = {
            "calib_src": 1,
            "camera_intrinsic_changed": 0,
            "need_calibration_flag": False,
            "vendor": "",
            "version": 1,
        }

        q_camera = None
        for q_camera in q_cameras:
            if q_camera['installation']['cameraId'] == q_camera_name:
                break

        if q_camera is None:
            continue

        if 'fisheye' in z_camera_name:
            json_camera_dict['type'] = 1
            json_camera_dict['vin'] = 'unknown'
            json_camera_dict['camera_x'] = q_camera['installation']['cameraToVehicleExtrinsics']['x']
            json_camera_dict['camera_y'] = q_camera['installation']['cameraToVehicleExtrinsics']['y']
            json_camera_dict['camera_z'] = q_camera['installation']['cameraToVehicleExtrinsics']['z']
            json_camera_dict['center_u'] = q_camera['inherent']['intrinsics']['cameraMatrix']['cx']
            json_camera_dict['center_v'] = q_camera['inherent']['intrinsics']['cameraMatrix']['cy']
            json_camera_dict['focal_u'] = q_camera['inherent']['intrinsics']['cameraMatrix']['fx']
            json_camera_dict['focal_v'] = q_camera['inherent']['intrinsics']['cameraMatrix']['fy']
            json_camera_dict['fov'] = fov_dict[z_camera_name]
            json_camera_dict['image_height'] = q_camera['common']['height']
            json_camera_dict['image_width'] = q_camera['common']['width']
            json_camera_dict['pitch'] = q_camera['installation']['cameraToVehicleExtrinsics']['pitch']
            json_camera_dict['roll'] = q_camera['installation']['cameraToVehicleExtrinsics']['roll']
            json_camera_dict['yaw'] = q_camera['installation']['cameraToVehicleExtrinsics']['yaw']
            json_camera_dict['distort'] = [
                q_camera['inherent']['intrinsics']['distortCoeffs']['k1'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k2'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k3'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k4'],
            ]
            json_camera_dict['valid_height'] = [1280, 1280] # 所有鱼眼相机都是 [1280, 1280]

        else:
            json_camera_dict['type'] = 0
            json_camera_dict['vin'] = ''
            json_camera_dict['camera_x'] = q_camera['installation']['cameraToVehicleExtrinsics']['x']
            json_camera_dict['camera_y'] = q_camera['installation']['cameraToVehicleExtrinsics']['y']
            json_camera_dict['camera_z'] = q_camera['installation']['cameraToVehicleExtrinsics']['z']
            json_camera_dict['center_u'] = q_camera['inherent']['intrinsics']['cameraMatrix']['cx']
            json_camera_dict['center_v'] = q_camera['inherent']['intrinsics']['cameraMatrix']['cy']
            json_camera_dict['focal_u'] = q_camera['inherent']['intrinsics']['cameraMatrix']['fx']
            json_camera_dict['focal_v'] = q_camera['inherent']['intrinsics']['cameraMatrix']['fy']
            json_camera_dict['fov'] = fov_dict[z_camera_name]
            json_camera_dict['image_height'] = q_camera['common']['height']
            json_camera_dict['image_width'] = q_camera['common']['width']
            json_camera_dict['pitch'] = q_camera['installation']['cameraToVehicleExtrinsics']['pitch']
            json_camera_dict['roll'] = q_camera['installation']['cameraToVehicleExtrinsics']['roll']
            json_camera_dict['yaw'] = q_camera['installation']['cameraToVehicleExtrinsics']['yaw']
            json_camera_dict['distort'] = [
                q_camera['inherent']['intrinsics']['distortCoeffs']['k1'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k2'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['p1'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['p2'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k3'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k4'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k5'],
                q_camera['inherent']['intrinsics']['distortCoeffs']['k6'],
            ]
            if z_camera_name == 'front':
                json_camera_dict['valid_height'] = [2160, 2160]
            elif z_camera_name == 'rear':
                json_camera_dict['valid_height'] = [1280, 1280]
            elif z_camera_name == 'front_30fov':
                json_camera_dict['valid_height'] = [2160, 2160]

        with open(os.path.join(json_folder, f'{z_camera_name}.json'), 'w+') as camera_json:
            json.dump(json_camera_dict, camera_json, indent=2)


if __name__ == '__main__':
    pass
    # json_folder = '/home/caobingqi/下载/2J5'
    # yaml_folder = '/home/caobingqi/下载/1J5'
    # transfer_2j5_2_1j5(json_folder, yaml_folder)
    #
    # calibration_json = '/media/data/kunyi_driving_data/20240119_143055_n000003/Config/20240119_143055_calibration.json'
    # output_folder = '/media/data/kunyi_driving_data/20240119_143055_n000003/Config/456'
    # CJ = ConvertJsonFile2(calibration_json, output_folder)
    #
    # JJ_folder = '/media/data/kunyi_driving_data/20240119_143055_n000003/Config/456'
    # yaml_folder = '/media/data/kunyi_driving_data/20240119_143055_n000003/Config/789'
    # ConvertJsonFile(calibration_json, JJ_folder)
    # transfer_2j5_2_1j5(JJ_folder, yaml_folder)

    # json_folder = '/home/zhangliwei01/ZONE/123'
    # yaml_folder = '/home/zhangliwei01/ZONE/456'
    #
    # transfer_es39_2_1j5(json_folder, yaml_folder)

    # json_folder_es39 = '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_manual_20241205_181840/01_Prediction/20241111_093841_n000013/scenario_info/json_calib'
    # json_folder_2j5 = '/home/zhangliwei01/ZONE/dfg'

    # transfer_es39_2_2j5(json_folder_es39, json_folder_2j5)

    # yaml_folder = '/home/vcar/tmp'
    # json_folder = '/home/vcar/json'
    # transfer_1j5_2_es39(yaml_folder, json_folder)
