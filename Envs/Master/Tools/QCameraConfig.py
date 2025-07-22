import getpass
import glob
import json
import os
import re
import shutil
import subprocess

import yaml
from google.protobuf import text_format
from google.protobuf.json_format import ParseDict

from Docs.Resources.proto.camera_inherent_pb2 import *
from Docs.Resources.proto.camera_installation_imx728_pb2 import *
from Docs.Resources.proto.camera_installation_pb2 import *
from Docs.Resources.proto.carId_pb2 import *
from Envs.Master.Modules.Libs import get_project_path
from Envs.ReplayClient.Modules.BirdEyeView import ConvertJsonFile, transfer_2j5_2_1j5, transfer_1j5_2_es39, transfer_q_2_es39


class QCameraConfig:
    def __init__(self):
        self.demo_v2_path = os.path.join(get_project_path(), 'Docs', 'Resources', 'v2_demo')
        self.quotes_data = self.check_pb_text_demo_with_double_quotes()
        self.q_info_json = os.path.join(get_project_path(), 'Docs', 'Resources', 'q_info_json', 'run_info.json')


    def check_pb_text_demo_with_double_quotes(self):
        """解析protobuf格式的文件并转换为字典"""
        data = {}
        # 指定要遍历的子文件夹
        target_folders = ['inherent', 'installation']
        for target_folder in target_folders:
            for root, dirs, files in os.walk(os.path.join(self.demo_v2_path, 'camera', target_folder)):
                for file in files:
                    file_full_path = os.path.join(root, file)
                    with open(file_full_path, 'r', encoding='utf-8') as f:
                        for line in f:
                            line = line.strip()
                            if not line or line.startswith('#'):
                                continue
                            if ':' in line and '"' in line:
                                data[line.split(':')[0]] = True

        return data

    def camel_to_snake(self, name):
        # 处理 "大写字母+小写字母" 组合，如 "CamelCase" → "Camel_Case"
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
        # 处理 "小写字母+大写字母" 组合，如 "fooBar" → "foo_Bar"
        s2 = re.sub('([a-z])([A-Z])', r'\1_\2', s1)
        # 处理 "数字+大写字母" 组合，如 "90Ccw" → "90_Ccw"
        s3 = re.sub('([0-9])([A-Z])', r'\1_\2', s2)
        # 处理 "小写字母+数字+大写字母" 组合，如 "rotate90Ccw" → "rotate_90_Ccw"
        s4 = re.sub('([a-z])([0-9][A-Z])', r'\1_\2', s3)
        # 全部转换为小写
        return s4.lower()

    def convert_dict_keys(self, d):
        if isinstance(d, dict):
            return {self.camel_to_snake(k): self.convert_dict_keys(v) for k, v in d.items()}
        elif isinstance(d, list):
            return [self.convert_dict_keys(item) for item in d]
        else:
            return d

    def generate_carId_pbtext(self, car_id, hardwares, output_folder):
        """
        生成符合carId.proto格式的pb.txt文件

        参数:
            car_id: 车辆ID字符串
            hardwares: 硬件设备列表，每个元素为包含model、key、type的字典
            output_file: 输出文件名，若为None则返回文本内容

        返回:
            若output_file为None，返回生成的pb.txt文本内容，否则返回None
        """
        # 创建CarConfig消息对象
        car_config = CarConfig()
        car_config.car_id = car_id

        # 处理硬件设备列表
        for hw in hardwares:
            hardware = car_config.hardwares.add()
            hardware.model = hw.get("model", "")
            hardware.key = hw.get("key", "")

            # 处理硬件类型
            type_map = {
                "HARDWARE_CAMERA": HardwareType.HARDWARE_CAMERA,
                "HARDWARE_GNSS": HardwareType.HARDWARE_GNSS,
                "HARDWARE_OMC": HardwareType.HARDWARE_OMC,
                "HARDWARE_RADAR": HardwareType.HARDWARE_RADAR,
                "HARDWARE_VEHICLE": HardwareType.HARDWARE_VEHICLE
            }
            hw_type = hw.get("type", "HARDWARE_UNKNOWN")
            hardware.type = type_map.get(hw_type, HardwareType.HARDWARE_UNKNOWN)

        # 序列化为文本格式
        pb_text = text_format.MessageToString(car_config)
        with open(os.path.join(output_folder, f'{car_id}.pb.txt'), 'w') as f:
            f.write(pb_text)

    def generate_inherent_pbtext(self, key, inherent_dict, output_folder):
        """
        将相机配置字典转换为pb.txt格式文本，自动过滤值为0的p1/p2/k5/k6参数

        参数:
            inherent_dict: 相机配置字典，需包含以下格式:
                {
                    "intrinsics": {
                        "calibration_time": "时间字符串",
                        "distort_coeffs": {
                            "k1": float, "k2": float, "p1": float, "p2": float,
                            "k3": float, "k4": float, "k5": float, "k6": float
                        },
                        "camera_matrix": {"fx": float, "fy": float, "cx": float, "cy": float},
                        "serial_no": "序列号",
                        "calibration_engineer": "工程师",
                        "rms": float
                    },
                    "max_fps": int,
                    "mid_row_time_since_trigger": float,
                    "rolling_elapse_time": float,
                    "camera_vendor": "厂商名称",
                    "serial_no": "序列号"
                }
            output_file: 输出文件名，若为None则返回文本内容

        返回:
            若output_file为None，返回生成的pb.txt文本内容，否则返回None
        """
        # 创建根消息对象
        camera_config = CameraConfig()

        # 处理intrinsics字段
        if "intrinsics" in inherent_dict:
            intrinsics = inherent_dict["intrinsics"]
            pb_intrinsics = camera_config.intrinsics

            # 填充基本字段
            pb_intrinsics.calibration_time = intrinsics.get("calibration_time", "")
            pb_intrinsics.serial_no = intrinsics.get("serial_no", "")
            calibration_engineer = intrinsics.get("calibration_engineer", "") if intrinsics.get("calibration_engineer",
                                                                                                "") else 'n/a'
            pb_intrinsics.calibration_engineer = calibration_engineer
            pb_intrinsics.rms = float(intrinsics.get("rms", 0.0))

            # 处理distort_coeffs嵌套字段（关键过滤逻辑）
            if "distort_coeffs" in intrinsics:
                dist_coeffs = intrinsics["distort_coeffs"]
                pb_dist_coeffs = pb_intrinsics.distort_coeffs

                # 必选字段（无论是否为0都设置）
                pb_dist_coeffs.k1 = float(dist_coeffs.get("k1", 0.0))
                pb_dist_coeffs.k2 = float(dist_coeffs.get("k2", 0.0))
                pb_dist_coeffs.k3 = float(dist_coeffs.get("k3", 0.0))
                pb_dist_coeffs.k4 = float(dist_coeffs.get("k4", 0.0))

                # 可选字段（仅当值不为0时设置）
                if dist_coeffs.get("p1", 0.0) != 0:
                    pb_dist_coeffs.p1 = float(dist_coeffs["p1"])
                if dist_coeffs.get("p2", 0.0) != 0:
                    pb_dist_coeffs.p2 = float(dist_coeffs["p2"])
                if dist_coeffs.get("k5", 0.0) != 0:
                    pb_dist_coeffs.k5 = float(dist_coeffs["k5"])
                if dist_coeffs.get("k6", 0.0) != 0:
                    pb_dist_coeffs.k6 = float(dist_coeffs["k6"])

            # 处理camera_matrix嵌套字段
            if "camera_matrix" in intrinsics:
                cam_matrix = intrinsics["camera_matrix"]
                pb_cam_matrix = pb_intrinsics.camera_matrix
                pb_cam_matrix.fx = float(cam_matrix.get("fx", 0.0))
                pb_cam_matrix.fy = float(cam_matrix.get("fy", 0.0))
                pb_cam_matrix.cx = float(cam_matrix.get("cx", 0.0))
                pb_cam_matrix.cy = float(cam_matrix.get("cy", 0.0))

        # 处理顶层字段
        camera_config.max_fps = int(inherent_dict.get("max_fps", 0))
        camera_config.mid_row_time_since_trigger = float(
            inherent_dict.get("mid_row_time_since_trigger", 0.0)
        )
        camera_config.rolling_elapse_time = float(
            inherent_dict.get("rolling_elapse_time", 0.0)
        )
        camera_config.camera_vendor = inherent_dict.get("camera_vendor", "")
        camera_config.serial_no = inherent_dict.get("serial_no", "")

        # 序列化为文本格式
        pb_text = self._custom_serialize(camera_config)
        with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
            f.write(pb_text)

    def set_output_value(self, output, data_dict, keys_to_check):
        """
        该函数用于根据extrinsics字典设置pb_extrinsics对象的属性

        参数:
        output: 待设置的协议缓冲区对象
        data_dict: 包含键值对的字典
        keys_to_check: 需要检查的键列表

        返回:
        设置好的pb_extrinsics对象
        """
        for key in keys_to_check:
            if key in data_dict:
                setattr(output, key, data_dict[key])
        return output

    def generate_installation_pbtext(self, key, installation_dict, output_folder):
        # 创建根消息对象
        camera_config = CameraInstallationConfig()

        # 处理camera_id字段（字符串类型）
        camera_config.camera_id = str(installation_dict.get("camera_id", ""))
        # camera_config.encode_type = str(installation_dict.get("encode_type", "ENCODE_TYPE_JPEG"))
        self.set_output_value(camera_config, installation_dict,
                         ["ref_lidar_id", 'device_path', 'hardware_trigger', 'rotate_90_ccw', 'auto_exposure',
                          'expected_fps', 'hardware_encoder', 'flip_x', 'used_run', 'full_undistort_fov',
                          'warp_perspective', 'set_virtual_camera', 'pipeline_id', 'destination_node_name',
                          'is_calibration'])

        # 处理extrinsics嵌套字段
        if "extrinsics" in installation_dict:
            extrinsics = installation_dict["extrinsics"]
            pb_extrinsics = camera_config.extrinsics
            self.set_output_value(pb_extrinsics, extrinsics,
                             ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer',
                              'calibration_run', 'calibration_mode'])

        # 处理camera_to_vehicle_extrinsics嵌套字段
        if "camera_to_vehicle_extrinsics" in installation_dict:
            installation_dict["camera_to_vehicle_extrinsics"]['z'] = installation_dict["camera_to_vehicle_extrinsics"][
                                                                         'z'] - 0.336
            veh_extrinsics = installation_dict["camera_to_vehicle_extrinsics"]
            pb_veh_extrinsics = camera_config.camera_to_vehicle_extrinsics
            self.set_output_value(pb_veh_extrinsics, veh_extrinsics,
                             ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer',
                              'calibration_run', 'calibration_mode'])

        # 处理virtual_cameras嵌套字段
        if "virtual_cameras" in installation_dict:
            for vc_dict in installation_dict["virtual_cameras"]:
                virtual_camera = camera_config.virtual_cameras.add()
                self.set_output_value(virtual_camera, vc_dict,
                                 ["camera_id", 'type', 'width', 'height', 'pipeline_id'])

                # 处理intrinsics
                if "intrinsics" in vc_dict:
                    intrinsics = vc_dict["intrinsics"]
                    pb_intrinsics = virtual_camera.intrinsics
                    self.set_output_value(pb_intrinsics, intrinsics,
                                     ["fx", 'fy', 'cx', 'cy'])

                # 处理extrinsics
                if "extrinsics" in vc_dict:
                    vc_extrinsics = vc_dict["extrinsics"]
                    pb_vc_extrinsics = virtual_camera.extrinsics
                    self.set_output_value(pb_vc_extrinsics, vc_extrinsics,
                                     ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer',
                                      'calibration_run', 'calibration_mode'])

                # 处理calib_extrinsics
                if "calib_extrinsics" in vc_dict:
                    for vc_calib_extrinsics in vc_dict["calib_extrinsics"]:
                        pb_vc_calib_extrinsics = virtual_camera.calib_extrinsics.add()
                        self.set_output_value(pb_vc_calib_extrinsics, vc_calib_extrinsics,
                                         ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll',
                                          'calibration_engineer',
                                          'calibration_run', 'calibration_mode'])
        # 自定义序列化，确保所有字段都显示

        pb_text = self._custom_serialize(camera_config)
        with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
            f.write(pb_text)

    def generate_pb_txt(self, key, data_dict, output_folder):
        # 动态加载生成的 pb2 模块
        # spec = importlib.util.spec_from_file_location("camera_installation_imx728_pb2", "camera_installation_imx728_pb2.py")
        # camera_installation_imx728_pb2 = importlib.util.module_from_spec(spec)
        # spec.loader.exec_module(camera_installation_imx728_pb2)

        # 创建 CameraInstallationConfig_imx728 消息对象
        config = CameraInstallationConfig_imx728()

        # 将字典解析为消息对象
        ParseDict(data_dict, config)

        # 将消息对象转换为文本格式
        pb_text = text_format.MessageToString(config, as_utf8=True, float_format='full')

        # 将文本写入文件
        with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
            f.write(pb_text)

    def _custom_serialize(self, message):
        lines = []

        # 处理所有字段
        for field_descriptor in message.DESCRIPTOR.fields:
            # print(field_descriptor.type)
            field_name = field_descriptor.name
            field_value = getattr(message, field_name)
            try:
                if not field_descriptor.type == field_descriptor.TYPE_MESSAGE and not message.HasField(field_name):
                    print(field_name)
                    continue
            except Exception as e:
                pass
            # 处理嵌套消息（普通字段）
            if field_descriptor.type == field_descriptor.TYPE_MESSAGE and not field_descriptor.label == field_descriptor.LABEL_REPEATED:
                if field_value:
                    lines.append(f"{field_name} {{")
                    sub_lines = self._custom_serialize(field_value).split('\n')
                    for sub_line in sub_lines:
                        if sub_line.strip():
                            lines.append(f"  {sub_line}")
                    lines.append("}")
                continue

            # 处理重复字段（repeated）
            if field_descriptor.label == field_descriptor.LABEL_REPEATED:
                for item in field_value:
                    lines.append(f"{field_name} {{")
                    sub_lines = self._custom_serialize(item).split('\n')
                    for sub_line in sub_lines:
                        if sub_line.strip():
                            lines.append(f"  {sub_line}")
                    lines.append("}")
                continue

            # 处理字符串字段（包括空字符串）
            if field_descriptor.type == field_descriptor.TYPE_STRING:
                if field_name in self.quotes_data.keys():
                    lines.append(f'{field_name}: "{field_value}"')
                    continue
                else:
                    lines.append(f'{field_name}: {field_value}')
                    continue

            # 处理布尔字段（包括false）
            if field_descriptor.type == field_descriptor.TYPE_BOOL:
                lines.append(f"{field_name}: {str(field_value).lower()}")
                continue

            # 处理数值字段（包括0）
            if field_descriptor.type in (field_descriptor.TYPE_INT32, field_descriptor.TYPE_INT64,
                                         field_descriptor.TYPE_UINT32, field_descriptor.TYPE_UINT64,
                                         field_descriptor.TYPE_DOUBLE, field_descriptor.TYPE_FLOAT):
                lines.append(f"{field_name}: {field_value}")

        return '\n'.join(lines)

    def kunyi_to_q_calib(self, kunyi_folder):
        convert_camera_name = {
            'CAM_BACK': 'CAM_PBQ_REAR',
            'CAM_FRONT_120': 'CAM_PBQ_FRONT_WIDE',
            'CAM_FISHEYE_FRONT': 'CAM_PBQ_FRONT_FISHEYE',
            'CAM_FISHEYE_RIGHT': 'CAM_PBQ_RIGHT_FISHEYE',
            'CAM_FISHEYE_BACK': 'CAM_PBQ_REAR_FISHEYE',
            'CAM_FISHEYE_LEFT': 'CAM_PBQ_LEFT_FISHEYE'}
        with open(self.q_info_json, 'r', encoding='utf-8') as file:
            q_config = json.load(file)
        with open(os.path.join(kunyi_folder, 'cam_description.yaml'), 'r', encoding='utf-8') as file:
            cameras = yaml.safe_load(file)
        cameras_config = {}
        for camera_num, camera_name in cameras.items():
            with open(os.path.join(kunyi_folder, f'camera_{camera_num}.yaml'), 'r', encoding='utf-8') as file:
                camera_config = yaml.safe_load(file)
            if camera_name in convert_camera_name.keys():
                camera_config_replace_key = {}
                for key, value in camera_config.items():
                    new_key = key.replace(f'cam_{camera_num}_', '')
                    camera_config_replace_key[new_key] = value
                cameras_config[convert_camera_name[camera_name]] = camera_config_replace_key
        q_cameras_config = q_config['carInfo']['runParams']['v2VehicleParams']['cameras']
        for q_camera_config in q_cameras_config:
            if q_camera_config['installation']['cameraId'] in cameras_config.keys():
                print(q_camera_config['installation']['cameraId'])
                camera_name = q_camera_config['installation']['cameraId']
                q_camera_config['common']['width'] = cameras_config[camera_name]['image_width']
                q_camera_config['common']['height'] = cameras_config[camera_name]['image_height']
                q_camera_inherent_config = q_camera_config['inherent']
                q_camera_inherent_config['intrinsics']['distortCoeffs']['k1'] = cameras_config[camera_name].get(
                    'distort_k1', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['k2'] = cameras_config[camera_name].get(
                    'distort_k2', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['k3'] = cameras_config[camera_name].get(
                    'distort_k3', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['k4'] = cameras_config[camera_name].get(
                    'distort_k4', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['k5'] = cameras_config[camera_name].get(
                    'distort_k5', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['k6'] = cameras_config[camera_name].get(
                    'distort_k6', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['p1'] = cameras_config[camera_name].get(
                    'distort_p1', 0.0)
                q_camera_inherent_config['intrinsics']['distortCoeffs']['p2'] = cameras_config[camera_name].get(
                    'distort_p2', 0.0)
                q_camera_inherent_config['intrinsics']['cameraMatrix']['fx'] = cameras_config[camera_name].get(
                    'focal_x', 0.0)
                q_camera_inherent_config['intrinsics']['cameraMatrix']['fy'] = cameras_config[camera_name].get(
                    'focal_y', 0.0)
                q_camera_inherent_config['intrinsics']['cameraMatrix']['cx'] = cameras_config[camera_name].get(
                    'center_u', 0.0)
                q_camera_inherent_config['intrinsics']['cameraMatrix']['cy'] = cameras_config[camera_name].get(
                    'center_v', 0.0)
                q_camera_installation_config = q_camera_config['installation']
                q_camera_installation_config['cameraToVehicleExtrinsics']['x'] = cameras_config[camera_name].get(
                    'pos_x', 0.0)
                q_camera_installation_config['cameraToVehicleExtrinsics']['y'] = cameras_config[camera_name].get(
                    'pos_y', 0.0)
                q_camera_installation_config['cameraToVehicleExtrinsics']['z'] = cameras_config[camera_name].get(
                    'pos_z', 0.0)
                q_camera_installation_config['cameraToVehicleExtrinsics']['yaw'] = cameras_config[camera_name].get(
                    'yaw', 0.0)
                q_camera_installation_config['cameraToVehicleExtrinsics']['pitch'] = cameras_config[camera_name].get(
                    'pitch', 0.0)
                q_camera_installation_config['cameraToVehicleExtrinsics']['roll'] = cameras_config[camera_name].get(
                    'roll', 0.0)
        with open(os.path.join(kunyi_folder, 'q_info.json'), 'w', encoding='utf-8') as file:
            json.dump(q_config, file, indent=2)
        return os.path.join(kunyi_folder, 'q_info.json')

    def transform_q_calib(self, config_json_path, v2_folder):
        if not os.path.exists(v2_folder):
            os.makedirs(v2_folder)

        with open(config_json_path, 'r', encoding='utf-8') as file:
            camera_config = json.load(file)

        # 生成总文件
        car_id = camera_config['carInfo']['carId']
        hardwares = []
        for camera_info in camera_config['carInfo']['runParams']['v2VehicleParams']['cameras']:
            hardwares.append(
                {
                    'model': camera_info['model'],
                    'key': camera_info['key'],
                    'type': 'HARDWARE_CAMERA'
                }
            )

        hardwares.extend([
            {
                'model': 'GNSS_LG69T_ASENSING5115',
                'key': 'GNSS_NOKKJ9NB',
                'type': 'HARDWARE_GNSS'
            },
            {
                'model': 'OMC_J6',
                'key': 'OMC_NAZSQTNZ',
                'type': 'HARDWARE_OMC'
            },
            {
                'model': 'RADAR_CONTI408',
                'key': 'RADAR_7KWZNV9C',
                'type': 'HARDWARE_RADAR'
            },
            {
                'model': 'RADAR_CONTI308',
                'key': 'RADAR_0UGEHIDL',
                'type': 'HARDWARE_RADAR'
            },
            {
                'model': 'RADAR_CONTI308',
                'key': 'RADAR_NOC6WHPJ',
                'type': 'HARDWARE_RADAR'
            },
            {
                'model': 'VEHICLE_ZONE_P_CAR',
                'key': 'PCAR',
                'type': 'HARDWARE_VEHICLE'
            },
        ])
        self.generate_carId_pbtext(car_id, hardwares, v2_folder)

        # 生成inherent
        inherent_folder = os.path.join(v2_folder, 'camera', 'inherent')
        os.makedirs(inherent_folder)
        for camera_info in camera_config['carInfo']['runParams']['v2VehicleParams']['cameras']:
            inherent = camera_info['inherent']
            inherent = self.convert_dict_keys(inherent)
            print(inherent)
            self.generate_inherent_pbtext(camera_info['key'], inherent, inherent_folder)

        # 生成installation
        installation_folder = os.path.join(v2_folder, 'camera', 'installation')
        os.makedirs(installation_folder)
        for camera_info in camera_config['carInfo']['runParams']['v2VehicleParams']['cameras']:
            installation = camera_info['installation']
            installation = self.convert_dict_keys(installation)
            self.generate_installation_pbtext(camera_info['key'], installation, installation_folder)
            print(installation)

    def transform_kunyi_calib(self, kunyi_config_json_path, output_path, docker_path, bin_tool_path, target_car_id='ZP01'):
        """
        通过昆易数据中的标定文件生成v2文件中的相机的内外参文件

        参数:
            kunyi_config_json_path: 昆易数据中的标定文件的地址
            output_path: v2文件输出地址

        """
        if os.path.exists(output_path):
            shutil.rmtree(output_path)
        temp_path = os.path.join(output_path, 'temp')
        os.makedirs(temp_path)
        shutil.copytree(self.demo_v2_path, os.path.join(output_path, 'v2'))
        shutil.rmtree(os.path.join(output_path, 'v2', 'camera', 'installation'))
        shutil.rmtree(os.path.join(output_path, 'v2', 'camera', 'inherent'))
        ConvertJsonFile(kunyi_config_json_path, temp_path)
        transfer_2j5_2_1j5(temp_path, temp_path)
        modify_q_config_path = self.kunyi_to_q_calib(temp_path)
        self.transform_q_calib(modify_q_config_path, temp_path)
        shutil.copytree(os.path.join(temp_path, 'camera', 'installation'),
                        os.path.join(output_path, 'v2', 'camera', 'installation'))
        shutil.copytree(os.path.join(temp_path, 'camera', 'inherent'),
                        os.path.join(output_path, 'v2', 'camera', 'inherent'))
        self.change_car_id(temp_path, target_car_id)
        shutil.copyfile(os.path.join(temp_path, f'{target_car_id}.pb.txt'), os.path.join(output_path, 'v2', f'{target_car_id}.pb.txt'))
        os.makedirs(os.path.join(output_path, 'json'))
        transfer_1j5_2_es39(temp_path, os.path.join(output_path, 'json'))
        if os.path.exists(temp_path):
            shutil.rmtree(temp_path)
        v2_path = self.generate_bin(output_path, docker_path, bin_tool_path, target_car_id)
        return v2_path

    def gen_v2(self, config_path, output_path, docker_path, bin_tool_path, source='qcraft', target_car_id='ZP01'):

        if os.path.exists(output_path):
            shutil.rmtree(output_path)
        temp_path = os.path.join(output_path, 'temp')
        os.makedirs(temp_path)
        shutil.copytree(self.demo_v2_path, os.path.join(output_path, 'v2'))
        shutil.rmtree(os.path.join(output_path, 'v2', 'camera', 'installation'))
        shutil.rmtree(os.path.join(output_path, 'v2', 'camera', 'inherent'))
        os.makedirs(os.path.join(output_path, 'json'))

        if source == 'kunyi':
            ConvertJsonFile(config_path, temp_path)
            transfer_2j5_2_1j5(temp_path, temp_path)
            transfer_1j5_2_es39(temp_path, os.path.join(output_path, 'json'))
            config_path = self.kunyi_to_q_calib(temp_path)
        else:
            transfer_q_2_es39(config_path, os.path.join(output_path, 'json'))

        self.transform_q_calib(config_path, temp_path)
        shutil.copytree(os.path.join(temp_path, 'camera', 'installation'),
                        os.path.join(output_path, 'v2', 'camera', 'installation'))
        shutil.copytree(os.path.join(temp_path, 'camera', 'inherent'),
                        os.path.join(output_path, 'v2', 'camera', 'inherent'))
        self.change_car_id(temp_path, target_car_id)
        shutil.copyfile(os.path.join(temp_path, f'{target_car_id}.pb.txt'), os.path.join(output_path, 'v2', f'{target_car_id}.pb.txt'))

        if os.path.exists(temp_path):
            shutil.rmtree(temp_path)

        return self.generate_bin(output_path, docker_path, bin_tool_path, target_car_id)

    def change_car_id(self, resource_path, car_id):
        try:
            resource_path = glob.glob(os.path.join(resource_path, '*.pb.txt'))[0]
            # 读取文件所有内容
            with open(resource_path, 'r', encoding='utf-8') as file:
                lines = file.readlines()

            if not lines:
                print("错误：文件为空，无法修改第一行")
                return False

            # 修改第一行内容（去除原行首尾空白字符，添加新内容）
            lines[0] = f'car_id: "{car_id}"'

            # 写回修改后的内容
            with open(os.path.join(os.path.dirname(resource_path), f'{car_id}.pb.txt'), 'w', encoding='utf-8') as file:
                file.writelines(lines)
            return True

        except FileNotFoundError:
            print(f"错误：文件 {resource_path} 不存在")
            return False
        except Exception as e:
            print(f"修改文件时发生错误：{e}")
            return False

    def generate_bin(self, output_path, docker_path, bin_tool_path, car_id):
        command = ''
        timeout = 10
        try:
            command = f'{docker_path}'
            # 使用 subprocess 执行 shell 命令
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
                shell=True
            )
            current_user = getpass.getuser()
            command = f'docker start snoah_dev_{current_user}'
            # 使用 subprocess 执行 shell 命令
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
                shell=True
            )

            inner_command = (
                f'cd {bin_tool_path} && '
                f'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:./" ./v2_txt_to_bin '
                f'--vehicle_param_dir={output_path} --only_car={car_id}  2>&1'
            )

            # 第三步：构建 docker exec 命令
            docker_command = [
                'docker', 'exec',
                '-e', f'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{bin_tool_path}',  # 直接设置环境变量
                f'snoah_dev_{current_user}',  # 使用 Python 获取的用户名
                'bash', '-c',  # 在容器内执行 bash 命令
                inner_command  # 要在容器内执行的完整命令
            ]            # 使用 subprocess 执行 shell 命令
            result = subprocess.run(
                docker_command,  # 注意：这里传递的是列表，而不是字符串
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False
            )
            pattern = r'vehicle_param_dir:\s*([^\s]+)'
            match = re.search(pattern, result.stdout)
            if match:
                return match.group(1)  # 返回捕获的路径部分
            else:
                return None

        except subprocess.TimeoutExpired:
            print(f"错误: 命令执行超时（{timeout}秒）: {command}", file=sys.stderr)
            return None
        except Exception as e:
            print(f"错误: 执行命令时发生异常: {command}", file=sys.stderr)
            return None


if __name__ == '__main__':
    camera_config = QCameraConfig()
    kunyi_config_path = '/home/vcar/ZONE/20250102_172710_n000011/Config/20250102_172710_calibration.json'
    q_config_path = '/home/vcar/Replay-Autotest-at-Home/Docs/Resources/q_info_json/run_info.json'
    output_folder = '/home/vcar/ZONE/0715_v2'
    docker_path = '/home/vcar/ZONE/Tools/start_docker.sh'
    bin_tool_path = '/home/vcar/ZONE/Tools/v2_txt_to_bin_tools'
    # v2_path = camera_config.transform_kunyi_calib(kunyi_config_json_path, output_folder, docker_path, bin_tool_path)
    v2_path = camera_config.gen_v2(kunyi_config_path, output_folder, docker_path, bin_tool_path, 'kunyi')
    print(v2_path)