import json
import os
import shutil
import re

import yaml
from google.protobuf import text_format
import google.protobuf.descriptor as descriptor
import importlib.util
import os
import sys
from google.protobuf.json_format import ParseDict


from Docs.Resources.proto.camera_inherent_pb2 import *
from Docs.Resources.proto.camera_installation_pb2 import *
from Docs.Resources.proto.camera_installation_ox03cd_pb2 import *
from Docs.Resources.proto.carId_pb2 import *
# from Docs.Resources.proto.camera_installation_isx031_pb2 import *
from Docs.Resources.proto.camera_installation_isx031_pb2 import *
from Docs.Resources.proto.camera_installation_imx728_pb2 import *
from Envs.ReplayClient.Modules.BirdEyeView import ConvertJsonFile, transfer_2j5_2_1j5


def camel_to_snake(name):
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


def convert_dict_keys(d):
    if isinstance(d, dict):
        return {camel_to_snake(k): convert_dict_keys(v) for k, v in d.items()}
    elif isinstance(d, list):
        return [convert_dict_keys(item) for item in d]
    else:
        return d


def generate_carId_pbtext(car_id, hardwares, output_folder):
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


def generate_inherent_pbtext(key, inherent_dict, output_folder):
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
        calibration_engineer = intrinsics.get("calibration_engineer", "") if intrinsics.get("calibration_engineer", "") else 'n/a'
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
    pb_text = text_format.MessageToString(camera_config)
    with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
        f.write(pb_text)


def set_output_value(output, data_dict, keys_to_check):
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

def generate_installation_pbtext(key, installation_dict, output_folder):
    # 创建根消息对象
    camera_config = CameraInstallationConfig()

    # 处理camera_id字段（字符串类型）
    camera_config.camera_id = str(installation_dict.get("camera_id", ""))
    set_output_value(camera_config, installation_dict,
                     ["ref_lidar_id", 'device_path', 'hardware_trigger', 'rotate_90_ccw', 'auto_exposure', 'hardware_encoder', 'flip_x', 'full_undistort_fov',
                      'warp_perspective', 'set_virtual_camera', 'is_calibration', 'expected_fps', 'pipeline_id','destination_node_name'])

    # 处理extrinsics嵌套字段
    if "extrinsics" in installation_dict:
        extrinsics = installation_dict["extrinsics"]
        pb_extrinsics = camera_config.extrinsics
        set_output_value(pb_extrinsics, extrinsics, ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer', 'calibration_run', 'calibration_mode'])

    # 处理camera_to_vehicle_extrinsics嵌套字段
    if "camera_to_vehicle_extrinsics" in installation_dict:
        installation_dict["camera_to_vehicle_extrinsics"]['z'] = installation_dict["camera_to_vehicle_extrinsics"]['z'] -0.366
        veh_extrinsics = installation_dict["camera_to_vehicle_extrinsics"]
        pb_veh_extrinsics = camera_config.camera_to_vehicle_extrinsics
        set_output_value(pb_veh_extrinsics, veh_extrinsics,
                         ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer',
                          'calibration_run', 'calibration_mode'])

    # 处理virtual_cameras嵌套字段
    if "virtual_cameras" in installation_dict:
        for vc_dict in installation_dict["virtual_cameras"]:
            virtual_camera = camera_config.virtual_cameras.add()
            set_output_value(virtual_camera, vc_dict,
                             ["camera_id", 'type', 'width', 'height', 'pipeline_id'])

            # 处理intrinsics
            if "intrinsics" in vc_dict:
                intrinsics = vc_dict["intrinsics"]
                pb_intrinsics = virtual_camera.intrinsics
                set_output_value(pb_intrinsics, intrinsics,
                                 ["fx", 'fy', 'cx', 'cy'])

            # 处理extrinsics
            if "extrinsics" in vc_dict:
                vc_extrinsics = vc_dict["extrinsics"]
                pb_vc_extrinsics = virtual_camera.extrinsics
                set_output_value(pb_vc_extrinsics, vc_extrinsics,
                                 ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer',
                                  'calibration_run', 'calibration_mode'])

            # 处理calib_extrinsics
            if "calib_extrinsics" in vc_dict:
                for vc_calib_extrinsics in vc_dict["calib_extrinsics"]:
                    pb_vc_calib_extrinsics = virtual_camera.calib_extrinsics.add()
                    set_output_value(pb_vc_calib_extrinsics, vc_calib_extrinsics,
                                     ["calibration_time", 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'calibration_engineer',
                                      'calibration_run', 'calibration_mode'])
    # 自定义序列化，确保所有字段都显示

    pb_text = _custom_serialize(camera_config)
    with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
        f.write(pb_text)

def generate_pb_txt(key, data_dict, output_folder):
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


def _custom_serialize(message):
    lines = []
    print(message.DESCRIPTOR.fields)

    # 处理所有字段
    for field_descriptor in message.DESCRIPTOR.fields:
        print(field_descriptor.type)
        field_name = field_descriptor.name
        field_value = getattr(message, field_name)

        if field_descriptor.HasOptions() and field_descriptor.GetOptions().HasField('optional'):
            # 处理 optional 字段
           print(field_name)
        else:
            # 处理非 optional 字段，设置默认值
            default_value = field_descriptor.default_value
            if field_descriptor.cpp_type == field_descriptor.CPPTYPE_MESSAGE:
                pass
            else:
                # 设置简单类型字段的默认值
                setattr(message, field_name, default_value)
        # 处理嵌套消息（普通字段）
        if field_descriptor.type == field_descriptor.TYPE_MESSAGE and not field_descriptor.label == field_descriptor.LABEL_REPEATED:
            if field_value:
                lines.append(f"{field_name} {{")
                sub_lines = _custom_serialize(field_value).split('\n')
                for sub_line in sub_lines:
                    if sub_line.strip():
                        lines.append(f"  {sub_line}")
                lines.append("}")
            continue

        # 处理重复字段（repeated）
        if field_descriptor.label == field_descriptor.LABEL_REPEATED:
            for item in field_value:
                lines.append(f"{field_name} {{")
                sub_lines = _custom_serialize(item).split('\n')
                for sub_line in sub_lines:
                    if sub_line.strip():
                        lines.append(f"  {sub_line}")
                lines.append("}")
            continue

        # 处理字符串字段（包括空字符串）
        if field_descriptor.type == field_descriptor.TYPE_STRING:
            lines.append(f'{field_name}: "{field_value}"')
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

def kunyi_to_q_calib(kunyi_folder, q_config_json_path):
    convert_camera_name = {
    'CAM_BACK': 'CAM_PBQ_REAR',
    'CAM_FRONT_120': 'CAM_PBQ_FRONT_WIDE',
    'CAM_FISHEYE_FRONT': 'CAM_PBQ_FRONT_FISHEYE',
    'CAM_FISHEYE_RIGHT': 'CAM_PBQ_RIGHT_FISHEYE',
    'CAM_FISHEYE_BACK': 'CAM_PBQ_REAR_FISHEYE',
    'CAM_FISHEYE_LEFT': 'CAM_PBQ_LEFT_FISHEYE'}
    with open(q_config_json_path, 'r', encoding='utf-8') as file:
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
            q_camera_inherent_config['intrinsics']['distortCoeffs']['k1'] = cameras_config[camera_name].get('distort_k1',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['k2'] = cameras_config[camera_name].get('distort_k2',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['k3'] = cameras_config[camera_name].get('distort_k3',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['k4'] = cameras_config[camera_name].get('distort_k4',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['k5'] = cameras_config[camera_name].get('distort_k5',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['k6'] = cameras_config[camera_name].get('distort_k6',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['p1'] = cameras_config[camera_name].get('distort_p1',0.0)
            q_camera_inherent_config['intrinsics']['distortCoeffs']['p2'] = cameras_config[camera_name].get('distort_p2',0.0)
            q_camera_inherent_config['intrinsics']['cameraMatrix']['fx'] = cameras_config[camera_name].get('focal_x',0.0)
            q_camera_inherent_config['intrinsics']['cameraMatrix']['fy'] = cameras_config[camera_name].get('focal_y',0.0)
            q_camera_inherent_config['intrinsics']['cameraMatrix']['cx'] = cameras_config[camera_name].get('center_u',0.0)
            q_camera_inherent_config['intrinsics']['cameraMatrix']['cy'] = cameras_config[camera_name].get('center_v',0.0)
            q_camera_installation_config = q_camera_config['installation']
            q_camera_installation_config['cameraToVehicleExtrinsics']['x'] = cameras_config[camera_name].get('pos_x',0.0)
            q_camera_installation_config['cameraToVehicleExtrinsics']['y'] = cameras_config[camera_name].get('pos_y',0.0)
            q_camera_installation_config['cameraToVehicleExtrinsics']['z'] = cameras_config[camera_name].get('pos_z',0.0)
            q_camera_installation_config['cameraToVehicleExtrinsics']['yaw'] = cameras_config[camera_name].get('yaw',0.0)
            q_camera_installation_config['cameraToVehicleExtrinsics']['pitch'] = cameras_config[camera_name].get('pitch',0.0)
            q_camera_installation_config['cameraToVehicleExtrinsics']['roll'] = cameras_config[camera_name].get('roll',0.0)
    with open(os.path.join(kunyi_folder, 'q_info.json'), 'w', encoding='utf-8') as file:
        json.dump(q_config, file, indent=2)
    return os.path.join(kunyi_folder, 'q_info.json')



def transform_q_calib(config_json_path, v2_folder):
    if os.path.exists(v2_folder):
        shutil.rmtree(v2_folder)
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
    generate_carId_pbtext(car_id, hardwares, v2_folder)

    # 生成inherent
    inherent_folder = os.path.join(v2_folder, 'camera', 'inherent')
    os.makedirs(inherent_folder)
    if os.path.exists(inherent_folder):
        print("--------------------------")
    for camera_info in camera_config['carInfo']['runParams']['v2VehicleParams']['cameras']:
        inherent = camera_info['inherent']
        inherent = convert_dict_keys(inherent)
        print(inherent)
        generate_inherent_pbtext(camera_info['key'], inherent, inherent_folder)

    # 生成installation
    installation_folder = os.path.join(v2_folder, 'camera', 'installation')
    os.makedirs(installation_folder)
    for camera_info in camera_config['carInfo']['runParams']['v2VehicleParams']['cameras']:
        installation = camera_info['installation']
        installation = convert_dict_keys(installation)
        generate_installation_pbtext(camera_info['key'], installation, installation_folder)
        print(installation)

def transform_kunyi_calib(kunyi_config_json_path, kunyi_folder, q_config_json_path):
    if os.path.exists(kunyi_folder):
        shutil.rmtree(kunyi_folder)
    os.makedirs(kunyi_folder)
    ConvertJsonFile(kunyi_config_json_path, kunyi_folder)
    transfer_2j5_2_1j5(kunyi_folder, kunyi_folder)
    modify_q_config_path = kunyi_to_q_calib(kunyi_folder, q_config_json_path)
    transform_q_calib(modify_q_config_path, os.path.join(kunyi_folder, 'v2'))


if __name__ == '__main__':
    config_json_path = '/media/data/Config/20250529_102333_calibration.json'
    transform_kunyi_calib(config_json_path, '/home/vcar/tmp', 'run_info.json')