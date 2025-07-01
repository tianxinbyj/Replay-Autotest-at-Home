import json
import os
import shutil
import re
from google.protobuf import text_format
import google.protobuf.descriptor as descriptor
import importlib.util
import os
import sys
from google.protobuf.json_format import ParseDict


from Docs.Resources.proto.camera_inherent_pb2 import *
from Docs.Resources.proto.camera_installation_ox03cd_pb2 import *
from Docs.Resources.proto.carId_pb2 import *
# from Docs.Resources.proto.camera_installation_isx031_pb2 import *
from Docs.Resources.proto.camera_installation_isx031_pb2 import *
from Docs.Resources.proto.camera_installation_imx728_pb2 import *

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


def generate_installation_isx031_pbtext(key, installation_dict, output_folder):
    """
    将相机安装配置字典转换为pb.txt格式文本

    参数:
        inherent_dict: 相机安装配置字典，需符合以下格式:
            {
                "camera_id": "CAM_PBQ_REAR_FISHEYE",
                "extrinsics": {
                    "calibration_time": "校准时间",
                    "x": float, "y": float, "z": float,
                    "yaw": float, "pitch": float, "roll": float,
                    "calibration_engineer": "工程师",
                    "calibration_run": "校准标识"
                },
                "ref_lidar_id": "LDR_UNKNOWN",
                "device_path": "/dev/video6",
                "hardware_trigger": True,
                "rotate_90_ccw": False,
                "auto_exposure": True,
                "hardware_encoder": True,
                "flip_x": False,
                "full_undistort_fov": False,
                "warp_perspective": False,
                "camera_to_vehicle_extrinsics": {
                    "calibration_time": "校准时间",
                    "x": float, "y": float, "z": float,
                    "yaw": float, "pitch": float, "roll": float
                },
                "pipeline_id": 8,
                "destination_node_name": "main_node_0"
            }
        output_file: 输出文件名，若为None则返回文本内容

    返回:
        若output_file为None，返回生成的pb.txt文本内容，否则返回None
    """
    # 创建根消息对象
    camera_config = CameraInstallationConfig_isx031()

    # 处理camera_id字段（字符串类型）
    camera_config.camera_id = str(installation_dict.get("camera_id", ""))

    # 处理extrinsics嵌套字段
    if "extrinsics" in installation_dict:
        extrinsics = installation_dict["extrinsics"]
        pb_extrinsics = camera_config.extrinsics
        pb_extrinsics.calibration_time = extrinsics.get("calibration_time", "")
        pb_extrinsics.x = float(extrinsics.get("x", 0.0))
        pb_extrinsics.y = float(extrinsics.get("y", 0.0))
        pb_extrinsics.z = float(extrinsics.get("z", 0.0))
        pb_extrinsics.yaw = float(extrinsics.get("yaw", 0.0))
        pb_extrinsics.pitch = float(extrinsics.get("pitch", 0.0))
        pb_extrinsics.roll = float(extrinsics.get("roll", 0.0))
        pb_extrinsics.calibration_engineer = extrinsics.get("calibration_engineer", "")
        pb_extrinsics.calibration_run = extrinsics.get("calibration_run", "")
        # pb_extrinsics.calibration_mode = extrinsics.get("calibration_mode", "")

    # 处理ref_lidar_id字段（字符串类型）
    camera_config.ref_lidar_id = installation_dict.get("ref_lidar_id", "")

    # 处理device_path字段
    camera_config.device_path = installation_dict.get("device_path", "")

    # 处理布尔字段（强制设置，包括false）
    camera_config.hardware_trigger = bool(installation_dict.get("hardware_trigger", False))
    camera_config.rotate_90_ccw = bool(installation_dict.get("rotate_90_ccw", False))
    camera_config.auto_exposure = bool(installation_dict.get("auto_exposure", True))
    camera_config.hardware_encoder = bool(installation_dict.get("hardware_encoder", True))
    camera_config.flip_x = bool(installation_dict.get("flip_x", False))
    camera_config.full_undistort_fov = bool(installation_dict.get("full_undistort_fov", False))
    camera_config.warp_perspective = bool(installation_dict.get("warp_perspective", False))
    camera_config.set_virtual_camera = bool(installation_dict.get("set_virtual_camera", False))
    camera_config.is_calibration = bool(installation_dict.get("is_calibration", False))

    # 处理数值字段（包括0）
    camera_config.expected_fps = int(installation_dict.get("expected_fps", 0))
    camera_config.pipeline_id = int(installation_dict.get("pipeline_id", 0))

    # 处理camera_to_vehicle_extrinsics嵌套字段
    if "camera_to_vehicle_extrinsics" in installation_dict:
        veh_extrinsics = installation_dict["camera_to_vehicle_extrinsics"]
        pb_veh_extrinsics = camera_config.camera_to_vehicle_extrinsics
        pb_veh_extrinsics.calibration_time = veh_extrinsics.get("calibration_time", "")
        pb_veh_extrinsics.calibration_engineer = veh_extrinsics.get("calibration_engineer", "")
        pb_veh_extrinsics.x = float(veh_extrinsics.get("x", 0.0))
        pb_veh_extrinsics.y = float(veh_extrinsics.get("y", 0.0))
        pb_veh_extrinsics.z = float(veh_extrinsics.get("z", 0.0))
        pb_veh_extrinsics.yaw = float(veh_extrinsics.get("yaw", 0.0))
        pb_veh_extrinsics.pitch = float(veh_extrinsics.get("pitch", 0.0))
        pb_veh_extrinsics.roll = float(veh_extrinsics.get("roll", 0.0))
        pb_veh_extrinsics.calibration_mode = veh_extrinsics.get("calibration_mode", "")
        # pb_veh_extrinsics.calibration_run = veh_extrinsics.get("calibration_run", "")

    # 处理virtual_cameras嵌套字段
    if "virtual_cameras" in installation_dict:
        for vc_dict in installation_dict["virtual_cameras"]:
            virtual_camera = camera_config.virtual_cameras.add()
            virtual_camera.camera_id = vc_dict.get("camera_id", "")
            virtual_camera.type = vc_dict.get("type", "")

            # 处理intrinsics
            if "intrinsics" in vc_dict:
                intrinsics = vc_dict["intrinsics"]
                pb_intrinsics = virtual_camera.intrinsics
                pb_intrinsics.fx = float(intrinsics.get("fx", 0.0))
                pb_intrinsics.fy = float(intrinsics.get("fy", 0.0))
                pb_intrinsics.cx = float(intrinsics.get("cx", 0.0))
                pb_intrinsics.cy = float(intrinsics.get("cy", 0.0))

            # 处理数值字段
            virtual_camera.width = int(vc_dict.get("width", 0))
            virtual_camera.height = int(vc_dict.get("height", 0))
            virtual_camera.pipeline_id = int(vc_dict.get("pipeline_id", 0))

            # 处理extrinsics
            if "extrinsics" in vc_dict:
                vc_extrinsics = vc_dict["extrinsics"]
                pb_vc_extrinsics = virtual_camera.extrinsics
                pb_vc_extrinsics.calibration_time = vc_extrinsics.get("calibration_time", "")
                pb_vc_extrinsics.calibration_engineer = vc_extrinsics.get("calibration_engineer", "")
                pb_vc_extrinsics.yaw = float(vc_extrinsics.get("yaw", 0.0))
                pb_vc_extrinsics.pitch = float(vc_extrinsics.get("pitch", 0.0))
                pb_vc_extrinsics.roll = float(vc_extrinsics.get("roll", 0.0))
                pb_vc_extrinsics.calibration_mode = vc_extrinsics.get("calibration_mode", "")
                # pb_vc_extrinsics.calibration_run = vc_extrinsics.get("calibration_run", "")
                # pb_vc_extrinsics.x = float(vc_extrinsics.get("x", 0.0))
                # pb_vc_extrinsics.y = float(vc_extrinsics.get("y", 0.0))
                # pb_vc_extrinsics.z = float(vc_extrinsics.get("z", 0.0))

            # 处理calib_extrinsics
            if "calib_extrinsics" in vc_dict:
                for vc_calib_extrinsics in vc_dict["calib_extrinsics"]:
                    pb_vc_calib_extrinsics = virtual_camera.calib_extrinsics.add()
                    pb_vc_calib_extrinsics.calibration_time = vc_calib_extrinsics.get("calibration_time", "")
                    pb_vc_calib_extrinsics.calibration_engineer = vc_calib_extrinsics.get("calibration_engineer", "")
                    pb_vc_calib_extrinsics.yaw = float(vc_calib_extrinsics.get("yaw", 0.0))
                    pb_vc_calib_extrinsics.pitch = float(vc_calib_extrinsics.get("pitch", 0.0))
                    pb_vc_calib_extrinsics.roll = float(vc_calib_extrinsics.get("roll", 0.0))
                    pb_vc_calib_extrinsics.calibration_mode = vc_calib_extrinsics.get("calibration_mode", "")

    # 处理destination_node_name字段
    camera_config.destination_node_name = installation_dict.get("destination_node_name", "")
    # 自定义序列化，确保所有字段都显示

    pb_text = _custom_serialize(camera_config)
    with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
        f.write(pb_text)


def generate_installation_imx728_pbtext(key, installation_dict, output_folder):
    """
    将相机安装配置字典转换为pb.txt格式文本，确保空值、false和0都显示

    参数:
        installation_dict: 相机安装配置字典，需符合proto结构
        output_file: 输出文件名，若为None则返回文本内容

    返回:
        若output_file为None，返回生成的pb.txt文本内容，否则返回None
    """
    # 创建根消息对象
    camera_config = CameraInstallationConfig_imx728()

    # 处理camera_id字段（字符串类型）
    camera_config.camera_id = installation_dict.get("camera_id", "")

    # 处理extrinsics嵌套字段
    if "extrinsics" in installation_dict:
        extrinsics = installation_dict["extrinsics"]
        pb_extrinsics = camera_config.extrinsics
        pb_extrinsics.calibration_time = extrinsics.get("calibration_time", "")
        pb_extrinsics.x = float(extrinsics.get("x", 0.0))
        pb_extrinsics.y = float(extrinsics.get("y", 0.0))
        pb_extrinsics.z = float(extrinsics.get("z", 0.0))
        pb_extrinsics.yaw = float(extrinsics.get("yaw", 0.0))
        pb_extrinsics.pitch = float(extrinsics.get("pitch", 0.0))
        pb_extrinsics.roll = float(extrinsics.get("roll", 0.0))
        pb_extrinsics.calibration_engineer = extrinsics.get("calibration_engineer", "")
        pb_extrinsics.calibration_run = extrinsics.get("calibration_run", "")
        # pb_extrinsics.calibration_mode = extrinsics.get("calibration_mode", "")

    # 处理ref_lidar_id字段（字符串类型）
    camera_config.ref_lidar_id = installation_dict.get("ref_lidar_id", "")

    # 处理device_path字段
    camera_config.device_path = installation_dict.get("device_path", "")

    # 处理布尔字段（强制设置，包括false）
    camera_config.hardware_trigger = bool(installation_dict.get("hardware_trigger", False))
    camera_config.rotate_90_ccw = bool(installation_dict.get("rotate_90_ccw", False))
    camera_config.auto_exposure = bool(installation_dict.get("auto_exposure", True))
    camera_config.hardware_encoder = bool(installation_dict.get("hardware_encoder", True))
    camera_config.flip_x = bool(installation_dict.get("flip_x", False))
    camera_config.full_undistort_fov = bool(installation_dict.get("full_undistort_fov", False))
    camera_config.warp_perspective = bool(installation_dict.get("warp_perspective", False))
    camera_config.set_virtual_camera = bool(installation_dict.get("set_virtual_camera", False))
    camera_config.is_calibration = bool(installation_dict.get("is_calibration", False))

    # 处理数值字段（包括0）
    camera_config.expected_fps = int(installation_dict.get("expected_fps", 0))
    camera_config.pipeline_id = int(installation_dict.get("pipeline_id", 0))

    # 处理used_run字段
    camera_config.used_run = installation_dict.get("used_run", "")

    # 处理camera_to_vehicle_extrinsics嵌套字段
    if "camera_to_vehicle_extrinsics" in installation_dict:
        veh_extrinsics = installation_dict["camera_to_vehicle_extrinsics"]
        pb_veh_extrinsics = camera_config.camera_to_vehicle_extrinsics
        pb_veh_extrinsics.calibration_time = veh_extrinsics.get("calibration_time", "")
        pb_veh_extrinsics.calibration_engineer = veh_extrinsics.get("calibration_engineer", "")
        pb_veh_extrinsics.x = float(veh_extrinsics.get("x", 0.0))
        pb_veh_extrinsics.y = float(veh_extrinsics.get("y", 0.0))
        pb_veh_extrinsics.z = float(veh_extrinsics.get("z", 0.0))
        pb_veh_extrinsics.yaw = float(veh_extrinsics.get("yaw", 0.0))
        pb_veh_extrinsics.pitch = float(veh_extrinsics.get("pitch", 0.0))
        pb_veh_extrinsics.roll = float(veh_extrinsics.get("roll", 0.0))
        pb_veh_extrinsics.calibration_mode = veh_extrinsics.get("calibration_mode", "")
        # pb_veh_extrinsics.calibration_run = veh_extrinsics.get("calibration_run", "")

    # 处理encode_type字段（字符串类型）
    camera_config.encode_type = installation_dict.get("encode_type", "")

    # 处理virtual_cameras嵌套字段
    if "virtual_cameras" in installation_dict:
        for vc_dict in installation_dict["virtual_cameras"]:
            virtual_camera = camera_config.virtual_cameras.add()
            virtual_camera.camera_id = vc_dict.get("camera_id", "")
            virtual_camera.type = vc_dict.get("type", "")

            # 处理intrinsics
            if "intrinsics" in vc_dict:
                intrinsics = vc_dict["intrinsics"]
                pb_intrinsics = virtual_camera.intrinsics
                pb_intrinsics.fx = float(intrinsics.get("fx", 0.0))
                pb_intrinsics.fy = float(intrinsics.get("fy", 0.0))
                pb_intrinsics.cx = float(intrinsics.get("cx", 0.0))
                pb_intrinsics.cy = float(intrinsics.get("cy", 0.0))

            # 处理数值字段
            virtual_camera.width = int(vc_dict.get("width", 0))
            virtual_camera.height = int(vc_dict.get("height", 0))
            virtual_camera.pipeline_id = int(vc_dict.get("pipeline_id", 0))

            # 处理extrinsics
            if "extrinsics" in vc_dict:
                vc_extrinsics = vc_dict["extrinsics"]
                pb_vc_extrinsics = virtual_camera.extrinsics
                pb_vc_extrinsics.calibration_time = vc_extrinsics.get("calibration_time", "")
                pb_vc_extrinsics.calibration_engineer = vc_extrinsics.get("calibration_engineer", "")
                pb_vc_extrinsics.yaw = float(vc_extrinsics.get("yaw", 0.0))
                pb_vc_extrinsics.pitch = float(vc_extrinsics.get("pitch", 0.0))
                pb_vc_extrinsics.roll = float(vc_extrinsics.get("roll", 0.0))
                pb_vc_extrinsics.calibration_mode = vc_extrinsics.get("calibration_mode", "")
                # pb_vc_extrinsics.calibration_run = vc_extrinsics.get("calibration_run", "")
                # pb_vc_extrinsics.x = float(vc_extrinsics.get("x", 0.0))
                # pb_vc_extrinsics.y = float(vc_extrinsics.get("y", 0.0))
                # pb_vc_extrinsics.z = float(vc_extrinsics.get("z", 0.0))

            # 处理calib_extrinsics
            if "calib_extrinsics" in vc_dict:
                for vc_calib_extrinsics in vc_dict["calib_extrinsics"]:
                    pb_vc_calib_extrinsics = virtual_camera.calib_extrinsics.add()
                    pb_vc_calib_extrinsics.calibration_time = vc_calib_extrinsics.get("calibration_time", "")
                    pb_vc_calib_extrinsics.calibration_engineer = vc_calib_extrinsics.get("calibration_engineer", "")
                    pb_vc_calib_extrinsics.yaw = float(vc_calib_extrinsics.get("yaw", 0.0))
                    pb_vc_calib_extrinsics.pitch = float(vc_calib_extrinsics.get("pitch", 0.0))
                    pb_vc_calib_extrinsics.roll = float(vc_calib_extrinsics.get("roll", 0.0))
                    pb_vc_calib_extrinsics.calibration_mode = vc_calib_extrinsics.get("calibration_mode", "")

    # 处理destination_node_name字段
    camera_config.destination_node_name = installation_dict.get("destination_node_name", "")
    # 自定义序列化，确保所有字段都显示

    pb_text = _custom_serialize(camera_config)
    with open(os.path.join(output_folder, f'{key}.pb.txt'), 'w') as f:
        f.write(pb_text)


def generate_installation_ox03cd_pbtext(key, installation_dict, output_folder):
    """
    将相机安装配置字典转换为pb.txt格式文本，确保空值、false和0都显示

    参数:
        installation_dict: 相机安装配置字典，需符合proto结构
        output_file: 输出文件名，若为None则返回文本内容

    返回:
        若output_file为None，返回生成的pb.txt文本内容，否则返回None
    """
    # 创建根消息对象
    camera_config = CameraInstallationConfig_ox03cd()

    # 处理camera_id字段（字符串类型）
    camera_config.camera_id = installation_dict.get("camera_id", "")

    # 处理extrinsics嵌套字段
    if "extrinsics" in installation_dict:
        extrinsics = installation_dict["extrinsics"]
        pb_extrinsics = camera_config.extrinsics
        pb_extrinsics.calibration_time = extrinsics.get("calibration_time", "")
        pb_extrinsics.x = float(extrinsics.get("x", 0.0))
        pb_extrinsics.y = float(extrinsics.get("y", 0.0))
        pb_extrinsics.z = float(extrinsics.get("z", 0.0))
        pb_extrinsics.yaw = float(extrinsics.get("yaw", 0.0))
        pb_extrinsics.pitch = float(extrinsics.get("pitch", 0.0))
        pb_extrinsics.roll = float(extrinsics.get("roll", 0.0))
        pb_extrinsics.calibration_engineer = extrinsics.get("calibration_engineer", "")
        pb_extrinsics.calibration_run = extrinsics.get("calibration_run", "")
        # pb_extrinsics.calibration_mode = extrinsics.get("calibration_mode", "")

    # 处理ref_lidar_id字段（字符串类型）
    camera_config.ref_lidar_id = installation_dict.get("ref_lidar_id", "")

    # 处理device_path字段
    camera_config.device_path = installation_dict.get("device_path", "")

    # 处理布尔字段（强制设置，包括false）
    camera_config.hardware_trigger = bool(installation_dict.get("hardware_trigger", False))
    camera_config.rotate_90_ccw = bool(installation_dict.get("rotate_90_ccw", False))
    camera_config.auto_exposure = bool(installation_dict.get("auto_exposure", True))
    camera_config.hardware_encoder = bool(installation_dict.get("hardware_encoder", True))
    camera_config.flip_x = bool(installation_dict.get("flip_x", False))
    camera_config.full_undistort_fov = bool(installation_dict.get("full_undistort_fov", False))
    camera_config.warp_perspective = bool(installation_dict.get("warp_perspective", False))
    camera_config.set_virtual_camera = bool(installation_dict.get("set_virtual_camera", False))
    camera_config.is_calibration = bool(installation_dict.get("is_calibration", False))

    # 处理数值字段（包括0）
    camera_config.expected_fps = int(installation_dict.get("expected_fps", 0))
    camera_config.pipeline_id = int(installation_dict.get("pipeline_id", 0))

    # 处理camera_to_vehicle_extrinsics嵌套字段
    if "camera_to_vehicle_extrinsics" in installation_dict:
        veh_extrinsics = installation_dict["camera_to_vehicle_extrinsics"]
        pb_veh_extrinsics = camera_config.camera_to_vehicle_extrinsics
        pb_veh_extrinsics.calibration_time = veh_extrinsics.get("calibration_time", "")
        pb_veh_extrinsics.calibration_engineer = veh_extrinsics.get("calibration_engineer", "")
        pb_veh_extrinsics.x = float(veh_extrinsics.get("x", 0.0))
        pb_veh_extrinsics.y = float(veh_extrinsics.get("y", 0.0))
        pb_veh_extrinsics.z = float(veh_extrinsics.get("z", 0.0))
        pb_veh_extrinsics.yaw = float(veh_extrinsics.get("yaw", 0.0))
        pb_veh_extrinsics.pitch = float(veh_extrinsics.get("pitch", 0.0))
        pb_veh_extrinsics.roll = float(veh_extrinsics.get("roll", 0.0))
        pb_veh_extrinsics.calibration_mode = veh_extrinsics.get("calibration_mode", "")
        # pb_veh_extrinsics.calibration_run = veh_extrinsics.get("calibration_run", "")

    # 处理virtual_cameras嵌套字段
    if "virtual_cameras" in installation_dict:
        for vc_dict in installation_dict["virtual_cameras"]:
            virtual_camera = camera_config.virtual_cameras.add()
            virtual_camera.camera_id = vc_dict.get("camera_id", "")
            virtual_camera.type = vc_dict.get("type", "")

            # 处理intrinsics
            if "intrinsics" in vc_dict:
                intrinsics = vc_dict["intrinsics"]
                pb_intrinsics = virtual_camera.intrinsics
                pb_intrinsics.fx = float(intrinsics.get("fx", 0.0))
                pb_intrinsics.fy = float(intrinsics.get("fy", 0.0))
                pb_intrinsics.cx = float(intrinsics.get("cx", 0.0))
                pb_intrinsics.cy = float(intrinsics.get("cy", 0.0))

            # 处理数值字段
            virtual_camera.width = int(vc_dict.get("width", 0))
            virtual_camera.height = int(vc_dict.get("height", 0))
            virtual_camera.pipeline_id = int(vc_dict.get("pipeline_id", 0))

            # 处理extrinsics
            if "extrinsics" in vc_dict:
                vc_extrinsics = vc_dict["extrinsics"]
                pb_vc_extrinsics = virtual_camera.extrinsics
                pb_vc_extrinsics.calibration_time = vc_extrinsics.get("calibration_time", "")
                pb_vc_extrinsics.calibration_engineer = vc_extrinsics.get("calibration_engineer", "")
                pb_vc_extrinsics.yaw = float(vc_extrinsics.get("yaw", 0.0))
                pb_vc_extrinsics.pitch = float(vc_extrinsics.get("pitch", 0.0))
                pb_vc_extrinsics.roll = float(vc_extrinsics.get("roll", 0.0))
                pb_vc_extrinsics.calibration_mode = vc_extrinsics.get("calibration_mode", "")
                # pb_vc_extrinsics.calibration_run = vc_extrinsics.get("calibration_run", "")
                # pb_vc_extrinsics.x = float(vc_extrinsics.get("x", 0.0))
                # pb_vc_extrinsics.y = float(vc_extrinsics.get("y", 0.0))
                # pb_vc_extrinsics.z = float(vc_extrinsics.get("z", 0.0))

            # 处理calib_extrinsics
            if "calib_extrinsics" in vc_dict:
                for vc_calib_extrinsics in vc_dict["calib_extrinsics"]:
                    pb_vc_calib_extrinsics = virtual_camera.calib_extrinsics.add()
                    pb_vc_calib_extrinsics.calibration_time = vc_calib_extrinsics.get("calibration_time", "")
                    pb_vc_calib_extrinsics.calibration_engineer = vc_calib_extrinsics.get("calibration_engineer", "")
                    pb_vc_calib_extrinsics.yaw = float(vc_calib_extrinsics.get("yaw", 0.0))
                    pb_vc_calib_extrinsics.pitch = float(vc_calib_extrinsics.get("pitch", 0.0))
                    pb_vc_calib_extrinsics.roll = float(vc_calib_extrinsics.get("roll", 0.0))
                    pb_vc_calib_extrinsics.calibration_mode = vc_calib_extrinsics.get("calibration_mode", "")

    # 处理destination_node_name字段
    camera_config.destination_node_name = installation_dict.get("destination_node_name", "")
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

    # 处理所有字段
    for field_descriptor in message.DESCRIPTOR.fields:
        field_name = field_descriptor.name
        field_value = getattr(message, field_name)

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


def transform_calib(config_json_path, v2_folder):
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
        if 'isx031' in camera_info['key'].lower():
            print(installation)
            generate_installation_isx031_pbtext(camera_info['key'], installation, installation_folder)
        elif 'imx728' in camera_info['key'].lower():
            print(installation)
            generate_installation_imx728_pbtext(camera_info['key'], installation, installation_folder)
        elif 'ox03cd' in camera_info['key'].lower():
            print(installation)
            generate_installation_ox03cd_pbtext(camera_info['key'], installation, installation_folder)


if __name__ == '__main__':
    config_json_path = 'run_info.json'
    transform_calib(config_json_path, '/home/vcar/tmp')