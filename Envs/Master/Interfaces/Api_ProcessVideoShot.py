"""
Created on 2024/8/9
@author: Bu Yujun
"""
import argparse
import glob
import sys

import cv2

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.BirdEyeView import *

camera_fov = {
    'CAM_FRONT_120': [-50 * np.pi / 180, 50 * np.pi / 180],
    'CAM_FRONT_LEFT': [-50 * np.pi / 180 + 0.88, 50 * np.pi / 180 + 0.88],
    'CAM_FRONT_RIGHT': [-50 * np.pi / 180 - 0.88, 50 * np.pi / 180 - 0.88],
    'CAM_BACK_LEFT': [-50 * np.pi / 180 + 2.27, 50 * np.pi / 180 + 2.27],
    'CAM_BACK_RIGHT': [-50 * np.pi / 180 - 2.27, 50 * np.pi / 180 - 2.27],
    'CAM_FRONT_30': [-15 * np.pi / 180, 15 * np.pi / 180],
    'CAM_BACK': [-30 * np.pi / 180 + np.pi, 30 * np.pi / 180 + np.pi],
    'CAM_FISHEYE_FRONT': [-90 * np.pi / 180, 90 * np.pi / 180],
    'CAM_FISHEYE_BACK': [-90 * np.pi / 180 + np.pi, 90 * np.pi / 180 + np.pi],
    'CAM_FISHEYE_LEFT': [-90 * np.pi / 180 + np.pi / 2, 90 * np.pi / 180 + np.pi / 2],
    'CAM_FISHEYE_RIGHT': [-90 * np.pi / 180 - np.pi / 2, 90 * np.pi / 180 - np.pi / 2],
}

camera_models = {
    'CAM_FRONT_120': 'opencv_pinhole',
    'CAM_FRONT_LEFT': 'opencv_pinhole',
    'CAM_FRONT_RIGHT': 'opencv_pinhole',
    'CAM_BACK_LEFT': 'opencv_pinhole',
    'CAM_BACK_RIGHT': 'opencv_pinhole',
    'CAM_BACK': 'opencv_pinhole',
    'CAM_FISHEYE_FRONT': 'opencv_fisheye',
    'CAM_FISHEYE_BACK': 'opencv_fisheye',
    'CAM_FISHEYE_LEFT': 'opencv_fisheye',
    'CAM_FISHEYE_RIGHT': 'opencv_fisheye',
}


pred_arrow = cv2.imread(
    os.path.join(get_project_path(), 'Docs', 'Resources', 'Icon', 'pred_arrow.png'),
    cv2.IMREAD_UNCHANGED)
gt_arrow = cv2.imread(
    os.path.join(get_project_path(), 'Docs', 'Resources', 'Icon', 'gt_arrow.png'),
    cv2.IMREAD_UNCHANGED)


class KunyiCameraModel:

    def __init__(self, calibration_json_path):
        with open(calibration_json_path, 'r') as f:
            self.cameras = json.load(f)['camera']

    def register_camera(self, camera_name):
        res = [camera['name'] == camera_name for camera in self.cameras]
        if not any(res):
            return False
        else:
            origin_camera = self.cameras[res.index(True)]
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
            fov_range = camera_fov[origin_camera['name']]
            camera_par = {
                'size': size,
                'extrinsic': extrinsic,
                'intrinsic': intrinsic,
                'distort': distort,
                'xi': xi,
                'fov_range': fov_range,
            }

            return DistortCameraObject(camera_par=camera_par, camera_model=camera_model)


class NICameraModel:

    def __init__(self, calibration_yaml_folder):
        self.cameras = {}
        for f in glob.glob(os.path.join(calibration_yaml_folder, '*.yaml')):
            camera_name = os.path.splitext(os.path.basename(f))[0]
            self.cameras[camera_name] = f

    def register_camera(self, camera_name):
        camera_model = camera_models[camera_name]
        return DistortCameraObject(yaml_file=self.cameras[camera_name], camera_model=camera_model)


def overlay_and_resize(img1, img2, position, target_size=(100, 100), opacity=1.0):
    x, y = position
    img2_resized = cv2.resize(img2, target_size)
    # 提取 img2 的 alpha 通道，并设置透明度
    alpha_channel = (img2_resized[:, :, 3] / 255.0) * opacity
    # 将 img2 的 RGB 部分按照 alpha 通道叠加到 img1 上
    img1[y:y + target_size[1], x:x + target_size[0], :] = (
            (1.0 - alpha_channel[:, :, np.newaxis]) * img1[y:y + target_size[1], x:x + target_size[0], :] +
            alpha_channel[:, :, np.newaxis] * img2_resized[:, :, :3]
    )
    return img1


class ProcessVideoSnap:

    def __init__(self, calibration_json_path=None, calibration_yaml_folder=None, bug_label_info_path=None):
        with open(bug_label_info_path, 'r') as f:
            bug_label_info_list = json.load(f)

        camera_model = {}
        for bug_label_info in bug_label_info_list:
            for camera_name, camera_label_info in bug_label_info['camera_label_info'].items():
                if camera_name not in camera_model:
                    if calibration_json_path:
                        camera_model[camera_name] = KunyiCameraModel(calibration_json_path).register_camera(camera_name)
                    else:
                        camera_model[camera_name] = NICameraModel(calibration_yaml_folder).register_camera(camera_name)

                size = camera_model[camera_name].camera_object.size
                origin_shot = cv2.imread(camera_label_info['origin_shot'], cv2.IMREAD_UNCHANGED)
                process_shot_path = camera_label_info['process_shot']

                for one_label_info in camera_label_info['label_info']:

                    if 'pred_arrow' in one_label_info:
                        r = camera_model[camera_name].world2camera_with_distort(*one_label_info['pred_arrow'])
                        if r:
                            u, v, limit_u, limit_v = r
                            arrow_size = round(limit_u / 15)
                            if (arrow_size / 2 < u < limit_u - arrow_size / 2
                                    and arrow_size < v < limit_v - arrow_size):
                                p = (int(u - arrow_size / 2), int(v - arrow_size))
                                s = (arrow_size, arrow_size)
                                origin_shot = overlay_and_resize(origin_shot, pred_arrow, p, s, 0.5)

                    if 'gt_arrow' in one_label_info:
                        r = camera_model[camera_name].world2camera_with_distort(*one_label_info['gt_arrow'])
                        if r:
                            u, v, limit_u, limit_v = r
                            arrow_size = round(limit_u / 15)
                            if (arrow_size / 2 < u < limit_u - arrow_size / 2
                                    and arrow_size < v < limit_v - arrow_size):
                                p = (int(u - arrow_size / 2), int(v - arrow_size))
                                s = (arrow_size, arrow_size)
                                origin_shot = overlay_and_resize(origin_shot, gt_arrow, p, s, 0.5)

                    if 'gt_corner' in one_label_info:
                        corner_uv = {
                            'bottom': [camera_model[camera_name].world2camera_with_distort(*pt, cut_flag=False)
                                       for pt in one_label_info['gt_corner']['bottom']],
                            'top': [camera_model[camera_name].world2camera_with_distort(*pt, cut_flag=False)
                                    for pt in one_label_info['gt_corner']['top']],
                        }

                        for i in range(4):
                            try:
                                cv2.line(origin_shot, tuple(corner_uv['bottom'][i]), tuple(corner_uv['bottom'][(i + 1) % 4]),
                                         (0, 0, 200), 2)
                            except OverflowError as e:
                                pass

                            try:
                                cv2.line(origin_shot, tuple(corner_uv['top'][i]), tuple(corner_uv['top'][(i + 1) % 4]),
                                         (0, 0, 200), 2)
                            except OverflowError as e:
                                pass

                        for i in range(4):
                            try:
                                cv2.line(origin_shot, tuple(corner_uv['bottom'][i]), tuple(corner_uv['top'][i]), (0, 0, 200), 2)
                            except OverflowError as e:
                                pass

                    if 'pred_corner' in one_label_info:
                        corner_uv = {
                            'bottom': [camera_model[camera_name].world2camera_with_distort(*pt, cut_flag=False)
                                       for pt in one_label_info['pred_corner']['bottom']],
                            'top': [camera_model[camera_name].world2camera_with_distort(*pt, cut_flag=False)
                                    for pt in one_label_info['pred_corner']['top']],
                        }

                        for i in range(4):
                            try:
                                cv2.line(origin_shot, tuple(corner_uv['bottom'][i]), tuple(corner_uv['bottom'][(i + 1) % 4]),
                                         (200, 0, 0), 2)
                            except OverflowError as e:
                                pass

                            try:
                                cv2.line(origin_shot, tuple(corner_uv['top'][i]), tuple(corner_uv['top'][(i + 1) % 4]),
                                         (200, 0, 0), 2)
                            except OverflowError as e:
                                pass

                        for i in range(4):
                            try:
                                cv2.line(origin_shot, tuple(corner_uv['bottom'][i]), tuple(corner_uv['top'][i]), (200, 0, 0), 2)
                            except OverflowError as e:
                                pass

                    if 'center' in one_label_info:
                        pt = one_label_info['center']
                        text_x, text_y = camera_model[camera_name].world2camera_with_distort(*pt, cut_flag=False)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        font_scale = origin_shot.shape[1] / 1600
                        font_color = (0, 20, 0)
                        font_thickness = round(font_scale * 1.5)
                        text = one_label_info['bug_type']
                        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
                        text_start_x = text_x - text_width // 2
                        text_start_y = text_y - text_height // 2 + baseline
                        cv2.putText(origin_shot, text, (text_start_x, text_start_y - round(size[0] / 15)), font, font_scale, font_color, font_thickness)

                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = origin_shot.shape[1] / 1200
                font_color = (0, 0, 255)
                font_thickness = round(font_scale * 1.5)
                text = f'{bug_label_info["scenario_id"]}-{camera_name}@frame-{bug_label_info["frame_index"]}'
                (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
                text_x = (origin_shot.shape[1] - text_width) // 2
                text_y = text_height + baseline
                cv2.putText(origin_shot, text, (text_x, text_y), font, font_scale, font_color, font_thickness)

                cv2.imwrite(process_shot_path, origin_shot)


def main():
    parser = argparse.ArgumentParser(description="process video shot")
    parser.add_argument("-c", "--calibration", type=str, required=True, help="calibration file or folder")
    parser.add_argument("-a", "--bug_label_info_json", type=str, required=True, help="bug label info json")
    args = parser.parse_args()

    if os.path.isdir(args.calibration):
        ProcessVideoSnap(calibration_yaml_folder=args.calibration, bug_label_info_path=args.bug_label_info_json)
    else:
        ProcessVideoSnap(calibration_json_path=args.calibration, bug_label_info_path=args.bug_label_info_json)


if __name__ == '__main__':
    main()