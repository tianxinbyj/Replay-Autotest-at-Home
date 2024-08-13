"""
Created on 2024/8/9
@author: Bu Yujun
"""
import argparse
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

predicted_arrow = cv2.imread(
    os.path.join(get_project_path(), 'Docs', 'Resources', 'Icon', 'arrow_1.png'),
    cv2.IMREAD_UNCHANGED)
truth_arrow = cv2.imread(
    os.path.join(get_project_path(), 'Docs', 'Resources', 'Icon', 'arrow_2.png'),
    cv2.IMREAD_UNCHANGED)


class CameraModel:

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

    def __init__(self, calibration_json_path, bug_arrow_json_path):
        with open(bug_arrow_json_path, 'r') as f:
            shot_json = json.load(f)

        camera_model = {}
        for shot_info in shot_json:
            for idx, camera_name in enumerate(shot_info['camera']):
                if camera_name not in camera_model:
                    camera_model[camera_name] = CameraModel(calibration_json_path).register_camera(camera_name)

                origin_shot = cv2.imread(shot_info['origin_shot'][idx], cv2.IMREAD_UNCHANGED)

                if 'pred' in shot_info:
                    r = camera_model[camera_name].world2camera_with_distort(*shot_info['pred'])
                    if r:
                        u, v, limit_u, limit_v = r
                        arrow_size = round(limit_u / 15)
                        if (arrow_size / 2 < u < limit_u - arrow_size / 2
                                and arrow_size < v < limit_v - arrow_size):
                            p = (int(u - arrow_size / 2), int(v - arrow_size))
                            s = (arrow_size, arrow_size)
                            origin_shot = overlay_and_resize(origin_shot, predicted_arrow, p, s, 0.5)

                if 'gt' in shot_info:
                    r = camera_model[camera_name].world2camera_with_distort(*shot_info['gt'])
                    if r:
                        u, v, limit_u, limit_v = r
                        arrow_size = round(limit_u / 15)
                        if (arrow_size / 2 < u < limit_u - arrow_size / 2
                                and arrow_size < v < limit_v - arrow_size):
                            p = (int(u - arrow_size / 2), int(v - arrow_size))
                            s = (arrow_size, arrow_size)
                            origin_shot = overlay_and_resize(origin_shot, truth_arrow, p, s, 0.5)

                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = origin_shot.shape[1] / 1200
                font_color = (0, 0, 255)  # BGR格式，这里是红色
                font_thickness = round(font_scale * 1.5)
                text = f'{shot_info["scenario_id"]}-{camera_name}@frame-{shot_info["frame_index"]}'
                (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
                text_x = (origin_shot.shape[1] - text_width) // 2
                text_y = text_height + baseline
                cv2.putText(origin_shot, text, (text_x, text_y), font, font_scale, font_color, font_thickness)

                cv2.imwrite(shot_info['arrow_shot'][idx], origin_shot)

                print(shot_info['arrow_shot'][idx])


def main():
    parser = argparse.ArgumentParser(description="process video shot")
    parser.add_argument("-j", "--calibration_json", type=str, required=True, help="calibration json")
    parser.add_argument("-a", "--bug_arrow_json", type=str, required=True, help="bug arrow json")
    args = parser.parse_args()

    ProcessVideoSnap(args.calibration_json, args.bug_arrow_json)


if __name__ == '__main__':
    main()
