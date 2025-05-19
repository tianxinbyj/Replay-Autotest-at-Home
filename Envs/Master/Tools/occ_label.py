import glob
import json
import os
import time
from datetime import datetime

import numpy as np
import open3d as o3d
import pandas as pd


def dict_to_custom_string(data):
    parts = []
    for key, value in data.items():
        if isinstance(value, list):
            # 处理列表（如 matrix4）
            list_str = ", ".join(map(str, value))
            parts.append(f"{key}=[{list_str}]")
        else:
            # 处理其他类型（如字符串、数字）
            parts.append(f"{key}={value}")
    return "{" + ", ".join(parts) + "}"

def get_timestamp(file_name):
    time_str = file_name.split('_')[1]
    dt = datetime.strptime(time_str, "%Y-%m-%d-%H-%M-%S-%f")
    return dt.timestamp()

def match_timestamp(prediction_timestamp, groundtruth_timestamp, match_tolerance):
    rows = []
    for gt_t_idx, gt_t in enumerate(sorted(groundtruth_timestamp)):
        for pred_t_idx, pred_t in enumerate(sorted(prediction_timestamp)):
            delta = abs(gt_t - pred_t)
            if delta < match_tolerance:
                rows.append([gt_t_idx, gt_t, pred_t_idx, pred_t, delta])

    temp_data = pd.DataFrame(rows, columns=['gt_t_idx', 'gt_timestamp', 'pred_t_idx',
                                            'pred_timestamp', 'delta']).sort_values(by=['delta'])

    pred_timestamp, gt_timestamp = [], []
    while len(temp_data):
        gt_timestamp.append(temp_data.at[0, 'gt_timestamp'])
        gt_t_idx = temp_data.at[0, 'gt_t_idx']
        pred_timestamp.append(temp_data.at[0, 'pred_timestamp'])
        pred_t_idx = temp_data.at[0, 'pred_t_idx']
        temp_data.drop(temp_data[(temp_data['gt_t_idx'] == gt_t_idx)
                                 | (temp_data['pred_t_idx'] == pred_t_idx)].index, axis=0, inplace=True)
        if len(temp_data):
            temp_data = temp_data.sort_values(by=['delta']).reset_index(drop=True)
        else:
            break

    return zip(*sorted(zip(pred_timestamp, gt_timestamp)))

final_csv_path = '/media/sf_VBshare/pcd/final.csv'
final_csv_data = pd.read_csv(final_csv_path, index_col=0)
pandar128_compensate_folder = '/media/sf_VBshare/pcd/Pandar128_Compensate'
pandar128_seg_folder = '/media/sf_VBshare/pcd/pandarSEG'
trackOD_folder = '/media/sf_VBshare/pcd/trackOD'
meta_json_path = '/media/sf_VBshare/pcd/meta.json'
image_folder = '/media/sf_VBshare/pcd/Images'
save_dir = '/media/sf_VBshare/pcd/Test'
columns = final_csv_data.columns
rows = []
new_final_csv_path = '/media/sf_VBshare/pcd/new_final.csv'

# 汇总lidar_res
od_res_data = {}
for bbid in os.listdir(trackOD_folder):
    od_res_folder = os.path.join(trackOD_folder, bbid, 'cache', 'autolabel_4dod', bbid, 'offline_showformat')
    if os.path.isdir(od_res_folder):
        for od_json_file in glob.glob(os.path.join(od_res_folder, '*.json')):
            with open(od_json_file) as json_file:
                od_json_data = json.load(json_file)
            pred_boxes = []
            pred_labels = []
            pred_sub_labels = []
            pred_scores = []
            for anno in od_json_data['annotations']:
                pred_boxes.append(anno['PC_3D'])
                pred_labels.append(anno['category'])
                pred_sub_labels.append(anno['sub_category'])
            pcd_name = os.path.basename(od_json_file)[:-5]
            od_res_data[pcd_name] = json.dumps({
                'pred_boxes': pred_boxes,
                'pred_labels': pred_labels,
                'pred_sub_labels': pred_sub_labels})

with open(meta_json_path) as json_file:
    meta_json_data = json.load(json_file)

# 生成final_res.csv
frame_index = 0
for pcd_name, info in meta_json_data['meta'].items():
    print(pcd_name)
    pcd_path = os.path.join(pandar128_compensate_folder, pcd_name)
    seg_path = os.path.join(pandar128_seg_folder, pcd_name.replace('.pcd', '.seg'))
    frame_index += 1
    if os.path.exists(seg_path) and os.path.exists(pcd_path) and pcd_name in od_res_data:
        lidar_type = 'Pandar128'
        batch_id = '20250324_145415-n000001'
        vincode = 'LSJWK4095NS119733'
        field = pcd_name
        pcd = pcd_path
        frame_id = info['frame_id']
        imu_state = 'available'
        pcd_type = 2
        ego_timestamp = frame_id
        ego_kinematics = ''
        ego_pose = ''
        sensor = ''
        eventdatatime = info['eventDataTime']
        createtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        updatetime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        dmp_info = ''
        meta_batch_id = batch_id
        fix_version = ''
        frame_timestamp = eventdatatime
        pre_frame = ''
        next_frame = ''
        pre_sweeps = ''
        next_sweeps = ''
        pre_sweeps_path = ''
        next_sweeps_path = ''
        pre_cur_gap_ms = 100
        next_cur_gap_ms = 100
        key = f'{vincode}+{pcd_name}'
        scenario = 'ground_parking'
        od_res = od_res_data[pcd_name]
        seg_res = seg_path
        raw_pose = {
            'matrix4': info['pose'][0]['matrix4'],
            'type': info['pose'][0]['type'],
            'customer': info['pose'][0]['customer']
        }
        pose = dict_to_custom_string(raw_pose)
        row = [
            lidar_type, batch_id, vincode, field, pcd,
            frame_id, imu_state, pcd_type, ego_timestamp, ego_kinematics,
            ego_pose, sensor, eventdatatime, createtime, updatetime,
            dmp_info, meta_batch_id, pcd_path, fix_version,
            frame_timestamp, pre_frame, next_frame,
            frame_index, pre_sweeps, next_sweeps,
            pre_sweeps_path, next_sweeps_path, pre_cur_gap_ms,
            next_cur_gap_ms, key, scenario, od_res, seg_res, pose, json.dumps(info['pose'][0]['matrix4']),
            get_timestamp(pcd_name)
        ]
        rows.append(row)

final_data = pd.DataFrame(rows, columns=list(columns)+['matrix4', 'timestamp'])
pcd_time_stamp = final_data['timestamp'].to_list()

# 对时间辍匹配4V图片
image_data = {}
for image_folder in glob.glob(os.path.join(image_folder, '*FISHEYE*')):
    image_data[os.path.basename(image_folder)] = {}
    for pic in glob.glob(os.path.join(image_folder, '*FISHEYE*jpeg')):
        image_data[os.path.basename(image_folder)][get_timestamp(os.path.basename(pic))] = '/'.join(pic.split('/')[4:])

pcd_timestamp_with_camera_pic = {}
for camera in image_data.keys():
    match_pcd_timestamp, match_camera_timestamp = match_timestamp(pcd_time_stamp, list(image_data[camera].keys()), 0.05)
    d = {
        t1: image_data[camera][t2] for t1, t2 in zip(match_pcd_timestamp, match_camera_timestamp)
    }

    final_data[camera] = final_data['timestamp'].map(d)

final_data = final_data.dropna(subset=list(image_data.keys()))
final_data.to_csv(new_final_csv_path, index=False)

# 遍历分组
start = time.time()
df = pd.read_csv(new_final_csv_path, index_col=False)
print(df.shape,df.columns)
groups = df.groupby('batch_id')
print('len(groups):',len(groups))
df1 = pd.DataFrame()
for group_name, group_data in groups:
    print(f"{group_name}: {len(group_data)}")
    print('group_data.shape:', group_data.shape)
    group_data = group_data.sort_values(by=['frame_id'])
    group_data = group_data.reset_index(drop=True)
    print('group_data:', group_data)
    sampled_df = group_data.iloc[::5]  # 每隔5帧抽一帧，每隔500nm抽一帧
    print('sampled_df.shape:', sampled_df.shape)
    num_segmentation_label_path = len(sampled_df) // 10  ##2HZ抽帧之后的df,每隔10帧保存一个.npy的文件
    print('num_segmentation_label_path:', num_segmentation_label_path)
    per_seg_points = []
    per_frame_list = []

    for index, row in enumerate(sampled_df.iterrows()):##遍历2hz的数据
        _, row = row
        index += 1
        batch_id = row['batch_id']
        key = row['key']
        frame_id = row['frame_id']
        scenario = row['scenario']
        seg_path = row['seg_res']
        with open(seg_path,'r') as f:
            seg_label = np.array(f.readline().strip('"').split(','),dtype=np.dtype('int32')).reshape(-1, 1)##二维数组
        pcd_path = row['pcd_path']
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points)
        print('points.shape,seg_label.shape:',points.shape,seg_label.shape)

        ### 将ego转到global
        points = np.column_stack((points, np.ones((points.shape[0], 1)))).T
        ego2global_mat = np.reshape(json.loads(row['matrix4']), (4,4))
        global_points = (ego2global_mat @ points).transpose(1,0)
        print('global_points.shape:',global_points.shape)
        seg_global = np.hstack([global_points[:,:3],seg_label])
        print('seg_global.shape:',seg_global.shape)
        per_seg_points.append(seg_global)
        per_frame_list.append(frame_id)

        if index % 10 == 0:  ##只有整除10才保存一个拼接帧
            print(len(per_seg_points), per_frame_list)
            merged_seg_points = np.concatenate(per_seg_points, axis=0)
            print('merged_seg_points.shape:', merged_seg_points.shape)
            npy_path = os.path.join(save_dir, str(batch_id) + '_' + str(index) + '.npy')
            np.save(npy_path, merged_seg_points)
            min_value = min(per_frame_list)
            max_value = max(per_frame_list)
            group_data_50 = group_data[(group_data['frame_id'] >= min_value) & (group_data['frame_id'] <= max_value)]
            group_data_50['segmentation_label_path'] = '/'.join(npy_path.split('/')[5:])
            df1 = pd.concat([df1, group_data_50])
            per_seg_points = []
            per_frame_list = []

df1.to_pickle(os.path.join(save_dir, 'res.pkl'))
df1.to_csv(os.path.join(save_dir, 'res.csv'))
end = time.time()

avg_time = end - start
print(avg_time)