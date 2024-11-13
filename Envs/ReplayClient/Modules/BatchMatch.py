import glob
import math
import os

import pathlib
import re
import subprocess
import time
from datetime import datetime

import pandas as pd
import yaml

from Envs.ReplayClient.Interfaces.Libs import get_project_path
from Utils.VideoProcess import parse_video


class BatchMatch:
    def __init__(self, batches_folder, robosense_folder):
        self.robosense_folder = robosense_folder
        self.batches_folder = batches_folder

    def check_is_batch_folder(self, folder_name):
        """
        判断一个文件夹的名字是否是一个batch_id
        """

        # batch id 的 正则表达式模式
        # 其中year是四位数，month是2位数，day是2位数，hour、min、sencond、是2位数，num是长度为6的数字，如000005或 000012
        pattern = r"^(?P<year>\d{4})(?P<month>0[1-9]|1[0-2])(?P<day>0[1-9]|[12]\d|3[01])_" \
                  r"(?P<hour>[01]\d|2[0-3])(?P<min>[0-5]\d)(?P<seconds>[0-5]\d)_n(?P<num>\d{6})$"

        match = re.match(pattern, folder_name)

        if match:
            print("检测到batch id , 匹配成功!")
            return True
            # print("匹配成功!")
            # print("year:", match.group("year"))
            # print("month:", match.group("month"))
            # print("day:", match.group("day"))
            # print("hour:", match.group("hour"))
            # print("min:", match.group("min"))
            # print("seconds:", match.group("seconds"))
            # print("num:", match.group("num"))
        else:
            return False
            # print("不符合格式,该文件夹不是一个batch id 的格式")

    # def read_batch_id_info_from_yaml(self):
    #     info_yaml_path = os.path.join(get_project_path(),'Docs','Resources','scenario_info','scenario_fps.yaml')
    #     with open("BatchMatch.yaml", 'r') as info_yaml_path:

    def get_image_duration(self, video_path):
        # fps, duration = parse_video(image_path)
        # return fps, duration
        # print(video_path)
        # batch_id = os.path.basename(video_path)

        # print()
        # y, m, d, H, M, S, ms = os.path.basename(video_path)[8:].split('_')[0].split('-')
        # start_time = time.mktime(time.strptime(f'{y}-{m}-{d}-{H}-{M}-{S}', "%Y-%m-%d-%H-%M-%S")) + int(ms) / 1000

        batch_id = os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(video_path))))
        # print(batch_id, 'batch_id in get_image_duration')
        scenario_info_yaml = os.path.join(get_project_path(), 'Docs', 'Resources', 'scenario_info', 'scenario_fps.yaml')
        with open(scenario_info_yaml, "r") as info_file:
            scenario_info_dict: dict = yaml.safe_load(info_file)
        if batch_id in scenario_info_dict:
            print(f'straight find batch {batch_id}')
            return scenario_info_dict[batch_id]['fps'], scenario_info_dict[batch_id]['duration']
        else:
            # interface_path = os.path.join(get_project_path(), 'Envs', 'ReplayClient', 'Interfaces')
            # command = [
            #     '/usr/bin/python3', 'Api_GetVideoInfo.py',
            #     '-s', os.path.basename(video_path)
            # ]
            # print('command',command)
            # res = subprocess.run(command,
            #                      cwd=interface_path,
            #                      capture_output=True,
            #                      text=True)
            # print('encode  ',res.stdout.encode('utf-8'))
            # time.sleep(1)

            fps ,duration = parse_video(video_path)
            # n000001-2024-11-08-13-34-40-499_CAM_FRONT_120.mkv
            y, m, d, H, M, S, ms = os.path.basename(video_path)[8:].split('_')[0].split('-')
            start_time = time.mktime(time.strptime(f'{y}-{m}-{d}-{H}-{M}-{S}', "%Y-%m-%d-%H-%M-%S")) + int(ms) / 1000

            video_info = {
                'fps': float(fps),
                'duration': float(duration),
                'start_date': time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time)),
                'start_time': start_time,
            }
            # print('scenario_info_dict:::', scenario_info_dict)
            # print(batch_id, 'batch_id seccond')
            scenario_info_dict[batch_id] = video_info

            with open(scenario_info_yaml, "w") as new_info_file:
                yaml.dump(scenario_info_dict, new_info_file, default_flow_style=False, allow_unicode=True)
            time.sleep(0.5)
            if batch_id in scenario_info_dict:
                return scenario_info_dict[batch_id]['fps'], scenario_info_dict[batch_id]['duration']
            else:
                raise Exception('get video info failed', video_path)

    def get_all_batch_images_path(self, folder_path):
        """
        获取到当前路径下所有的batch id 的 Front120 的视频的的文件路径
        """
        if not os.path.isdir(folder_path):
            raise Exception("Folder path does not exist, {}".format(folder_path))
        for batch_folder in os.listdir(folder_path):
            if batch_folder:
                pass

    def get_batch_id_start_time(self, batch_id):
        """
        获取到一个batch_id 的开始时间， return 一个 unix 时间
        """
        # 正则表达式模式
        pattern = r"^(?P<year>\d{4})(?P<month>0[1-9]|1[0-2])(?P<day>0[1-9]|[12]\d|3[01])_" \
                  r"(?P<hour>[01]\d|2[0-3])(?P<min>[0-5]\d)(?P<seconds>[0-5]\d)_n(?P<num>\d{6})$"

        match = re.match(pattern, batch_id)

        if match:
            # print("匹配成功!")

            # 提取日期和时间信息
            year = int(match.group("year"))
            month = int(match.group("month"))
            day = int(match.group("day"))
            hour = int(match.group("hour"))
            minute = int(match.group("min"))
            second = int(match.group("seconds"))
            num = match.group("num")  # 提取 num 部分

            # 创建一个 datetime 对象
            dt = datetime(year, month, day, hour, minute, second)

            # 转换为 Unix 时间戳
            unix_start_timestamp = int(dt.timestamp()) + (int(num) - 1) * 300

            return unix_start_timestamp

    def get_all_batch_info(self, folder_path):
        """
        获取该 folder_path 所有符合条件的batch id 的 信息
        return ： 一个字典 key = batch id ， value 是一个list [batch_id_folder_path ,images start_time, images duration, fps]
        """
        # 筛选 folder_path 下所有符合条件的 batch_id 对应的folder的路径 , key 是 batch id ，val是一个list，第一个元素为对应folder的path
        batch_id_folders = {batch_id_folder: [os.path.join(folder_path, batch_id_folder)] for batch_id_folder in
                            os.listdir(folder_path) if self.check_is_batch_folder(batch_id_folder)}
        print('aaaaaa222')
        # print(batch_id_folders)
        for batch_id, batch_folder in batch_id_folders.items():
            # print(' batch_id, batch_folder ' , batch_id, batch_folder )
            front120_path = os.path.join(batch_folder[0],'Images', 'CAM_FRONT_120','*.mkv')
            glob_front120_path = glob.glob(front120_path)
            front120_mkv_path = glob_front120_path[0]
            # front120_path : /media/data/test_locate/robo_test_20241108/NI_Replay_Data/20241108_133440_n000001/Images/CAM_FRONT_120/n000001-2024-11-08-13-34-40-499_CAM_FRONT_120.mkv
            fps, duration = self.get_image_duration(front120_mkv_path)
            # print('fps , duration')
            start_time = self.get_batch_id_start_time(batch_id)
            batch_folder += [start_time, duration, fps]
        print('aaaaaa222222')
        return batch_id_folders

    # def get_all_batch_duration(self,batch_ids: list):
    #     """
    #     获取到所有batch id对应的images的duration，返回一个字典，key是对应的batch_id, value 是对应的batch_id视频的duration
    #     images_list： 所有视频的文件路径
    #     """
    #     images_info = {}
    #     for batch_id in batch_ids:
    #         fps, duration =  parse_video(image_path)
    #         images_info[image_batch] = {'fps': fps, 'duration': duration, 'image_path': image_path}
    #
    #     return images_info

    def split_all_batches(self, images_info):
        """
        将所有batch_idx`
        """
        pass

    def get_all_batch_over_3min(self):
        """
        将所有视频中超过3min的片段调出来
        """

    def get_robosense_info(self, GT_csv):
        """
        获取到对应robosense 的gt 的csv文件的 start time （unix） 和 duration
        """
        # with open(GT_csv,'r') as GGGTTT:
        #     data = GGGTTT.readlines()
        GT_df = pd.read_csv(GT_csv)
        # print(GT_df.columns)
        # 获取最大和最小的时间戳
        max_timestamp = GT_df['stamp_sec'].max()
        min_timestamp = GT_df['stamp_sec'].min()

        start_time = min_timestamp
        end_time = max_timestamp
        duration = end_time - start_time

        return start_time, duration, end_time

    def get_all_robosense_start_time_and_duration(self, robosense_folder):
        """
        获取到所有 robosense_folder 的 start time 和 duration ，
        return 一个字典 ，key 是 robosense_folder_path, value 是 一个list [start_time, duration, end_time]
        """

        rbsense_list = os.listdir(robosense_folder)
        robo_gt_folder_ditc = {}
        print(rbsense_list)
        print(robosense_folder)
        for dir in rbsense_list:
            if dir.startswith('.'):
                # 跳过隐藏文件夹
                continue
            # 存在 *_GT.csv 文件的才能视作 一个robosense 的GT片段 对应的文件夹
            # print(os.path.join(robosense_folder, dir, '*_GT.csv'))
            GT_csv_path = glob.glob(os.path.join(robosense_folder, dir, '*_GT.csv'))[0]
            # print(GT_csv_path)
            if not GT_csv_path:
                continue
            elif os.path.exists(GT_csv_path):
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))] = []
                start_time, duration, end_time = self.get_robosense_info(GT_csv_path)
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))].append(start_time)
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))].append(duration)
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))].append(end_time)

        return robo_gt_folder_ditc

    def match_batch_with_robo_2min(self):
        """
        将时间长度超过2min的robosense片段 和 batch id 片段进行筛选， 并将其进行匹配，可能会产生对应的空缺无法匹配的场景。
        return 一个dict key是 batch id ，value 是 对应的gt的片段
        由于少于2min的片段不会参与匹配
        """
        batches_folder = self.batches_folder
        robosense_folder = self.robosense_folder
        print('aaaaaa111')
        # 筛选 batch id 里 时长小于120s的片段
        batches_info = self.get_all_batch_info(batches_folder)
        print('batches_info:', type(batches_info), batches_info)
        print('aaaaaa222')
        #batch_info 的顺序结构 [batch_id_folder_path, images start_time, images duration, fps]
        batch_less120 = [batch_clip for batch_clip, batch_info in batches_info.items() if batch_info[2] <= 120]
        for batch_clip in batch_less120:
            batches_info.pop(batch_clip)

        print('aaaaaa333')
        #  筛选  robosense 里 时长 > 120s 的片段
        robosense_info = self.get_all_robosense_start_time_and_duration(robosense_folder)
        print('robosense_info:', type(robosense_info), robosense_info)
        #  rb_clip_info 的顺序结构 [start_time, duration, end_time]
        rbsense_clip_less120 = [rbsense_clip for rbsense_clip, rb_clip_info in robosense_info.items() if
                                rb_clip_info[1] <= 120]
        for rbsense_clip_ in rbsense_clip_less120:
            robosense_info.pop(rbsense_clip_)

        print('robosense_inforobosense_info', robosense_info)

        print('aaaaaa444')
        # 结果字典：batch_id -> 选择的path
        result = {}

        # 用于记录没有匹配上的数据
        unmatched_batch_ids = []
        unmatched_paths = []
        print('aaaaaa555')
        # 遍历第一个字典，按batch_id进行匹配
        print(batches_info)
        for batch_id, (batch_id_folder_path, batch_start_time, duration, fps) in batches_info.items():
            closest_path = None
            closest_diff = 15  # 设置最小差为15s

            # 遍历第二个字典，找出与batch的start_time最接近的路径
            for rb_path, (rb_start_time, rb_duration, rb__end_time) in robosense_info.items():
                # 计算start_time的差值
                time_diff = abs(batch_start_time - rb_start_time)
                print(time_diff)
                # 如果当前差值更小，更新最接近的路径
                if time_diff < closest_diff:
                    # closest_diff = time_diff
                    closest_path = rb_path
                    # print('find closest path', batch_id, closest_path)
                    break

                # 如果差值大于15，则认为无法匹配
                if time_diff >= closest_diff:
                    print(rb_path, time_diff, 'time diff > 15 ')
                    continue

            # 如果找到了匹配的路径，记录匹配结果
            if closest_path:
                print('match res')
                result[batch_id] = closest_path
            else:
                unmatched_batch_ids.append(batch_id)
        print('aaaaaa666')

        # 记录没有匹配上的robo sense 真值的路径
        for rb_path in robosense_info:
            # 遍历所有的路径，检查是否有未匹配的路径
            # matched = False
            # for batch_id in result:
            #     if batch_less120[batch_id][2] == path:
            #         matched = True
            #         break
            # if not matched:
            if rb_path not in result.values():
                unmatched_paths.append(rb_path)
        print('aaaaaa777')
        print(unmatched_batch_ids)
        print(unmatched_paths)
        print('gggggg')
        for key1, varlue in result.items():
            print(key1, varlue)
        print('aaaaaa888888')
        print('batch_less120:::',batch_less120)
        print('rbsense_clip_less120:::', rbsense_clip_less120)
        return result, unmatched_batch_ids, unmatched_paths, batch_less120, rbsense_clip_less120

    def mv_es37_calib_annotation_mark_unmatch_less120(self):
        """
        将 robosense 的真值与 ni 的采集数据切片结果进行匹配后，将robosense的真值复制到对应的 ni的片段的文件夹中，
        生成Robosense_Annotation文件夹 及下属 es37_gt文件夹
        由于少于2min的片段不会参与匹配， 再将 未匹配到的 ni数据，和 未匹配到robosense数据对应的文件夹进行标记（改名）；
        将ni数据 和 robosense annotation中 少于2min的片段进行标记（修改对应文件夹名）
        """
        match_result, unmatched_batch_ids, unmatched_paths, batch_less120, rbsense_clip_less120  = self.match_batch_with_robo_2min()
        # self.batches_folder
        # self.robosense_folder

        # 将每个匹配到的真值文件夹复制到对应batch id 下，
        for match_batch_id, match_rbsense in match_result.items():
        # match_batch_id = '20241018_160637_n000002'
        # match_rbsense = '/media/data/test_locate/robo_test_20241018/robosense_gt/2024-10-18-16-11-40'
            es37_gt_folder = os.path.join(self.batches_folder, match_batch_id,'Robosense_Annotation','es37_gt',os.path.basename(match_rbsense))
            os.system(f'mkdir -p {es37_gt_folder}')
            time.sleep(0.05)
            rbsense_gt_csv = glob.glob(os.path.join(match_rbsense,'*GT.csv'))[0]
            cp_gtcsv_cmd = 'cp -r {} {}'.format(rbsense_gt_csv,
                                          os.path.join(self.batches_folder, match_batch_id,'Robosense_Annotation','es37_gt',os.path.basename(match_rbsense)))
            # batch id 文件夹下不存在robosense annotation真值的 csv文件时 cp过去
            gt_csv_file_ = glob.glob(os.path.join(self.batches_folder, match_batch_id,'Robosense_Annotation','es37_gt',os.path.basename(match_rbsense),'*_GT.csv'))
            if not gt_csv_file_:
                print('start copy robosense_annotation',cp_gtcsv_cmd)
                os.system(cp_gtcsv_cmd)
            else:
                print('already exists robosense_annotation gt csv file',match_rbsense)

        # 将每个未匹配到的batch_id 的文件夹进行改名
        for unmatch_batch_id in unmatched_batch_ids:
            unmatch_folder = os.path.join(self.batches_folder, unmatch_batch_id)
            if os.path.exists(unmatch_folder):
                mv_cmd = 'mv {} {}'.format(unmatch_folder,unmatch_batch_id + '_unmatched')
                print(mv_cmd)
                res =subprocess.run(mv_cmd,
                                    cwd= self.batches_folder,
                                    shell=True)
            else:
                print(f'不存在此未匹配的文件夹 batch id，{unmatch_folder}')

        # 将每个未匹配到的 robosense 文件夹进行 标记（mv）
        for unmatched_rb_path in unmatched_paths:
            rb_base_name = os.path.basename(unmatched_rb_path)
            rb_dir_name = os.path.dirname(unmatched_rb_path)
            mv_cmd = 'mv {} {}'.format(rb_base_name, rb_base_name + '_unmatched')
            print(mv_cmd)
            if os.path.exists(unmatched_rb_path):
                pass
                res = subprocess.run(mv_cmd,
                                     cwd=rb_dir_name,
                                     shell=True)
            else:
                print(f'不存在此未匹配的文件夹, robosense ，{unmatched_rb_path}')

        # 将每个 less 120s 的 batch id 文件夹进行标记
        for less120_batch_id in batch_less120:
            less120_folder = os.path.join(self.batches_folder, less120_batch_id)
            if os.path.exists(less120_folder):
                mv_cmd = 'mv {} {}'.format(less120_folder,less120_folder + '_less120')
                print(mv_cmd)
                res =subprocess.run(mv_cmd,
                                    cwd= self.batches_folder,
                                    shell=True)
            else:
                print(f'不存在此小于120s的文件夹 batch id，{less120_batch_id}')

        # 将每个 less 120s 的 robosense 文件夹进行标记 rbsense_clip_less120
        for less120_rb_path in rbsense_clip_less120:
            rb_base_name = os.path.basename(less120_rb_path)
            rb_dir_name = os.path.dirname(less120_rb_path)
            mv_cmd = 'mv {} {}'.format(rb_base_name, rb_base_name + '_less120')
            print(mv_cmd)
            if os.path.exists(less120_rb_path):
                pass
                res = subprocess.run(mv_cmd,
                                     cwd=rb_dir_name,
                                     shell=True)
            else:
                print(f'不存在此小于120s的文件夹, robosense ，{less120_rb_path}')

    def clean_robosense_in_batch_folder(self, batch_id):
        """
        清空 batch_id 的文件夹下的 robosense的annotation对应包含的真值文件夹
        """
        rb_anno_in_batch_folder = os.path.join(self.batches_folder, batch_id,'Robosense_Annotation')
        if not os.path.exists(rb_anno_in_batch_folder):
            # raise Exception('path not exist {}'.format(rb_anno_in_batch_folder))
            print(f'batch_id : {batch_id} does not have robosense annotation folder')
        else:
            rm_rb_annotation_cmd = f'rm -r {rb_anno_in_batch_folder}'
            print(rm_rb_annotation_cmd)
            # os.system(rm_rb_annotation_cmd)


if __name__ == '__main__':

    # BM.check_is_batch_folder("20240931_000000_n000001")
    ni_folder = '/media/data/test_locate/robo_test_20241108/NI_Replay_Data'
    rbsense_folder = '/media/data/test_locate/robo_test_20241108/robosense_gt'

    BM = BatchMatch(ni_folder, rbsense_folder)

    BM.match_batch_with_robo_2min()
    BM.mv_es37_calib_annotation_mark_unmatch_less120()
