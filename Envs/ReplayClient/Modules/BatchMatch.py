import math
import os

import pathlib
import re
from datetime import datetime

import pandas as pd

from Utils.VideoProcess import parse_video


class BatchMatch:
    def __init__(self):
        pass

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

    def get_image_duration(self,image_path):
        fps, duration = parse_video(image_path)
        return fps, duration

    def get_all_batch_images_path(self, folder_path):
        """
        获取到当前路径下所有的batch id 的 Front120 的视频的的文件路径
        """
        if not os.path.isdir(folder_path):
            raise Exception("Folder path does not exist, {}".format(folder_path))
        for batch_folder in os.listdir(folder_path):
            if batch_folder:
                pass
    def get_batch_id_start_time(self,batch_id):
        """
        获取到一个batch_id 的开始时间， return 一个 unix 时间
        """
        # 正则表达式模式
        pattern = r"^(?P<year>\d{4})(?P<month>0[1-9]|1[0-2])(?P<day>0[1-9]|[12]\d|3[01])_" \
                  r"(?P<hour>[01]\d|2[0-3])(?P<min>[0-5]\d)(?P<seconds>[0-5]\d)_n(?P<num>\d{6})$"

        match = re.match(pattern, batch_id)

        if match:
            print("匹配成功!")

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
        batch_folders = {batch_folder:[os.path.join(folder_path,batch_folder)] for batch_folder in os.listdir(folder_path) if self.check_is_batch_folder(batch_folder)}
        for batch_id, batch_folder in batch_folders.items():
            front120_path = os.path.join(batch_folder[0],'Images', 'CAM_FRONT_120','*.mkv')
            fps , duration = self.get_image_duration(front120_path)
            start_time = self.get_batch_id_start_time(batch_id)
            batch_folder += [start_time, duration, fps]

        return batch_folders


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

        # 获取最大和最小的时间戳
        max_timestamp = GT_df['timestamp'].max()
        min_timestamp = GT_df['timestamp'].min()

        start_time = min_timestamp
        end_time = max_timestamp
        duration = end_time - start_time

        return start_time, duration, end_time


    def get_all_robosense_start_time_and_duration(self,robosense_folder):
        """
        获取到所有 robosense_folder 的 start time 和 duration ，
        return 一个字典 ，key 是 robosense_folder_path, value 是 一个list [start_time, duration, end_time]
        """

        rbsense_list =  os.listdir(robosense_folder)
        robo_gt_folder_ditc = {}
        for dir in rbsense_list:
            # 存在 *_GT.csv 文件的才能视作 一个robosense 的GT片段 对应的文件夹
            if os.path.exists(os.path.join(robosense_folder, dir,'_GT.csv')):
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))] = []
                start_time, duration, end_time = self.get_robosense_info(os.path.join(robosense_folder, dir,'_GT.csv'))
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))].append(start_time)
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))].append(duration)
                robo_gt_folder_ditc[str(os.path.join(robosense_folder, dir))].append(end_time)

        return robo_gt_folder_ditc

    def match_batch_with_robo_3min(self,batches_folder ,robosense_folder):
        """
        将时间长度超过2min的robosense片段 和 batch id 片段进行筛选， 并将其进行匹配，可能会产生对应的空缺无法匹配的场景。
        return 一个dict key是 batch id ，value 是 对应的gt的片段
        """
        
        # 筛选 batch id 里 时长小于120s的片段
        batches_info = self.get_all_batch_info(batches_folder)

        #batch_info 的顺序结构 [batch_id_folder_path, images start_time, images duration, fps]
        batch_less120 = [batch_clip for batch_clip, batch_info in batches_info.items() if batch_info[2] >= 120]
        for batch_clip in batch_less120:
            batches_info.pop(batch_clip)


        #  筛选  robosense 里 时长 > 120s 的片段
        robosense_info = self.get_all_robosense_start_time_and_duration(robosense_folder)

        #  rb_clip_info 的顺序结构 [start_time, duration, end_time]
        rbsense_clip_less120 = [rbsense_clip for rbsense_clip, rb_clip_info in robosense_info.items() if rb_clip_info[1] >= 120]
        for rbsense_clip_ in rbsense_clip_less120:
            robosense_info.pop(rbsense_clip_)




        # 结果字典：batch_id -> 选择的path
        result = {}

        # 用于记录没有匹配上的数据
        unmatched_batch_ids = []
        unmatched_paths = []

        # 遍历第一个字典，按batch_id进行匹配
        for batch_id, (batch_id_folder_path,start_time, duration, fps) in batch_less120.items():
            closest_path = None
            closest_diff = 15  # 设置最小差为15s

            # 遍历第二个字典，找出与batch的start_time最接近的路径
            for rb_path, (rb_start_time, rb_duration, rb__end_time) in rbsense_clip_less120.items():
                # 计算start_time的差值
                time_diff = abs(start_time - rb_start_time)

                # 如果差值大于15，则认为无法匹配
                if time_diff > 15:
                    continue

                # 如果当前差值更小，更新最接近的路径
                if time_diff < closest_diff:
                    closest_diff = time_diff
                    closest_path = rb_path

            # 如果找到了匹配的路径，记录匹配结果
            if closest_path:
                result[batch_id] = closest_path
            else:
                unmatched_batch_ids.append(batch_id)

        # 记录没有匹配上的路径
        for path in rbsense_clip_less120:
            # 遍历所有的路径，检查是否有未匹配的路径
            matched = False
            for batch_id in result:
                if batch_less120[batch_id][2] == path:
                    matched = True
                    break
            if not matched:
                unmatched_paths.append(path)

        print(unmatched_batch_ids)
        print(unmatched_paths)

        return result,unmatched_batch_ids, unmatched_paths








if __name__ == '__main__':
    BM = BatchMatch()
    BM.check_is_batch_folder("20240931_000000_n000001")