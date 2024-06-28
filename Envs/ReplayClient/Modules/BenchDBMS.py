# !/usr/bin/env python
# -*-coding:utf-8 -*-
"""
# File      : BenchDBMS.py
# Time      :2023/6/25 下午2:01
# Author    :Tian Long
"""
import glob
import os
from pathlib import Path

import pandas as pd


def load_all_piece():
    pass


# 用于记录不同编号的台架的回灌片段存放的文件夹的总目录
Bench_with_folder = {
    1: '/media/data',
    2: '/media/data',
    3: '/media/data/choose/abc'
}


class BenchDBMS:
    """
    A Database Management System for a ReplayerBench
    下文中的变量：
        total_folder 指台架数据管理系统的存储的所有片段的文件夹，会便利文件夹下所有片段。
                    不用考虑片段所在文件夹与最大文件夹的层数，只需保证片段所在最大路径dir为固定的5个
                    ,会将 台架号 与 其对应的 total_folder 写入到字典 Bench_with_folder 中记录下来。
        piece一般指一个场景片段，或是所有场景片段
        id_tag_table_space表示id + tag的工作表空间csv文件
        DF_11CAN_11V_2Lidar:表示total—folder总文件路径下所有 单个片段id ，对应的11路CAN 、11路video的路径和 2Lidar 文件的路径 的service 组成的Dataframe
    单个 片段的id  由采集文件夹的时间戳 + 片段数 组成 如： 20230602_144755_n000001
    """

    def __init__(self, bench_num=2):

        self.Bench_num = bench_num

        # 初始化认为本地的 没有对应的片段,加载后会记录本地所有片段的id
        self.local_piece_id_list = []

        # 对应台架号码的总文件夹所在路径
        self.total_folder = Path(Bench_with_folder[self.Bench_num])
        # 与文件格式一致
        self.id_11CAN11V2Lidar_path_df_header = ['id',
                                                 'ADAS1', 'ADAS2', 'ADAS3', 'BKP', 'CH', 'IMU', 'IPS', 'PT', 'Safety',
                                                 'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT',
                                                 'CAM_FISHEYE_BACK', 'CAM_FISHEYE_FRONT', 'CAM_FISHEYE_LEFT',
                                                 'CAM_FISHEYE_RIGHT', 'CAM_FRONT_30', 'CAM_FRONT_120',
                                                 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT',
                                                 'AT128', 'Pandar128']

        # 加载本地的总文件夹下所有视频片段id及其对应的CAN,Video,Lidar的文件路径对应的DataFrame
        self.id_11CAN11V2Lidar_path_DF = self.load_all_piece_for_id_11CAN11V2Lidar_df()
        self.local_piece_id_list = self.id_11CAN11V2Lidar_path_DF.index.to_list()

    def load_all_piece_for_id_11CAN11V2Lidar_df(self):
        """
        加载所有片段对应的id与11路CAN，11路Video所在路径及片段所在文件夹与对应的DataFrame
        :return: A DataFrame
        """

        def any_none_in(series):
            """
            根据datafrme的每行中是否有存在None（该片段缺失某些CAN\Video\Lidar 文件），添加一列 ‘complete’，
            :param series: dataframe的一行
            :return: True-完整   或   False-不完整
            """
            # 所有都有
            if all(series.notnull()):
                # print('all is conmment', series['id'])
                return True
            elif (all(series[['IMU', 'AT128', 'Pandar128']].isnull()) and
                  all(series.drop(['IMU', 'AT128', 'Pandar128']).notnull())):
                # print('IMU', 'AT128', 'Pandar128' 'is none', 'but is ok', series['id'])
                return True
            else:
                return False

        # def lidar_imu_notin(series):

        # 用于记录所有本地的片段产生的 series 的列表集
        id_11CAN11V2Lidar_series_list = []
        # 记录所有片段对应的id,用于做该dataframe的索引
        id_index_list = []
        piece_id_dir_list = ['Tag', 'Lidar', 'CAN_Trace', 'Images', 'Config']
        for root, dirs, file in os.walk(self.total_folder):
            if Path(root).parts[-1] == 'mnt':
                dirs[:] = []  # 忽略当前目录下的子目录
                # print(root)
                continue
            if all(items in piece_id_dir_list for items in dirs) and len(dirs) == 5:
                a_root_series_list, a_root_id_index_list = self.peice_id2series_list(root)
                # print('a_root_series_list, a_root_id_index_list:::', a_root_series_list, a_root_id_index_list)
                # print('a_root_series_list:', a_root_series_list)
                # print('a_root_id_index_list:', a_root_id_index_list)
                # if len(a_root_series_list) == 1:
                #     pass
                if not set(a_root_id_index_list).issubset(set(id_index_list)):
                    id_11CAN11V2Lidar_series_list += a_root_series_list
                    id_index_list += a_root_id_index_list
                # 将dirs置空,后续不会再向下层文件夹继续 os.walk()迭代会从其他文件夹开始
                dirs[:] = []

        # 将读取到的列表转换为dataframe 格式 ， 并判断对应CAN、Video、Lidar文件是否完整，添加对应的 ‘Complete’ 列
        id_11CAN11V2Lidar_DF = pd.DataFrame(id_11CAN11V2Lidar_series_list, index=id_index_list,
                                            columns=self.id_11CAN11V2Lidar_path_df_header)
        # id_11CAN11V2Lidar_DF = id_11CAN11V2Lidar_DF.set_index('id', drop=False, append=False)
        id_11CAN11V2Lidar_DF["Complete"] = id_11CAN11V2Lidar_DF.apply(any_none_in, axis=1, raw=False)
        return id_11CAN11V2Lidar_DF

    def peice_id2series_list(self, root):
        """
        传入视频片段路径，与视频片段总数，返回对应的多个片段 id及其 11路CAN、11V、2Lidar的路径的列表
        若对应片段的某路CAN或VIdeo或lidar数据不存在，则添加为None
        :param root: 视频片段根目录，即以时间戳命名的文件夹
        :return:一个列表 ，包含所有的片段对应的CAN、video lidar的路径。形如[ [片段1的..] ,[片段2的] [片段3的]...]
        """

        single_root_series_list = []
        single_root_id_index_list = []
        # piece_num = len(os.listdir((os.path.join(root, 'Images', 'CAM_FRONT_120'))))
        # 从Images  CAM_FRONT_120 文件夹中，获取所有片段的编号 ,放进piece_num_list 中
        piece_file_list = glob.glob((os.path.join(root, 'Images', 'CAM_FRONT_120', 'n000*.mkv')))
        # piece_num_list = []
        # print(root)
        # print(Path(root).parts[-1])
        if '_n000' in Path(root).parts[-1]:  # root文件夹是分割开的，如 20231128_152430_n000001
            print('find split')
            piece_id = Path(root).parts[-1]
            piece_num_str = piece_id[-7:]  # n000002
            print(piece_num_str)
            single_root_series_list = []

            single_root_id_index_list = [piece_id]
            temp_series_list = [piece_id]
            for col in self.id_11CAN11V2Lidar_path_df_header[1:]:
                try:
                    if col in ['CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT',
                               'CAM_FISHEYE_BACK', 'CAM_FISHEYE_FRONT', 'CAM_FISHEYE_LEFT',
                               'CAM_FISHEYE_RIGHT', 'CAM_FRONT_30', 'CAM_FRONT_120',
                               'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT']:
                        temp_series_list.append(
                            glob.glob(os.path.join(root, '*', col, 'n{:s}*.mkv'.format(piece_num_str)))[0])  # .mkv
                    elif col in ['ADAS1', 'ADAS2', 'ADAS3', 'BKP', 'CH', 'IMU', 'IPS', 'PT', 'Safety']:
                        temp_series_list.append(
                            glob.glob(os.path.join(root, '*', col, 'n{:s}*.asc'.format(piece_num_str)))[0])  # .asc
                    else:
                        temp_series_list.append(
                            glob.glob(os.path.join(root, '*', col, 'n{:s}*'.format(piece_num_str)))[0])  # .pcap等
                except:
                    # print('not find ', col, temp_series_list)
                    temp_series_list.append(None)
            single_root_series_list.append(temp_series_list)
            # single_root_id_index_list.append(temp_series_list)
            return single_root_series_list, single_root_id_index_list
        else:
            # root文件夹是未分割的一个完整批次 如 20231128_152430
            # print('piece_file_list aaaaaaaa',piece_file_list)
            for piece_file in piece_file_list:
                temp_series_list = []
                piece_num_str = Path(piece_file).parts[-1][:7]  # n000002

                # piece_num = int(piece_num_str)
                # piece_num_list.append(piece_num)

                piece_id = str(Path(root).parts[-1]) + '_{:s}'.format(piece_num_str)

                # id
                temp_series_list.append(piece_id)
                single_root_id_index_list.append(piece_id)
                # CAN_Trace
                for col in self.id_11CAN11V2Lidar_path_df_header[1:]:
                    try:
                        if col in ['CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT',
                                   'CAM_FISHEYE_BACK', 'CAM_FISHEYE_FRONT', 'CAM_FISHEYE_LEFT',
                                   'CAM_FISHEYE_RIGHT', 'CAM_FRONT_30', 'CAM_FRONT_120',
                                   'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT']:
                            temp_series_list.append(
                                glob.glob(os.path.join(root, '*', col, '{:s}*.mkv'.format(piece_num_str)))[0])  # .mkv
                        elif col in ['ADAS1', 'ADAS2', 'ADAS3', 'BKP', 'CH', 'IMU', 'IPS', 'PT', 'Safety']:
                            temp_series_list.append(
                                glob.glob(os.path.join(root, '*', col, '{:s}*.asc'.format(piece_num_str)))[0])  # .asc
                        else:
                            temp_series_list.append(
                                glob.glob(os.path.join(root, '*', col, '{:s}*'.format(piece_num_str)))[0])  # .pcap等
                    except:
                        # print('not find ', col, temp_series_list)
                        temp_series_list.append(None)

                single_root_series_list.append(temp_series_list)
            return single_root_series_list, single_root_id_index_list


if __name__ == '__main__':
    # id000 = ['20251001_1949_n000001','NaN','NaN','NaN','NaN','NaN','NaN']
    BenchDBMS1 = BenchDBMS()
    # BenchDBMS1.load_local_ids_into_csv()
    # print(BenchDBMS1.id_tag_table_space_path)
    #
    # print('/////////////////////////')
    # print(BenchDBMS1.piece_id_tag_tablespace)

    print('///////////////////////')
    print(BenchDBMS1.id_11CAN11V2Lidar_path_DF)
    print('///////////////////////')

    # aaa = BenchDBMS1.piece_id_tag_tablespace[(BenchDBMS1.piece_id_tag_tablespace['Comments'] == 'left') & (BenchDBMS1.piece_id_tag_tablespace['at_local'] is True)]
    # aaa = BenchDBMS1.piece_id_tag_tablespace[(BenchDBMS1.piece_id_tag_tablespace['Comments'] == 'left')]
    # aaa = BenchDBMS1.piece_id_tag_tablespace[(BenchDBMS1.piece_id_tag_tablespace['at_local'] == True) & (
    #         BenchDBMS1.piece_id_tag_tablespace['Comments'] == 'left')]
    # BenchDBMS1.piece_id_tag_tablespace = BenchDBMS1.piece_id_tag_tablespace.drop()
    # BenchDBMS1.save_id_tag_tablespace2csv()
    # path111 = '/media/data/abcde.csv'
    # BenchDBMS1.id_11CAN11V2Lidar_path_DF.to_csv(path111)
