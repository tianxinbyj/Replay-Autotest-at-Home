import glob
import os
import shutil
import time

from Envs.Master.Modules.DataTransformer import DataDownloader, DataTransformer
from Envs.Master.Tools.DataReplayTest import DataReplayTest


class DownloadProcessReplay:

    def __init__(self, install_path, info_path, local_download_path, test_project_path):
        self.install_path = install_path
        self.info_path = info_path
        self.test_project_path = test_project_path
        self.local_download_path = local_download_path
        self.data_downloader = DataDownloader()
        self.data_transformer = DataTransformer(install_path=install_path)

    def run(self):
        download_info = self.data_downloader.read_download_info(self.info_path)
        for info in download_info:
            # 数据下载
            s3_path = f"backup/data/collect/self/driving/{info['package_name']}/{info['vin']}/{info['date']}/"
            src_folder = os.path.join(self.local_download_path, 'download')
            dst_folder = os.path.join(self.local_download_path, f"{info['date']}_{info['video_serial']}")
            dst_folders = glob.glob(os.path.join(dst_folder, '*'))
            if os.path.exists(dst_folder) and os.path.join(dst_folder, 'Images') in dst_folders and os.path.join(dst_folder, 'CAN_Trace') in dst_folders and os.path.join(dst_folder, 'Config') in dst_folders:
                print(f"{dst_folder}已下载")
            else:
                result = self.data_downloader.data_download_from_amazon(s3_path, src_folder, info['video_serial'])
                if not result:
                    return
                if os.path.exists(dst_folder):
                    shutil.rmtree(dst_folder)
                os.makedirs(dst_folder)
                src_folders = glob.glob(os.path.join(src_folder, '*'))
                for folder in src_folders:
                    shutil.move(folder, os.path.join(self.local_download_path, f"{info['date']}_{info['video_serial']}"))
                dst_folders = glob.glob(os.path.join(dst_folder, '*'))
                if len(src_folders) == len(dst_folders) and len(glob.glob(os.path.join(src_folder, '*'))) == 0:
                    print(f"{s3_path}内的{info['video_serial']}已下载至{dst_folder}")
                else:
                    print(f"文件移动至失败{dst_folder}")
                    return
            folders_in_kunyi_package = glob.glob(f"{dst_folder}/*")
            # 数据处理
            if (os.path.join(dst_folder, 'Config') in folders_in_kunyi_package and
                    os.path.join(dst_folder, self.data_transformer.can_file_name) in folders_in_kunyi_package and
                    os.path.join(dst_folder, self.data_transformer.images_file_name) in folders_in_kunyi_package):
                h265_config_path = self.data_transformer.kunyiMkv_to_h265(dst_folder)
                print(h265_config_path)
                time.sleep(1)
                self.data_transformer.h265_to_db3(h265_config_path, os.path.join(dst_folder, self.data_transformer.ros2bag_h265_name))
                self.data_transformer.kunyiCan_to_db3(dst_folder, self.install_path)
                self.data_transformer.combine_Kunyi_db3(dst_folder)
                self.data_transformer.gen_AVM_from_db3(dst_folder, self.install_path)
                print(f"{dst_folder}处理完成, 删除原始文件")
                if os.path.exists(os.path.join(dst_folder, self.data_transformer.can_file_name)):
                    shutil.rmtree(os.path.join(dst_folder, self.data_transformer.can_file_name))
                if os.path.exists(os.path.join(dst_folder, self.data_transformer.images_file_name)):
                    shutil.rmtree(os.path.join(dst_folder, self.data_transformer.images_file_name))
            else:
                print(f"{dst_folder}不包含指定文件夹, 跳过")

            # 数据回灌
            t0 = time.time()
            # drt = DataReplayTest(self.test_project_path)
            # drt.start()

            print('====================', time.time() - t0, '======================')

if __name__ == '__main__':
    install_path = '/home/hp/artifacts/ZPD_AH4EM/3.3.0_RC1/install'
    info_path = '/home/hp/temp/77w数据汇总_tmp.xlsx'
    local_download_path = '/home/hp/temp'
    test_project_path = ''
    dpr = DownloadProcessReplay(install_path, info_path, local_download_path, test_project_path)
    dpr.run()