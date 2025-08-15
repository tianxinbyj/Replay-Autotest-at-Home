"""
@Author: BU YUJUN
@Date: 2025/6/17 11:03
"""

import argparse
import json
import os

import boto3
from botocore.client import Config

from Envs.Master.Modules.Libs import get_project_path


class S3Client:

    def __init__(self, endpoint_url, aws_access_key_id, aws_secret_access_key):
        # 配置S3客户端
        self.s3_client = boto3.client(
            's3',
            endpoint_url=endpoint_url,  # 你的S3 endpoint
            aws_access_key_id=aws_access_key_id,  # 替换为你的Access Key
            aws_secret_access_key=aws_secret_access_key,  # 替换为你的Secret Key
            config=Config(signature_version='s3v4'),
            region_name='cn-north-1'  # 区域名称，可根据实际情况修改
        )

    def list_objects(self, bucket_name, s3_path, include=None, exclude=None):
        if include is None:
            include = []
        if exclude is None:
            exclude = []
        path_dict = {}

        try:
            total_size = 0
            total_objects = 0
            filtered_objects = 0
            continuation_token = None

            # 循环处理分页，直到获取所有对象
            while True:
                # 准备请求参数
                params = {
                    'Bucket': bucket_name,
                    'Prefix': s3_path
                }
                # 如果有续传令牌，添加到请求参数中
                if continuation_token:
                    params['ContinuationToken'] = continuation_token

                # 调用API获取对象列表
                response = self.s3_client.list_objects_v2(**params)

                # 处理当前页的对象
                if 'Contents' in response:
                    page_objects = len(response['Contents'])
                    total_objects += page_objects

                    for obj in response['Contents']:
                        # 应用包含过滤条件
                        if len(include) and (not any([i in obj['Key'] for i in include])):
                            continue

                        # 应用排除过滤条件
                        if len(exclude) and (any([e in obj['Key'] for e in exclude])):
                            continue

                        # 显示符合条件的对象
                        print(f"- {obj['Key']} (大小 {round(obj['Size'] / 1024 / 1024, 4)} MB)")
                        total_size += obj['Size']
                        filtered_objects += 1
                        path_dict[obj['Key']] = obj['Size'] / 1024 / 1024

                # 检查是否还有更多对象
                if not response.get('IsTruncated', False):
                    break

                # 更新续传令牌，用于获取下一页
                continuation_token = response.get('NextContinuationToken')

            # 输出汇总信息
            print(f"在路径 {s3_path} 下共找到 {total_objects} 个对象，"
                  f"符合条件的有 {filtered_objects} 个，"
                  f"总大小 {round(total_size / 1024 / 1024 / 1024, 4)} GB")

            AEB_data_path = os.path.join(get_project_path(), 'Temp', 'AEB_data_path.json')
            with open(AEB_data_path, 'w', encoding='utf-8') as f:
                json.dump(path_dict, f, ensure_ascii=False, indent=4)
            return AEB_data_path

        except Exception as e:
            print(f"列出对象时出错: {e}")

    def download_s3_folder(self, bucket_name, s3_path, local_dir, include=None, exclude=None):
        if include is None:
            include = []
        if exclude is None:
            exclude = []

        os.makedirs(local_dir, exist_ok=True)

        """递归下载S3路径下的所有文件"""
        total_downloaded = 0
        total_size = 0

        # 使用分页器处理超过1000个对象的情况
        paginator = self.s3_client.get_paginator('list_objects_v2')
        for page in paginator.paginate(Bucket=bucket_name, Prefix=s3_path):
            if 'Contents' in page:
                for obj in page['Contents']:
                    s3_key = obj['Key']
                    # 跳过目录（S3中没有真正的目录，只有以/结尾的键）
                    if s3_key.endswith('/'):
                        continue

                    # 如果需要包含字符，则至少包含其中一个
                    if len(include) and (not any([i in obj['Key'] for i in include])):
                        continue

                    # 如果需要排除字符，则包含一个字符则排除
                    if len(exclude) and (any([e in obj['Key'] for e in exclude])):
                        continue

                    # 构建本地文件路径
                    local_file = os.path.join(local_dir, os.path.relpath(s3_key, s3_path))
                    local_path = os.path.dirname(local_file)

                    # 创建本地目录结构
                    os.makedirs(local_path, exist_ok=True)

                    # 下载文件
                    print(f"下载: {s3_key} -> {local_file}, 大小 {round(obj['Size'] / 1024 / 1024, 4)} MB")
                    self.s3_client.download_file(bucket_name, s3_key, local_file)

                    total_downloaded += 1
                    total_size += obj['Size']

        # 输出更详细的下载统计信息
        print(f"下载完成，共下载 {total_downloaded} 个文件，"
              f"总大小 {round(total_size / 1024 / 1024, 2)} MB，"
              f"文件保存在: {os.path.abspath(local_dir)}")

    def upload_directory(self, bucket_name, local_dir, s3_path):
        if not os.path.isdir(local_dir):
            print(f"错误：{local_dir} 不是一个有效的目录")
            return False

        total_uploaded = 0
        total_size = 0

        # 遍历本地目录
        for root, dirs, files in os.walk(local_dir):
            for file in files:
                local_file_path = os.path.join(root, file)

                # 获取相对路径，用于保持目录结构
                relative_path = os.path.relpath(local_file_path, local_dir)
                s3_key = os.path.join(s3_path, relative_path).replace(os.sep, '/')

                # 上传文件
                file_size = os.path.getsize(local_file_path)
                try:
                    print(f"上传: {local_file_path} -> {s3_key} "
                          f"(大小: {round(file_size / 1024 / 1024, 2)} MB)")

                    self.s3_client.upload_file(local_file_path, bucket_name, s3_key)
                    total_uploaded += 1
                    total_size += file_size
                except Exception as e:
                    print(f"上传失败 {local_file_path}: {e}")

        # 输出上传统计信息
        print(f"上传完成，共上传 {total_uploaded} 个文件，"
              f"总大小 {round(total_size / 1024 / 1024, 2)} MB")
        return True


def main():
    parser = argparse.ArgumentParser(description="match obstacles")
    parser.add_argument("-u", "--endpoint_url", type=str, required=True, help="endpoint url")
    parser.add_argument("-k", "--aws_access_key_id", type=str, required=True, help="access key id")
    parser.add_argument("-s", "--aws_secret_access_key", type=str, required=True, help="secret access key")
    parser.add_argument("-n", "--bucket_name", type=str, required=True, help="bucket name")
    parser.add_argument("-p", "--s3_path", type=str, required=True, help="s3 path")
    parser.add_argument("-f", "--local_dir", type=str, required=False, default='n/a', help="local dir")
    parser.add_argument("-i", "--include", type=str, nargs='*', default=None, required=False, help="必须包含字符串的文件")
    parser.add_argument("-x", "--exclude", type=str, nargs='*', default=None, required=False, help="排除包含字符串的文件")
    args = parser.parse_args()

    endpoint_url = args.endpoint_url
    aws_access_key_id = args.aws_access_key_id
    aws_secret_access_key = args.aws_secret_access_key
    bucket_name = args.bucket_name
    s3_path = args.s3_path
    local_dir = args.local_dir
    include = args.include
    exclude = args.exclude

    if s3_path.startswith('/'):
        s3_path = s3_path[1:]

    s3_client = S3Client(endpoint_url, aws_access_key_id, aws_secret_access_key)
    if local_dir != 'n/a':
        s3_client.download_s3_folder(bucket_name, s3_path, local_dir, include, exclude)
    else:
        s3_client.list_objects(bucket_name, s3_path, include, exclude)


if __name__ == '__main__':
    # main()
    cmd = '''
    /usr/bin/python3 Api_DownloadS3.py -u http://10.192.53.221:8080 -k QB1YGVNUKJP2MRK8AK2R -s JxRde3bPdoxWaBBFwmmqH81ytiNIoTILh9CGCYJH -n prod-ac-dmp -p backup/data/collect/self/driving/20250530-20250529-car2-bev-Lidar/3D_data_LSJWK4095NS119733 -i n000001 -x .pcap
    '''
    endpoint_url='http://10.192.53.221:8080'  # 你的S3 endpoint
    aws_access_key_id='44JAMVA71J5L90D9DK77'  # 替换为你的Access Key
    aws_secret_access_key='h1cY4WzpNxmQCpsXlXFpO4nWjNp3pbH0ZuBsuGmu'  # 替换为你的Secret Key
    # bucket_name = 'prod-ac-dmp'
    bucket_name = 'aeb'
    s3_path = 'ALL/'
    include = ['AH4EM-SIMU182/202507/20250713']
    exclude = ['canlog']
    local_dir = '/media/data/Q_DATA/AebRawData'
    s3_client = S3Client(endpoint_url, aws_access_key_id, aws_secret_access_key)
    # s3_client.list_objects(bucket_name, s3_path, include=include, exclude=exclude)
    s3_client.download_s3_folder(bucket_name, s3_path, local_dir, include, exclude)