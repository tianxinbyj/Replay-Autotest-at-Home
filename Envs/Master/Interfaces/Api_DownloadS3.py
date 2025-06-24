"""
@Author: BU YUJUN
@Date: 2025/6/17 11:03
"""

import argparse
import os
import shutil
import boto3
from botocore.client import Config


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

    def list_objects(self, bucket_name, s3_path):
        try:
            response = self.s3_client.list_objects_v2(
                Bucket=bucket_name,
                Prefix=s3_path  # 过滤指定路径下的对象
            )
            total_size = 0
            if 'Contents' in response:
                # print(f"在路径 {s3_path} 下找到 {len(response['Contents'])} 个对象：")
                for obj in response['Contents']:
                    print(f"- {obj['Key']} (大小 {round(obj['Size'] / 1024 / 1024 / 1024, 4)} GB)")
                    total_size += obj['Size']
                print(
                    f"在路径 {s3_path} 下找到 {len(response['Contents'])} 个对象, 大小 {round(total_size / 1024 / 1024 / 1024, 4)} GB")
            else:
                print(f"路径 {s3_path} 下没有找到对象。")
        except Exception as e:
            print(f"列出对象时出错: {e}")

    def download_s3_folder(self, bucket_name, s3_path, local_dir):

        if not os.path.exists(local_dir):
            shutil.rmtree(local_dir)
        # 创建本地目录
        os.makedirs(local_dir, exist_ok=True)

        """递归下载S3路径下的所有文件"""
        # 列出所有对象
        paginator = self.s3_client.get_paginator('list_objects_v2')
        for page in paginator.paginate(Bucket=bucket_name, Prefix=s3_path):
            if 'Contents' in page:
                for obj in page['Contents']:
                    s3_key = obj['Key']
                    # 跳过目录（S3中没有真正的目录，只有以/结尾的键）
                    if s3_key.endswith('/'):
                        continue

                    # 构建本地文件路径
                    local_file = os.path.join(local_dir, os.path.relpath(s3_key, s3_path))
                    local_path = os.path.dirname(local_file)

                    # 创建本地目录结构
                    os.makedirs(local_path, exist_ok=True)

                    # 下载文件
                    print(f"下载: {s3_key} -> {local_file}")
                    self.s3_client.download_file(bucket_name, s3_key, local_file)

        print(f"下载完成，文件保存在: {os.path.abspath(local_dir)}")


def main():
    parser = argparse.ArgumentParser(description="match obstacles")
    parser.add_argument("-u", "--endpoint_url", type=str, required=True, help="endpoint url")
    parser.add_argument("-k", "--aws_access_key_id", type=str, required=True, help="access key id")
    parser.add_argument("-s", "--aws_secret_access_key", type=str, required=True, help="secret access key")
    parser.add_argument("-n", "--bucket_name", type=str, required=True, help="bucket name")
    parser.add_argument("-p", "--s3_path", type=str, required=True, help="s3 path")
    parser.add_argument("-f", "--local_dir", type=str, required=False, default='n/a', help="local dir")
    args = parser.parse_args()

    endpoint_url = args.endpoint_url
    aws_access_key_id = args.aws_access_key_id
    aws_secret_access_key = args.aws_secret_access_key
    bucket_name = args.bucket_name
    s3_path = args.s3_path
    local_dir = args.local_dir

    if s3_path.startswith('/'):
        s3_path = s3_path[1:]

    s3_client = S3Client(endpoint_url, aws_access_key_id, aws_secret_access_key)
    if local_dir != 'n/a':
        s3_client.download_s3_folder(bucket_name, s3_path, local_dir)
    else:
        s3_client.list_objects(bucket_name, s3_path)


if __name__ == '__main__':
    main()
    cmd = '''
    /usr/bin/python3 Api_DownloadS3.py -u http://10.192.53.221:8080 -k QB1YGVNUKJP2MRK8AK2R -s JxRde3bPdoxWaBBFwmmqH81ytiNIoTILh9CGCYJH -n prod-ac-dmp -p backup/data/collect/self/driving/20250616_upload_Q3402/
    '''
    cmd = '''
    /usr/bin/python3 Api_DownloadS3.py -u http://10.192.53.221:8080 -k QB1YGVNUKJP2MRK8AK2R -s JxRde3bPdoxWaBBFwmmqH81ytiNIoTILh9CGCYJH -n prod-ac-dmp -p backup/data/collect/self/driving/20250530-20250529-car2-bev-Lidar/3D_data_LSJWK4095NS119733
    '''
    endpoint_url='http://10.192.53.221:8080',  # 你的S3 endpoint
    aws_access_key_id='QB1YGVNUKJP2MRK8AK2R',  # 替换为你的Access Key
    aws_secret_access_key='JxRde3bPdoxWaBBFwmmqH81ytiNIoTILh9CGCYJH',  # 替换为你的Secret Key
    bucket_name = 'prod-ac-dmp'
    s3_path = 'backup/data/collect/self/driving/20250616_upload_Q3402/'
    local_dir = '/media/data/Q_DATA/debug_data'