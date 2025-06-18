"""  
@Author: BU YUJUN
@Date: 2025/6/17 11:03  
"""

import boto3
from botocore.client import Config

# 配置S3客户端
s3_client = boto3.client(
    's3',
    endpoint_url='http://10.192.53.221:8080',  # 你的S3 endpoint
    aws_access_key_id='QB1YGVNUKJP2MRK8AK2R',  # 替换为你的Access Key
    aws_secret_access_key='JxRde3bPdoxWaBBFwmmqH81ytiNIoTILh9CGCYJH',  # 替换为你的Secret Key
    config=Config(signature_version='s3v4'),
    region_name='us-east-1'  # 区域名称，可根据实际情况修改
)

# 指定存储桶和路径
bucket_name = 'prod-ac-dmp'
s3_path = 'backup/data/collect/self/driving/20250616_upload_Q3402/'  # S3路径（注意不要以斜杠开头）


# 1. 列出存储桶中的对象
def list_objects():
    try:
        response = s3_client.list_objects_v2(
            Bucket=bucket_name,
            Prefix=s3_path  # 过滤指定路径下的对象
        )
        total_size = 0
        if 'Contents' in response:
            # print(f"在路径 {s3_path} 下找到 {len(response['Contents'])} 个对象：")
            for obj in response['Contents']:
                print(f"- {obj['Key']} (大小 {round(obj['Size']/1024/1024/1024, 6)} GB)")
                total_size += obj['Size']
            print(f"在路径 {s3_path} 下找到 {len(response['Contents'])} 个对象, 大小 {round(total_size/1024/1024/1024, 6)} GB")
        else:
            print(f"路径 {s3_path} 下没有找到对象。")
    except Exception as e:
        print(f"列出对象时出错: {e}")


# 2. 上传文件到S3
def upload_file(local_file_path, s3_key):
    try:
        s3_client.upload_file(
            Filename=local_file_path,
            Bucket=bucket_name,
            Key=s3_key
        )
        print(f"文件 {local_file_path} 已成功上传到 {s3_key}")
    except Exception as e:
        print(f"上传文件时出错: {e}")


# 3. 从S3下载文件
def download_file(s3_key, local_file_path):
    try:
        s3_client.download_file(
            Bucket=bucket_name,
            Key=s3_key,
            Filename=local_file_path
        )
        print(f"文件 {s3_key} 已成功下载到 {local_file_path}")
    except Exception as e:
        print(f"下载文件时出错: {e}")


# 4. 删除S3对象
def delete_object(s3_key):
    try:
        s3_client.delete_object(
            Bucket=bucket_name,
            Key=s3_key
        )
        print(f"对象 {s3_key} 已成功删除")
    except Exception as e:
        print(f"删除对象时出错: {e}")


# 示例使用
if __name__ == "__main__":
    # 1. 列出对象
    list_objects()

    # 2. 上传示例（取消注释并替换为实际文件路径）
    # upload_file('path/to/local/file.txt', f'{s3_path}file.txt')

    # 3. 下载示例（取消注释并指定要下载的对象）
    # download_file(f'{s3_path}file.txt', 'path/to/save/file.txt')

    # 4. 删除示例（取消注释并指定要删除的对象）
    # delete_object(f'{s3_path}file.txt')
