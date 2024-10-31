#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : DataServer.py
# @Time     : 24/10/21 7:23 PM
# @Author   : Bu Yujun
import json
import os

from pymongo import MongoClient, ASCENDING, errors




class DataServer:

    def __init__(self, db_name, collection_name, host='localhost', port=27017):
        client = MongoClient(host, port)
        self.db = client[db_name]
        self.collection = self.db[collection_name]
        # 创建唯一索引
        try:
            self.collection.create_index([('uuid', ASCENDING)], unique=True)
        except errors.OperationFailure as e:
            print(f"Error creating index: {e}")

    def drop_collection(self):
        self.collection.drop()

    def add_data(self, path):

        def insert_json_data(json_file):
            with open(json_file, 'r', encoding='utf-8') as file:
                data = json.load(file)
            try:
                self.collection.insert_one(data)
                return True
            except errors.BulkWriteError as e:
                print('BulkWriteError with uuid {:s}'.format(data['uuid']))
                return False
            except errors.DuplicateKeyError as e:
                print('DuplicateKeyError with uuid {:s}'.format(data['uuid']))
                return False

        insert_count = 0
        if os.path.isdir(path):
            for f in os.listdir(path):
                json_file = os.path.join(path, f)
                if '.json' in json_file:
                    if insert_json_data(json_file):
                        insert_count += 1
        else:
            if '.json' in path:
                if insert_json_data(path):
                    insert_count += 1

        print(f'上傳了{insert_count}條數據')

    def query(self, filter_condition):
        db_filter = {key: {'$in': value} for key, value in filter_condition.items()}
        results = self.collection.find(db_filter)
        # 处理结果
        for result in results:
            print(result)


if __name__ == '__main__':
    db_name = 'replay_database'
    collection_name = 'test_result'
    path = '/home/zhangliwei01/ZONE/TestProject/ES39/p_feature_20241022_123455/04_TestData/1-Obstacles/03_OutputResult/statistics'

    ds = DataServer(db_name, collection_name)
    # ds.drop_collection()
    # ds.add_data(path)
    filter_condition = {
        'TopicName': ['/VA/Obstacles'],
        # 'RoadTypeCondition': ['城区道路'],
        'MetricTypeName': ['纵向距离误差'],
        'FeatureDetail': ['全局目标'],
    }
    ds.query(filter_condition)