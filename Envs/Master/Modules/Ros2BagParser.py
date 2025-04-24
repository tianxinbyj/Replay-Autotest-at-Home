#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : EQ4LaneTest.py
# @Time         : 3/22/23 9:29 AM
# @Author      : Bu Yujun

import csv
import glob
import multiprocessing as mp
import os
import time
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg, register_types

warnings.filterwarnings("ignore")

msg_description = {
    'proto_horizon_msgs/msg/Obstacles':
        {
            'ObstacleType':
                {
                    0: 'Vehicle',  # Veh_R
                    1: 'Vehicle',
                    2: 'Pedestrian',
                    18: 'Cyclist',
                },
            'ObstacleLane':
                {
                    -1: 'Rear',
                    0: 'Unknown',
                    1: 'LL',
                    2: 'Left',
                    3: 'Host',
                    4: 'Right',
                    5: 'RR',
                },
            'MotionState':
                {
                    0: 'Invalid',
                    1: 'Unknown',
                    2: 'Moving',
                    3: 'Stationary',
                    4: 'Stopped',
                    5: 'Slowly',
                },
            'MotionCategory':
                {
                    0: 'Invalid',
                    1: 'Undefined',
                    2: 'Passing',
                    3: 'Passing_In',
                    4: 'Passing_Out',
                    5: 'Cut_In',
                    6: 'Moving_In',
                    7: 'Moving_Out',
                    8: 'Crossing',
                    9: 'Left_Turn',
                    10: 'Right_Turn',
                    11: 'Moving',
                    12: 'Preceding',
                    13: 'Oncoming',
                },
            'MeasurementStatus':
                {
                    0: 'Invalid',
                    1: 'Measured',
                    2: 'Predicted',
                },
            'PropertyCategory':
                {
                    0: {
                        0: {  # 车辆分类
                            0: 'Bus',
                            1: 'Small_Medium_Car',
                            2: 'Truck',
                            3: 'Tricycle',
                            4: 'Special',
                            5: 'Tiny_Car',
                            6: 'Lorry',
                        },
                        2: {  # 车灯分类
                            0: 'LeftLightOn',
                            1: 'RightLightOn',
                            2: 'BrakeLightOn',
                            3: 'AllLightOff',
                            4: 'Left+BrakeLightOn',
                            5: 'Right+BrakeLightOn',
                            6: 'ClearanceLampOn',
                        },
                        9: {  # 车辆遮挡程度
                            0: 'Full_Visible',
                            1: 'Occluded',
                            2: 'Heavily_Occluded',
                            3: 'Invisible',
                            4: 'Unknown',
                        },
                    },
                    1: {
                        0: {  # 行人朝向
                            0: 'Back',
                            1: 'Front',
                            2: 'Left',
                            3: 'Left_Anterior',
                            4: 'Left_Back',
                            5: 'Right',
                            6: 'Right_Back',
                            7: 'Right_Front',
                        },
                        2: {  # 行人年龄
                            0: 'Adult',
                            1: 'Child',
                        },
                        3: {  # 行人姿态
                            0: 'Bended',
                            1: 'Cyclist',
                            2: 'Lier',
                            3: 'Pedestrian',
                            4: 'Sitter',
                        },
                        7: {  # 行人遮挡程度
                            0: 'Full_visible',
                            1: 'Occluded',
                            2: 'Heavily_occluded',
                            3: 'Invisible',
                        },
                    },
                },
        },

    'proto_horizon_msgs/msg/Lines':
        {
            'LineColor':
                {
                    0: (42, 42, 165),  # red
                    2: (190, 190, 190),  # white
                    4: (0, 238, 238),  # yellow
                    8: (0, 165, 255),  # orange
                    16: (255, 144, 30),  # blue
                    32: (69, 139, 0),  # green
                    64: (120, 120, 120),  # grey
                    128: (107, 183, 189),  # grey yellow
                    256: (205, 250, 255),  # yellow white
                },
            'LinePosition':
                {
                    -1: 'Unknown',
                    0: 'Left',
                    2: 'Right',
                    4: 'LL',
                    8: 'RR',
                    16: 'LO',
                    32: 'RO',
                    64: 'LLL',
                    128: 'RRR',
                },
            'LineType':
                {
                    -1: '-1',
                    0: 'Unknown',
                    2: 'Lane',
                    8: 'Center',
                    64: 'Fence',
                },
            'LineMarking':
                {
                    0: 'Unknown',
                    2: 'Solid',
                    4: 'Dashed',
                    8: 'S-Dashed',
                    16: '2-Solid',
                    32: '2-Dashed',
                    64: 'L-S,R-D',
                    128: 'R-S,L-D',
                    256: 'Shaded',
                    16384: 'De-Solid',
                    32784: 'De-Dashed',
                },
            'LineSource':
                {
                    1: 'Crossing',
                    4: 'Blocked',
                    16: 'History',
                    32: 'Extrapolate',
                    512: 'Unknown',
                    1024: 'New',
                    2048: 'Measured',
                    4096: 'Predicted',
                    8192: 'Left',
                    16384: 'Right',
                    32768: 'Curb',
                    65536: 'Guardrail',
                    131072: 'Barrier',
                    262144: 'Wall',
                    524288: 'Canopy',
                    2097152: 'Cone',
                    4194304: 'Other',
                    33554432: 'Stopline',
                    67108864: 'Separate',
                },
        },

    'proto_horizon_msgs/msg/FusLines':
        {
            'LineColor':
                {
                    0: (42, 42, 165),  # red
                    2: (190, 190, 190),  # white
                    4: (0, 238, 238),  # yellow
                    8: (0, 165, 255),  # orange
                    16: (255, 144, 30),  # blue
                    32: (69, 139, 0),  # green
                    64: (120, 120, 120),  # grey
                    128: (107, 183, 189),  # grey yellow
                    256: (205, 250, 255),  # yellow white
                },
            'LinePosition':
                {
                    -1: 'Unknown',
                    0: 'Left',
                    2: 'Right',
                    4: 'LL',
                    8: 'RR',
                    16: 'LO',
                    32: 'RO',
                    64: 'LLL',
                    128: 'RRR',
                },
            'LineType':
                {
                    -1: '-1',
                    0: 'Unknown',
                    2: 'Lane',
                    8: 'Center',
                    64: 'Fence',
                },
            'LineMarking':
                {
                    0: 'Unknown',
                    2: 'Solid',
                    4: 'Dashed',
                    8: 'S-Dashed',
                    16: '2-Solid',
                    32: '2-Dashed',
                    64: 'L-S,R-D',
                    128: 'R-S,L-D',
                    256: 'Shaded',
                    16384: 'De-Solid',
                    32784: 'De-Dashed',
                },
            'LineSource':
                {
                    1: 'Crossing',
                    4: 'Blocked',
                    16: 'History',
                    32: 'Extrapolate',
                    512: 'Unknown',
                    1024: 'New',
                    2048: 'Measured',
                    4096: 'Predicted',
                    8192: 'Left',
                    16384: 'Right',
                    32768: 'Curb',
                    65536: 'Guardrail',
                    131072: 'Barrier',
                    262144: 'Wall',
                    524288: 'Canopy',
                    2097152: 'Cone',
                    4194304: 'Other',
                    33554432: 'Stopline',
                    67108864: 'Separate',
                },
        },

    'proto_horizon_msgs/msg/Objects':
        {
            'ObjectType':
                {
                    0: 'Unknown',
                    1: 'TrafficSign',
                    2: 'TrafficLight',
                    3: 'TrafficLightBulb',
                    4: 'LaneMarking',
                    5: 'StopLine',
                    6: 'SpeedBump',
                    7: 'Pole',
                    8: 'CrosswalkLine',
                    9: 'Zone',
                    10: 'ParkingSlot',
                    11: 'TrafficCone',
                    12: 'ParkingLockClose',
                    13: 'ParkingLockOpen',
                    14: 'ParkingColumn',
                    15: 'ParkingFrameSign',
                    16: 'Junction',
                    17: 'ParkingLock',
                },
            'ObjectSubType':
                {
                    1:  # TrafficSignTypeCN
                        {
                            0: 'Unknown',  # 未知
                            109: 'IR_Ramp',  # 匝道
                            110: 'IR_RampArrow',  # 匝道（箭头）
                            134: 'IR_Zones',  # 施工区标志
                            135: 'IR_ZonesII',  # 施工区标志Ⅱ
                            136: 'I_AllowUturn',  # 允许掉头
                            137: 'I_Circle',  # 环岛行驶
                            138: 'I_FastBusLane',  # 快速公交系统专用道
                            139: 'I_Forward',  # 直行
                            140: 'I_ForwardLane',  # 直行车道
                            141: 'I_ForwardLeft',  # 直行和左转
                            142: 'I_ForwardLeftLane',  # 直⾏和左转合⽤⻋道
                            144: 'I_ForwardRight',  # 直行和右转
                            145: 'I_ForwardRightLane',  # 直⾏和右转合⽤⻋道
                            147: 'I_Honk',  # 鸣喇叭
                            148: 'I_Left',  # 靠左侧行驶
                            149: 'I_LeftBusLane',  # 公交线路专⽤⻋道
                            150: 'I_LeftLine',  # 单⾏路向左
                            151: 'I_LeftRight',  # 左转和右转
                            152: 'I_LeftTurn',  # 向左转弯
                            153: 'I_LeftTurnLane',  # 左转⻋道
                            154: 'I_MinSpeedLim',  # 最低时速限制
                            162: 'I_Motors',  # 机动⻋⾏驶
                            163: 'I_MotorsLane',  # 机动⻋道
                            164: 'I_MultiOccupantMotorsLane',  # 多乘员⻋辆专⽤⻋道
                            165: 'I_NonMotors',  # ⾮机动⻋⾏驶
                            166: 'I_NonMotorsLane',  # ⾮机动⻋⻋道
                            169: 'I_ParkingSpace',  # 停⻋位
                            171: 'I_PedestrianCross',  # ⼈⾏横道
                            172: 'I_PriorityIntereesction',  # 路⼝优先通⾏
                            173: 'I_Right',  # 靠右侧⾏驶
                            174: 'I_RightLine',  # 单⾏路向右
                            175: 'I_RightTurn',  # 向右转弯
                            176: 'I_RightTurnLane',  # 右转⻋道
                            177: 'I_SplitDriveLine',  # 分向⾏驶⻋道
                            178: 'I_StraightLine',  # 单⾏路直⾏
                            179: 'I_UTurnAndLeftLane',  # 掉头和左转合⽤⻋道
                            180: 'I_UTurnLane',  # 掉头⻋道
                            183: 'I_Uturn',  # 掉头
                            184: 'I_Walk',  # 步⾏
                            218: 'P_Custom',  # 海关
                            219: 'P_GiveWay',  # 会⻋让⾏
                            220: 'P_HeightLim',  # 限⾼
                            221: 'P_MultiVehicle',  # 多交通⼯具禁⽌
                            222: 'P_NoAnimalVehicle',  # 禁⽌畜⼒⻋进⼊
                            223: 'P_NoBikeDownSlope',  # 禁⽌骑⾃⾏⻋下坡
                            224: 'P_NoBikeUpSlope',  # 禁⽌骑⾃⾏⻋上坡
                            225: 'P_NoBus',  # 禁⽌客⻋驶⼊
                            226: 'P_NoCar',  # 禁⽌⼩汽⻋驶⼊
                            227: 'P_NoCargoTricycle',  # 禁⽌三轮货⻋驶⼊
                            228: 'P_NoDangerous',  # 禁⽌危险品⻋辆驶⼊
                            229: 'P_NoEntry',  # 禁⽌驶⼊
                            230: 'P_NoForward',  # 禁⽌直⾏
                            231: 'P_NoForwardLeft',  # 禁⽌直⾏和左转
                            232: 'P_NoForwardRight',  # 禁⽌直⾏和右转
                            233: 'P_NoHorning',  # 禁鸣喇叭
                            234: 'P_NoHuman',  # 禁⽌⾏⼈进⼊
                            235: 'P_NoHumanCargoTriangle',  # 禁⽌⼈⼒载货三轮⻋进⼊
                            236: 'P_NoHumanPassengerTriangle',  # 禁⽌⼈⼒三轮⻋进⼊
                            237: 'P_NoHumanVehicle',  # 禁⽌⼈⼒⻋进⼊
                            238: 'P_NoLeftRightTurn',  # 禁⽌左右转弯
                            239: 'P_NoLeftTurn',  # 禁⽌左转弯
                            240: 'P_NoLongParking',  # 禁⽌⻓时停⻋
                            242: 'P_NoMinibus',  # 禁⽌⼩型客⻋通⾏
                            244: 'P_NoMotor',  # 禁⽌机动⻋驶⼊
                            245: 'P_NoMotorcycle',  # 禁⽌摩托⻋驶⼊
                            246: 'P_NoNonMotor',  # 禁⽌⾮机动⻋进⼊
                            247: 'P_NoParking',  # 禁⽌停⻋
                            249: 'P_NoPassing',  # 禁⽌超⻋
                            250: 'P_NoPassingRev',  # 解除禁⽌超⻋
                            251: 'P_NoReturn',  # 禁⽌掉头
                            252: 'P_NoRightTurn',  # 禁⽌右转弯
                            253: 'P_NoSpecificLeftTurn',  # 禁⽌特定⻋辆左转
                            254: 'P_NoSpecificRightTurn',  # 禁⽌特定⻋辆右转
                            255: 'P_NoTractor',  # 禁⽌拖拉机驶⼊
                            256: 'P_NoTrailer',  # 禁⽌挂⻋驶⼊
                            257: 'P_NoTricycle',  # 禁⽌电动三轮⻋驶⼊
                            258: 'P_NoTruck',  # 禁⽌载货汽⻋驶⼊
                            259: 'P_Noway',  # 禁⽌通⾏
                            261: 'P_Other_SpeedLimele',  # 电⼦限速其他
                            262: 'P_ParkingCheck',  # 停⻋检查
                            263: 'P_SlowFor',  # 减速让⾏
                            264: 'P_SpeedLim',  # 限速
                            265: 'P_SpeedLimele',  # 电子限速
                            300: 'P_SpeedLimRev',  # 解除限速
                            324: 'P_StopFor',  # 停⻋让⾏
                            327: 'P_VehicleOther',  # 其他单交通⼯具禁⽌
                            328: 'P_WeightLim',  # 限重
                            329: 'P_WeightLimWheel',  # 轮胎限重
                            330: 'P_WidthLim',  # 限宽
                            376: 'W_AccidentProne',  # 注意事故易发路段
                            378: 'W_BadWeather',  # 注意恶劣天⽓
                            379: 'W_Bump',  # 注意路⾯凸起
                            380: 'W_Bumpy',  # 注意颠簸路⾯
                            381: 'W_Children',  # 注意⼉童
                            382: 'W_Circle',  # 注意环⾏路⼝
                            383: 'W_ContinuousDown',  # 注意连续下坡
                            384: 'W_ContinuousTurn',  # 注意连续弯道
                            385: 'W_Cross',  # 注意⼗字路⼝
                            386: 'W_CrossIntersection',  # 注意⼗字平⾯交叉
                            388: 'W_Cycle',  # 注意⾮机动⻋
                            389: 'W_DamLeft',  # 注意堤坝左侧⽔域
                            390: 'W_DamRight',  # 注意堤坝右侧⽔域
                            391: 'W_Danger',  # 注意危险
                            392: 'W_DetourAround',  # 注意左右侧绕⾏
                            393: 'W_DetourLeft',  # 注意左侧绕⾏
                            394: 'W_DetourRight',  # 注意右侧绕⾏
                            395: 'W_Disabled',  # 注意残疾⼈
                            397: 'W_Down',  # 注意下坡
                            399: 'W_Ferry',  # 注意渡⼝
                            400: 'W_Fog',  # 注意雾天
                            401: 'W_Ford',  # 注意过⽔路⾯
                            402: 'W_GuardedRailway',  # 注意铁道⼝
                            403: 'W_HumpBridge',  # 注意驼峰桥
                            404: 'W_Ice',  # 注意路⾯结冰
                            405: 'W_KeepDistance',  # 注意保持⻋距
                            406: 'W_LRNarrow',  # 注意两侧变窄
                            407: 'W_LRTurn',  # 注意左边反向弯道
                            408: 'W_LeftFalling',  # 注意左⽅落⽯
                            409: 'W_LeftNarrow',  # 注意左侧变窄
                            410: 'W_LeftTurn',  # 注意左急转弯
                            411: 'W_LowLying',  # 注意路⾯低洼
                            412: 'W_MergeLeft',  # 注意左侧合流
                            413: 'W_MergeRight',  # 注意右侧合流
                            414: 'W_MountLeft',  # 注意左旁⼭险路
                            415: 'W_MountRight',  # 注意右旁⼭险路
                            416: 'W_NarrowBridge',  # 注意窄桥
                            418: 'W_Pedestrian',  # 注意⾏⼈
                            419: 'W_RLTurn',  # 注意右边反向弯道
                            420: 'W_Railway',  # 注意⽆⼈看守铁道⼝
                            421: 'W_Rain',  # 注意⾬雪
                            422: 'W_RightFalling',  # 注意右⽅落⽯
                            423: 'W_RightNarrow',  # 注意右侧变窄
                            424: 'W_RightTurn',  # 注意右急转弯
                            425: 'W_SideWind',  # 注意横⻛(警告)
                            428: 'W_Slip',  # 注意易滑路段
                            429: 'W_Slow',  # 注意减速慢⾏
                            430: 'W_SlowDown',  # 建议减速
                            431: 'W_SplitLeft',  # 注意左侧分流
                            432: 'W_SplitRight',  # 注意右侧分流
                            433: 'W_TIntersection',  # 注意T字平⾯交叉
                            435: 'W_TShapLeft',  # 注意左向丁字路⼝
                            436: 'W_TShapRight',  # 注意右向丁字路⼝
                            437: 'W_Tidal',  # 注意潮汐⻋道
                            438: 'W_TrafficLight',  # 注意信号灯
                            439: 'W_Tripod',  # 注意故障⻋辆
                            440: 'W_Tshape',  # 注意丁字路⼝
                            441: 'W_Tshaps',  # 注意连续丁字路⼝
                            442: 'W_Tunnel',  # 注意隧道
                            443: 'W_TunnelHeadlight',  # 注意隧道开⻋灯
                            444: 'W_TwoWay',  # 注意双向交通
                            446: 'W_Up',  # 注意上坡
                            447: 'W_VehicleQueue',  # 注意⻋辆排队
                            448: 'W_Village',  # 注意村庄
                            449: 'W_Working',  # 注意施⼯
                            450: 'W_YBLeft',  # 注意左后⽅叉⼝
                            451: 'W_YBRight',  # 注意右后⽅叉⼝
                            454: 'W_YLeft',  # 注意左前⽅叉⼝
                            455: 'W_YRight',  # 注意右前⽅叉⼝
                            1501: 'RouteIDSign',  # 公路编号
                            1502: 'MiscSign',  # 嵌套牌
                            1503: 'GuideSign',  # 指示牌
                            1504: 'I_Other',  # 其它
                        },
                    2:  # TrafficLightType
                        {
                            0: 'Unknown',
                            1: 'Circle',
                            2: 'Cross',
                            3: 'Pedestrian',
                            4: 'Bicycle',
                            5: 'Arrow',
                            6: 'Time',
                            7: 'Text',
                            8: 'MultiLen',
                        },
                    4:  # LaneMarkingType
                        {
                            0: 'Unknown',
                            1: 'ArrowLeft',
                            2: 'ArrowForward',
                            3: 'ArrowRight',
                            4: 'ArrowL&F',
                            5: 'ArrowR&F',
                            6: 'ArrowL&R',
                            7: 'ArrowTurn',
                            8: 'ArrowT&F',
                            9: 'ArrowT&L',
                            10: 'ArrowMergeL',
                            11: 'ArrowMergeR',
                            12: 'CrossNotice',
                            13: 'SpeedLimitL',
                            14: 'SpeedLimitH',
                            15: 'ArrowNoL',
                            16: 'ArrowNoR',
                            17: 'ArrowNoT',
                            18: 'ArrowNoL&R',
                            19: 'ArrowNoT&L',
                            20: 'ArrowNoT&R',
                            21: 'Text',
                            22: 'Time',
                            23: 'CheckDist',
                            24: 'StopGiveWay',
                            25: 'SlowGiveWay',
                            26: 'Stop',
                            27: 'Nets',
                            28: 'ArrowNoF',
                            29: 'ArrowNoMergeL',
                            30: 'ArrowNoMergeR',
                        },
                    7:  # PoleType
                        {
                            0: 'Unknown',
                            1: 'Gantry',
                            2: 'SignPost',
                            3: 'Signal',
                            4: 'RoadGate',
                            5: 'HeightLim',
                        },
                    9:  # ZoneType
                        {
                            0: 'Unknown',
                            1: 'Circle',
                            2: 'DeadCircle',
                            3: 'Polygon',
                            4: 'NoStop',
                            5: 'DriveWay',
                            6: 'ParkingLot',
                            7: 'RailCross',
                            8: 'InterSect',
                            9: 'ParkingArea',
                            10: 'Pickup',
                            11: 'Dropoff',
                            12: 'RoadWork',
                            13: 'CrossWalk',
                        },
                    11:  # TrafficConeType
                        {
                            0: 'Unknown',
                            1: 'TrafficCone',
                            2: 'TrafficBollard',
                            3: 'IsolationBollard',
                            4: 'OtherBollard',
                            5: 'CrashBarrel',
                        },
                },
            'TrafficLightBulbType':
                {
                    0: 'Unknown',
                    1: 'Circle',
                    2: 'Left',
                    4: 'Right',
                    8: 'Up',
                    16: 'Down',
                    32: 'UTurn',
                    64: 'F&L',
                    128: 'F&R',
                    256: 'Pedestrian',
                    512: 'NoMotor',
                    1024: 'Time',
                    2048: 'L&T',
                    4096: 'NoDrive',
                    8192: 'TextPed',
                    16384: 'SignPed',
                    32768: 'TextNoPed',
                    65536: 'SignNoPed',
                },
            'TrafficLightBulbColor':
                {
                    0: 'Unknown',
                    1: 'Off',
                    10: 'Green',
                    11: 'Yellow',
                    12: 'Red',
                },
            'TrafficLightStructType':
                {
                    0: 'Unknown',
                    1: 'HV1',
                    2: 'H2',
                    3: 'V2',
                    4: 'H3',
                    5: 'V3',
                    6: 'H4',
                    7: 'V4',
                    8: 'H5',
                    9: 'V5',
                },
            'Orientation':
                {
                    0: 'Unknown',
                    1: 'Back',
                    2: 'Side',
                    3: 'Front',
                },
            'TrafficLightBulbSpotMode':
                {
                    0: 'Unknown',
                    1: 'On',
                    2: 'Flash',
                    3: 'Off',
                },
            'ObjectShape':
                {
                    0: 'Unknown',
                    1: 'Rectangle',
                    2: 'Triangle',
                    3: 'Round',
                    4: 'Cylinder',
                },
        },

    'per_fusion_msgs/msg/LaneMarkings':
        {
            'LineColor':
                {
                    0: (42, 42, 165),  # red,should be unknown,
                    1: (180, 180, 180),  # white
                    2: (0, 185, 240),  # yellow
                    3: (80, 127, 255),  # orange
                    4: (255, 105, 65),  # blue
                    5: (0, 128, 0),  # green
                    6: (80, 80, 80),  # gray
                    7: (107, 183, 189),  # LEFT_GRAY_RIGHT_YELLOW = 7
                    8: (205, 250, 250),  # LEFT_YELLOW_RIGHT_WHITE = 8
                },
            'LinePosition':
                {
                    -1: 'Unknown',
                    0: 'UNSPECIFIED',
                    1: 'Left',
                    2: 'Right',
                    3: 'LL',
                    4: 'RR',
                    5: 'LO',
                    6: 'RO',
                    7: 'LLL',
                    8: 'RRR',
                },
            'LineType':
                {
                    0: 'Unknown',
                    1: 'Lane',
                    2: 'Center',
                    3: 'Fence',
                },
            'LineMarking':
                {
                    0: 'Unknown',
                    1: 'Solid',
                    2: 'Dashed',
                    3: 'S-Dashed',
                    4: '2-Solid',
                    5: '2-Dashed',
                    6: 'L-S,R-D',
                    7: 'R-S,L-D',
                },
            'LineSource':
                {
                    1: 'Crossing',
                    4: 'Blocked',
                    16: 'History',
                    32: 'Extrapolate',
                    512: 'Unknown',
                    1024: 'New',
                    2048: 'Measured',
                    4096: 'Predicted',
                    8192: 'Left',
                    16384: 'Right',
                    32768: 'Curb',
                    65536: 'Guardrail',
                    131072: 'Barrier',
                    262144: 'Wall',
                    524288: 'Canopy',
                    1048576: 'Cone',
                    2097152: 'Other',
                    33554432: 'Stopline',
                    67108864: 'Separate',
                },
            'ExtraPoints':
                {
                    0: 'Unspecified',
                    1: 'Unknown',
                    2: 'Division',
                    4: 'Merge',
                    8: 'Change_Start',
                    16: 'Change_End',
                    32: 'Uturn',
                },
        },

    'per_fusion_msgs/msg/ObjTracks':
        {
            'ObstacleType':
                {
                    0: 'Unknown',
                    1: 'Pedestrian',
                    2: 'Cyclist',
                    3: 'Passenger_Car',
                    4: 'Small_Bus',
                    5: 'Big_bus',
                    6: 'Light_Truck',
                    7: 'Heavy_Truck',
                    8: 'Tricycle',
                    9: 'Van',
                    10: 'Other_Vehicle',
                    11: 'Cone',
                    12: 'Pole',
                },
            'ObstacleLane':
                {
                    -1: 'Rear',
                    0: 'Unknown',
                    1: 'LL',
                    2: 'Left',
                    3: 'Host',
                    4: 'Right',
                    5: 'RR',
                },
            'MotionModel':
                {
                    0: 'Unknown',
                    1: 'Model_Ca',
                    2: 'Model_Ctra',
                },
            'MotionCategory':
                {
                    0: 'Unknown',
                    1: 'Stationary',
                    2: 'Driving',
                    3: 'Oncoming',
                    4: 'Driving_Stopped',
                    5: 'Oncoming_Stopped',
                    6: 'Crossing',
                },
            'MeasurementStatus':
                {
                    0: 'New',
                    1: 'Measured',
                    2: 'Predicted',
                },
            'FusionSource':
                {
                    0: 'Unknown',
                    1: 'Vision',
                    2: 'Radar',
                    3: 'V+R',
                },
            'BrakeLight':
                {
                    0: 'Unknown',
                    1: 'BrakeLightOn',
                    2: 'BrakeLightOff',
                },
            'TurnLight':
                {
                    0: 'Unknown',
                    1: 'LeftLightOn',
                    2: 'RightLightOn',
                    3: 'BothLightOn',
                    4: 'AllLightOff',
                },
            'DayNight':
                {
                    0: 'Unknown',
                    1: 'Day',
                    2: 'Night',
                },
        },

    'sensor_abstraction_msgs/msg/RadarObjectArray':
        {
            'ObjectType':
                {
                    0: 'Unknown',
                    3: 'Pedestrian',
                    5: 'Cyclist',
                    6: 'Motorbike',
                    7: 'Vehicle',
                    8: 'Van',
                    9: 'Truck',
                    15: 'F_init',
                },
        },

    'proto_horizon_msgs/msg/Slots':
        {
            'SlotType':
                {
                    0: 'unknown',
                    1: 'vertical',
                    2: 'parallel',
                    3: 'oblique',
                }
        },

    'proto_horizon_msgs/msg/WorkCondition':
        {
            'WorkCondition': {
                "Weather": 0,
                "Light": 1
            },
            'Weather': {
                "Sunny": 0,
                "Cloudy": 1,
                "Rainy": 2,
                "Snowy": 3,
                "HeavyRain": 4,
                "Other": 5,
                "Unknown": 255
            },
            'Light': {
                "NatureLight": 0,
                "LampLight": 1,
                "HardLight": 2,
                "LowSun": 3,
                "Dark": 4,
                "Other": 5,
                "Unknown": 255
            },
        },

    'proto_horizon_msgs/msg/Freespaces':
        {}
}

data_columns = {
    'vehicle_msgs/msg/VehicleMotionIpd':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'vehicle_speed', 'shaft_spd',
            'FL_wheel_speed', 'FR_wheel_speed', 'RL_wheel_speed', 'RR_wheel_speed',
            'front_wheel_angle', 'rear_wheel_angle',
        ],
    'gnss_imu_msgs/msg/Inspva':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'utc_time_stamp', 'latitude', 'longitude',
            'roll', 'pitch', 'yaw',
        ],
    'parking_ego_motion_msgs/msg/DrResult':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'ego_x', 'ego_y', 'ego_z',
            'ego_vx', 'ego_vy', 'ego_vz',
            'roll', 'pitch', 'yaw',
            'ego_ax', 'ego_ay', 'ego_az',
        ],
    'proto_horizon_msgs/msg/Obstacles':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'obstacle_type_text', 'confidence', 'sub_type', 'curr_lane',
            'x', 'y', 'z', 'vx', 'vy', 'vx_rel', 'vy_rel', 'yaw', 'length', 'width', 'height', 'age', 'coverage',
            'is_cipv', 'is_mcp', 'status',
        ],
    'env_perception_msgs/msg/EnvFusLines':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'position', 'marker', 'color',
            'extra_type', 'x_points', 'y_points',
        ],
    'proto_horizon_msgs/msg/Lines':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'position', 'marker', 'color',
            'start_x', 'start_y', 'c_x_0', 'c_x_1', 'c_x_2', 'c_x_3',
            'c_y_0', 'c_y_1', 'c_y_2', 'c_y_3', 'length', 'width', 'curve_type',
        ],
    'proto_horizon_msgs/msg/FusLines':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'position', 'marker', 'color',
            'start_x', 'start_y', 'c_x_0', 'c_x_1', 'c_x_2', 'c_x_3',
            'c_y_0', 'c_y_1', 'c_y_2', 'c_y_3', 'length', 'width', 'curve_type',
        ],
    'proto_horizon_msgs/msg/Objects':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'sub_type', 'curr_lane_flag',
            'x', 'y', 'z', 'x0', 'y0', 'x1', 'y1', 'x2', 'y2', 'x3', 'y3',
            'value', 'age', 'child_types',
        ],
    'sensor_abstraction_msgs/msg/RadarObjectArray':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'exist_confidence', 'obstacle_confidence',
            'x', 'y', 'vx', 'vy', 'length', 'width', 'yaw',
        ],
    'pilot_ego_motion_msgs/msg/EgoMotionInfo':
        [
            'local_time', 'time_stamp', 'frame_id',
            'ego_x', 'ego_y', 'ego_vx', 'ego_vy', 'steering_angle', 'front_angle',
        ],
    'pilot_perception_msg/msg/ObstaclesDet':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'obstacle_type_text', 'confidence', 'sub_type', 'curr_lane',
            'x', 'y', 'z', 'vx', 'vy', 'vx_rel', 'vy_rel', 'yaw', 'length', 'width', 'height', 'age', 'coverage',
            'is_cipv', 'is_mcp', 'status',
        ],
    'pilot_perception_msg/msg/Obstacles2dDet':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'obstacle_type_text', 'confidence', 'sub_type', 'curr_lane',
            'x', 'y', 'z', 'vx', 'vy', 'vx_rel', 'vy_rel', 'yaw', 'length', 'width', 'height', 'age', 'coverage',
            'is_cipv', 'is_mcp', 'status',
        ],
    'per_fusion_msgs/msg/ObjTracks':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'obstacle_type_text', 'confidence', 'sub_type', 'curr_lane',
            'x', 'y', 'z', 'vx', 'vy', 'vx_rel', 'vy_rel', 'yaw', 'length', 'width', 'height', 'age', 'coverage',
            'is_cipv', 'is_mcp', 'status',
        ],
    'pilot_perception_msg/msg/VisionLaneMarkList':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'position', 'marker', 'color',
            'start_x', 'start_y', 'c_x_0', 'c_x_1', 'c_x_2', 'c_x_3',
            'c_y_0', 'c_y_1', 'c_y_2', 'c_y_3', 'length', 'width', 'curve_type',
        ],
    'per_fusion_msgs/msg/LaneMarkings':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'position', 'marker', 'color',
            'start_x', 'start_y', 'c_x_0', 'c_x_1', 'c_x_2', 'c_x_3',
            'c_y_0', 'c_y_1', 'c_y_2', 'c_y_3', 'length', 'width', 'curve_type',
        ],
    'proto_horizon_msgs/msg/WorkCondition':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'work_condition_type', 'type',
            'type_conf'
        ],
    'proto_horizon_msgs/msg/Freespaces':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id', 'points_num', 'label', 'x', 'y', 'z',
            'valid'
        ],
    'proto_horizon_msgs/msg/Slots':
        [
            'local_time', 'time_stamp', 'header_seq', 'header_stamp', 'frame_id', 'id',
            'obj_type', 'confidence', 'obj_life_time', 'obj_age', 'type',
            'slot_pose_length', 'slot_pose_width',
            'slot_pose_angle', 'slot_heading', 'slot_center_x', 'slot_center_y',
            'pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y',
            'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y',
            'occupy_status', 'lock_status', 'lock_center_x', 'lock_center_y',
            'stopper_valid', 'stopper_0_x', 'stopper_0_y', 'stopper_1_x', 'stopper_1_y',
        ],
    'cus_app_msgs/msg/HmiES33':
        [
            'local_time', 'time_stamp', 'frame_id',
            'acc_status', 'acc_fault_status', 'acc_tja_sw_status', 'lks_status', 'zop_sw_cmd', 'pilot_sw_status',
            'distance', 'acc_cannot_acive_msg', 'pilot_exit_takeover_msg', 'pilot_req_eps_status',
            'pilot_req_eps_status_validity', 'auto_lane_chng_left_status', 'auto_lane_chng_right_status',
            'auto_lane_chng_msg_req', 'auto_lane_chng_style_set_response', 'cc_speed_voice_cmd',
            'cc_speed_voice_cmd_confirm_status', 'front_video_ctrl_monitor_fault_status', 'zop_system_status',
            'auto_lane_chng_sw_cmd', 'lane_change_recmed_status', 'lane_chng_rcmd_in_hmi_status', 'lane_chng_msg_req',
            'tja_ica_sys_sts', 'tja_ica_sys_fault_status', 'hod_status_detection', 'hod_status_detection_valid',
            'distance_level', 'slif_mode_resp', 'slif_status', 'ai_pilot_sys_flt_sts', 'lane_change_lamps_ctrl',
            'tts_req', 'zop_cannot_active_msg', 'zop_status', 'slif_limit_ahead_source', 'ilc_sw_cmd',
            'ilc_active_dir_cmd',
            'ilc_status', 'alc_status', 'acc_seld_trgt_dist_lvl_es37', 'spd_lmt_ofstset_resp', 'icp_status_es37',
            'ilc_status_es37', 'fct_brake_status', 'noa_quick_sw_msg', 'noa_quick_sw_cmd', 'noa_decrease_dist',
            'ilc_velocity_dsp', 'icp_sw_cmd', 'icp_status', 'icp_direction', 'icp_req_msg', 'tsr_status',
            'tsr_ban_sign', 'tsr_warn_sign', 'ldp_ldw_audio_req', 'ldp_ldw_display_cmd',
            'ldp_ldw_audio_warning_disp_cmd',
            'ldp_ldw_haptic_warning_disp_cmd', 'ldp_ldw_sensitivity_lvl', 'ldp_ldw_system_sts',
            'ldp_ldw_system_fault_sts', 'ldp_ldw_vibration_lvl_req', 'ldp_ldw_left_vis_req', 'ldp_ldw_right_vis_req',
            'camr_sas_req_sts',
            'spd_lmt_fuc_sts', 'lane_keep_assist_sys_sts', 'lcc_noa_switching_sw', 'mad_sys_sts', 'lads_progress',
            'map_construction_status', 'build_map_valid_size', 'navigation_info1', 'navigation_info2',
            'mad_navi_report_size', 'build_map_resp', 'route_cancel_resp', 'super_ecall', 'noa_lane_set_cmd',
            'noa_report_set_cmd', 'auto_main_beam_light_req', 'ldw_lka_hpt_wrm_dsp_cmd', 'tja_ica_msg_req',
            'elk_dsp_cmd_ipd', 'elk_sys_flt_sts', 'elk_sys_sts_ipd', 'lka_sys_flt_sts', 'elk_warning',
            'pilot_system_message', 'tlr_sys_sts', 'tlr_md_sw_dsp_cmd', 'tlr_warning', 'tlr_msg_req'
        ],
    'cus_app_msgs/msg/FctDebug':
        [
            'local_time', 'time_stamp', 'frame_id',
            'acc_debug', 'is_torq_resp_ok', 'is_acc_dec_resp_ok', 'is_torq_resp', 'is_acc_dec_resp',
            'is_no_whl_brk_prs_too_long', 'is_epb_ok', 'is_epb_applied', 'is_sensor_ok', 'is_acc_ok',
            'acc_passive_error_code', 'acc_active_error_code', 'is_all_doors_ok', 'is_controller_ok',
            'is_brake_applied', 'is_accel_actu_pos_ok', 'is_drive_off_cond_true', 'is_overtaking_control_valid',
            'is_overtaking_control_activated', 'is_overtaking_control_too_long', 'acc_passive_cond_group_id',
            'set_spd_chng_reason', 'target_state_of_active_trans', 'off_reason',
            'eps_degrade_b', 'eps_ready_b', 'ego_lane_l', 'ego_lane_r', 'ego_lane', 'hands_on', 'lat_ovrd', 'tja_on',
            'off_error_code', 'passive_error_code', 'degrade_error_code', 'curv_ul_hold_dist', 'curv_ul_strt_idx',
            'curv_ul_end_idx', 'ego_curvature_ahead', 'ego_curvature_idx', 'toll_idx', 'lane_merge_warning_flag',
            'dist_to_merge_thrs',
            'nav_on', 'hd_map_ok', 'toll_ok', 'zop_miss_merge_point_b', 'errcode',
            'ilc_status', 'tos_obs_l3_id', 'tos_obs_l2_id', 'tos_obs_r3_id', 'tos_obs_r2_id', 'tos_id_k1', 'ilc_left_b',
            'ilc_right_b', 'ilc_offset_hold_id', 'ilc_set_speed_kph', 'ilc_tos_fl_cnt', 'ilc_tos_fr_cnt',
            'ilc_tos_rl_cnt',
            'ilc_tos_rr_cnt', 'ilc_offset_m', 'ilc_quit_flg',
            'brake_during_cruising_count', 'hpp_suspend_status', 'hpp_abort_status', 'hpp_abnormal_to_standby_status',
            'hpp_to_apa_flag', 'turn_hpp_off_button', 'start_or_return_cruising_btn', 'target_slot_status',
            'apa_return_to_hpp_flag', 'brake_overtime_to_standby_during_cruising', 'hpp_reserve_place_one',
            'hpp_reserve_place_two', 'hpp_reserve_place_three', 'hpp_reserve_place_four', 'hpp_reserve_place_five',
            'hpp_reserve_place_six', 'hpp_reserve_place_seven',
            'replan_count', 'pi_button', 'po_button', 'slot_selected', 'start_parking_button', 'parking_cancel_button',
            'parking_suspend_button', 'parking_continue_button', 'is_car_moving', 'get_path_info_success',
            'parking_suspend_status', 'parking_abort_status', 'slot_id', 'slot_id_previous', 'current_path_step_num',
            'total_path_steps_num', 'apa_total_percent', 'suspend_ctn_shift_time_count', 'gear_shift_count',
            'gear_first_shift_to_r_flag', 'abnormal_pi_to_standby_status', 'abnormal_po_to_standby_status',
            'pk_out_direc_status', 'handshake_succeed', 'pk_out_direc_details',
            'is_spd_adjust_lc_done_over_10s', 'is_dir_lamp_switch_activated', 'is_in_same_ramp',
            'is_ego_crossed_laneline',
            'ldp_state', 'ldp_direction', 'ldp_departure_stage_l', 'ldp_departure_stage_r', 'ldp_front_left2border',
            'ldp_front_right2border', 'ldp_inhibit_flag',
            'icp_state', 'avg_spd_left', 'avg_spd_right', 'avg_spd_host', 'icp_off_error_code',
            'icp_standby_error_code', 'icp_leftlane_error_code', 'icp_rightlane_error_code', 'icp_passive_error_code',
            'tar_obj_idx', 'tar_obj_intention', 'tar_obj_heading',
            'ldw_state', 'ldw_direction', 'ldw_departure_stage_l', 'ldw_departure_stage_r', 'ldw_front_left2border',
            'ldw_front_right2border', 'ldw_inhibit_flag',
            'elk_state', 'elk_departure_stage_l', 'elk_departure_stage_r', 'front_left2border', 'front_right2border',
            'front_left2edge_l', 'front_right2edge_r', 'roadedge_thrsld', 'line_thrsld_left', 'line_thrsld_right',
            'elk_overtaking_idx', 'elk_overtaking_ttc', 'elk_basic_errcode', 'elk_driver_errcode',
            'tos_fl_cnt', 'tos_fr_cnt', 'tos_rl_cnt', 'tos_rr_cnt', 'lat_vel_ego', 'ego_lane_width',
            'is_front_side_narrow', 'is_rear_side_narrow', 'is_nra_close_btn_has_pressed', 'is_apa_hpp_active',
            'nra_abort_status', 'visual_assist_count', 'driver_close_visual_count', 'uss_dist_narrow_count',
            'ihc_state', 'ihc_off_error_code', 'ihc_standby_error_code', 'ihc_beam_off_error_code',
            'ihc_ego_curvature_ahead'
        ],
    'sensor_msgs/msg/CompressedImage':
        [
            'local_time', 'time_stamp', 'frame_id', 'format_',
        ],
    'parking_perception_msgs/msg/VisionSlotDecodingList':
        [
            'local_time', 'time_stamp', 'header_seq', 'header_stamp', 'frame_id', 'id',
            'obj_type', 'confidence', 'type', 'slot_center_x', 'slot_center_y',
            'pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y',
            'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y',
            'occupy_status', 'lock_status',
            'stopper_valid', 'stopper_0_x', 'stopper_0_y', 'stopper_1_x', 'stopper_1_y',
        ],
    'qc_perception_msgs/msg/QcObstacles':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq', 'frame_id',
            'id', 'type', 'confidence', 'sub_type',
            'x', 'y', 'z', 'vx', 'vy', 'yaw', 'length', 'width', 'height', 'age', 'track_status',
        ],
    'qc_perception_msgs/msg/QcPose':
        [
            'local_time', 'time_stamp', 'header_stamp', 'header_seq',
            'roll', 'pitch', 'yaw', 'vx', 'vy', 'vel', 'trip_distance',
        ]
}

topic2msg = {
    '/Camera/FrontWide/H265': 'sensor_msgs/msg/CompressedImage',
    '/SA/INSPVA': 'gnss_imu_msgs/msg/Inspva',
    '/PK/DR/Result': 'parking_ego_motion_msgs/msg/DrResult',
    '/VA/BevLines': 'env_perception_msgs/msg/EnvFusLines',
    '/VA/Lines': 'proto_horizon_msgs/msg/Lines',
    '/Groundtruth/VA/Lines': 'proto_horizon_msgs/msg/Lines',
    '/VA/FusLines': 'proto_horizon_msgs/msg/FusLines',
    '/VA/FusObjects': 'proto_horizon_msgs/msg/Objects',
    '/VA/Objects': 'proto_horizon_msgs/msg/Objects',
    '/Groundtruth/VA/Objects': 'proto_horizon_msgs/msg/Objects',
    '/VA/VehicleMotionIpd': 'vehicle_msgs/msg/VehicleMotionIpd',
    '/VA/Obstacles': 'proto_horizon_msgs/msg/Obstacles',
    '/VA/PK/Obstacles': 'proto_horizon_msgs/msg/Obstacles',
    '/VA/VehicleResult': 'proto_horizon_msgs/msg/Obstacles',
    '/VA/PedResult': 'proto_horizon_msgs/msg/Obstacles',
    '/Groundtruth/VA/Obstacles': 'proto_horizon_msgs/msg/Obstacles',
    '/VA/FrontViewObstacles': 'proto_horizon_msgs/msg/Obstacles',
    '/VA/BevObstaclesDet': 'pilot_perception_msg/msg/ObstaclesDet',
    '/VA/PK/BevObstaclesDet': 'pilot_perception_msg/msg/ObstaclesDet',
    '/VA/FrontWideObstacles2dDet': 'pilot_perception_msg/msg/Obstacles2dDet',
    '/VA/BackViewObstacles2dDet': 'pilot_perception_msg/msg/Obstacles2dDet',
    '/VA/Slots': 'proto_horizon_msgs/msg/Slots',
    '/VA/PK/Slots': 'proto_horizon_msgs/msg/Slots',
    '/PK/PER/VisionSlotDecodingList': 'parking_perception_msgs/msg/VisionSlotDecodingList',
    '/SAFrontRadarObject': 'sensor_abstraction_msgs/msg/RadarObjectArray',
    '/SASR5FrontLeftCornerRadarObject': 'sensor_abstraction_msgs/msg/RadarObjectArray',
    '/SASR5FrontRightCornerRadarObject': 'sensor_abstraction_msgs/msg/RadarObjectArray',
    '/SASR5RearLeftCornerRadarObject': 'sensor_abstraction_msgs/msg/RadarObjectArray',
    '/SASR5RearRightCornerRadarObject': 'sensor_abstraction_msgs/msg/RadarObjectArray',
    '/PI/FS/ObjTracksHorizon': 'per_fusion_msgs/msg/ObjTracks',
    '/PI/FS/LaneMarkingsHorizon': 'per_fusion_msgs/msg/LaneMarkings',
    '/PI/FS/LaneMarkingsHorizonDebug': 'per_fusion_msgs/msg/LaneMarkings',
    '/PI/EG/EgoMotionInfo': 'pilot_ego_motion_msgs/msg/EgoMotionInfo',
    '/sil/scenario': 'std_msgs/msg/Header',
    '/SA/VisionLaneMarkList': 'pilot_perception_msg/msg/VisionLaneMarkList',
    '/VA/WeatherType': 'proto_horizon_msgs/msg/WorkCondition',
    '/VA/LightType': 'proto_horizon_msgs/msg/WorkCondition',
    '/VA/Freespaces': 'proto_horizon_msgs/msg/Freespaces',
    '/LP/Freespaces': 'proto_horizon_msgs/msg/Freespaces',
    '/VA/PK/Freespaces': 'proto_horizon_msgs/msg/Freespaces',
    '/PI/CUS/HMIOutputES33': 'cus_app_msgs/msg/HmiES33',
    '/PP/CUS/FctDebug': 'cus_app_msgs/msg/FctDebug',
    '/VA/QC/BEVObstaclesTracks': 'qc_perception_msgs/msg/QcObstacles',
    '/VA/QC/MonoObstaclesTracks': 'qc_perception_msgs/msg/QcObstacles',
    '/VA/QC/FsObstacles': 'qc_perception_msgs/msg/QcObstacles',
    '/VA/QC/Lines': 'qc_perception_msgs/msg/QcLines',
    '/VA/QC/Objects': 'qc_perception_msgs/msg/QcObjects',
    '/VA/QC/Pose': 'qc_perception_msgs/msg/QcPose',
}


class Ros2BagParser:

    def __init__(self, workspace):
        self.last_timestamp = None
        self.frame_id_saver = None
        self.time_saver = None
        self.install_folder = os.path.join(workspace, 'install')
        self.typestore = get_typestore(Stores.LATEST)
        msg_list = []
        for root, dirs, files in os.walk(self.install_folder):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        for root, dirs, files in os.walk('/opt/ros/rolling'):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        add_types = {}
        for pathstr in msg_list:
            msg_path = Path(pathstr)
            msg_def = msg_path.read_text(encoding='utf-8')
            temp = get_types_from_msg(msg_def, self.getMsgType(msg_path))
            if list(temp.keys())[0] not in self.typestore.types.keys():
                add_types.update(temp)
        register_types(add_types)

        # for pathstr in msg_list:
        #     msg_path = Path(pathstr)
        #     msg_def = msg_path.read_text(encoding='utf-8')
        #     temp = get_types_from_msg(msg_def, self.getMsgType(msg_path))
        #     if list(temp.keys())[0] not in self.typestore.types.keys():
        #         self.typestore.register(temp)
        # print(self.typestore.types.keys())
        # print('proto_horizon_msgs/msg/Obstacles' in self.typestore.types.keys())

    def getMsgType(self, path: Path) -> str:
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    def getMsgInfo(self, bag_path, topic_list, folder, tag):
        # 创建test
        TestTopicInfo = {
            'topics_for_parser': topic_list
        }
        with open(os.path.join(folder, 'TestTopicInfo.yaml'), 'w', encoding='utf-8') as f:
            yaml.dump(TestTopicInfo, f, encoding='utf-8', allow_unicode=True)

        self.last_timestamp = {
            topic: 0 for topic in topic_list
        }
        queue_by_topic = {
            topic: mp.Queue() for topic in topic_list
        }
        msg_saver = {
            topic: MsgSaver(topic, folder, tag, queue_by_topic[topic], data_columns[topic2msg[topic]]) for
            topic in topic_list
        }
        self.time_saver = {
            topic: [] for topic in topic_list
        }
        self.frame_id_saver = {
            topic: [] for topic in topic_list
        }

        local_time_saver = {
            topic: [] for topic in topic_list
        }

        for topic in msg_saver.keys():
            msg_saver[topic].start_save_process()

        t0 = time.time()
        connections = []
        with Reader(bag_path) as reader:
            for x in reader.connections:
                print(x.topic)
                if x.topic in topic_list:
                    connections.append(x)

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = deserialize_cdr(rawdata, connection.msgtype)
                local_time_saver[connection.topic].append(timestamp)
                self.parser(timestamp, connection.topic, msg, queue_by_topic[connection.topic])

        for topic in msg_saver.keys():
            msg_saver[topic].terminate_save_process()
            # 顺便求出频率

            time_df = pd.DataFrame()
            time_df['local_time'] = [t / 1e9 for t in local_time_saver[topic]]
            time_df['time_stamp'] = self.time_saver[topic]
            time_df['frame_id'] = self.frame_id_saver[topic]
            time_df = time_df.sort_values(by=['time_stamp', 'local_time'])
            new_time_df = time_df.drop_duplicates(subset=['time_stamp'], keep='first')
            s = time_df['time_stamp'].value_counts()
            new_time_df['count'] = [s.loc[t] for t in new_time_df['time_stamp'].values]
            if len(new_time_df) > 20:
                new_time_df = new_time_df.drop_duplicates(subset=['time_stamp'], keep='first').iloc[10:]

            print(f'======正在计算{topic}的hz======')
            hz = 0
            if len(local_time_saver[topic]) > 20:
                if new_time_df['time_stamp'].max() - new_time_df['time_stamp'].min() > 10:
                    hz = (len(new_time_df) - 1) / (new_time_df['time_stamp'].max() - new_time_df['time_stamp'].min())
                else:
                    hz = (len(time_df) - 1) / (time_df['local_time'].max() - time_df['local_time'].min())

            time_csv = os.path.join(folder, '{:s}_{:.2f}_hz.csv'.format(topic.replace('/', ''), hz))
            new_time_df.to_csv(time_csv, index=False)
            print(topic, f'hz = {hz}')

        print(time.time() - t0)

    def getTopics(self, msg):
        return [key for key, value in topic2msg.items() if value == msg]

    def parser(self, timestamp, topic, msg, queue):
        local_time = timestamp / 1e9
        if topic in self.getTopics('proto_horizon_msgs/msg/Obstacles'):
            time_stamp = msg.exposure_time_stamp / 1000
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for obstacle_data in msg.obstacles:
                    obstacle_id = obstacle_data.obstacle_id
                    if obstacle_id == 0:
                        continue

                    obstacle_type = obstacle_data.obstacle_type
                    world_info = obstacle_data.world_info
                    point_type = world_info.position_type
                    a = 0 if point_type in [7, 8, 9, 0] else 1 / 2 if point_type in [4, 5, 6] else -1 / 2
                    b = 0 if point_type in [0, 2, 5, 9] else 1 / 2 if point_type in [1, 4, 7] else -1 / 2

                    if obstacle_type in [0, 1]:
                        obstacle_type = 1
                        obstacle_type_text = 'vehicle'
                    elif obstacle_type == 2:
                        obstacle_type_text = 'pedestrian'
                    else:
                        obstacle_type_text = 'cyclist'

                    ped_yaws = {
                        0: np.pi,
                        1: 0,
                        2: np.pi / 2,
                        3: np.pi / 4,
                        4: np.pi / 4 * 3,
                        5: -np.pi / 2,
                        6: -np.pi / 4 * 3,
                        7: -np.pi / 4
                    }

                    yaw = world_info.yaw
                    if obstacle_type_text == 'pedestrian':
                        for category in obstacle_data.category:
                            if category.category_property_conf and category.category_property_type == 0:
                                yaw = ped_yaws[category.category_property]
                                break

                    length = world_info.length
                    width = world_info.width
                    height = world_info.height
                    x = world_info.position.x - a * length * np.cos(yaw) + b * width * np.sin(yaw)
                    y = world_info.position.y - a * length * np.sin(yaw) - b * width * np.cos(yaw)
                    vx, vx_rel = world_info.vel_abs_world.vx, world_info.vel.vx
                    vy, vy_rel = world_info.vel_abs_world.vy, world_info.vel.vy

                    sub_type = obstacle_data.sub_type
                    type_conf = obstacle_data.type_conf
                    if type_conf > 1:
                        type_conf = obstacle_data.type_conf / 100
                    curr_lane = world_info.curr_lane
                    age = obstacle_data.age
                    coverage = world_info.cover_ratio
                    is_cipv = world_info.is_cipv
                    is_mcp = world_info.is_mcp
                    status = world_info.measurement_status

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        obstacle_id, obstacle_type, obstacle_type_text, type_conf, sub_type, curr_lane,
                        x, y, world_info.position.z, vx, vy, vx_rel, vy_rel, yaw, length, width, height, age, coverage,
                        is_cipv, is_mcp, status,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic in ['/VA/VehicleResult', '/VA/PedResult']:
            time_stamp = msg.exposure_time_stamp / 1000
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for obstacle_data in msg.obstacles:
                    obstacle_id = obstacle_data.obstacle_id
                    if obstacle_id == 0:
                        continue

                    obstacle_type = obstacle_data.obstacle_type
                    world_info = obstacle_data.world_info
                    point_type = world_info.position_type
                    a = 0 if point_type in [7, 8, 9, 0] else 1 / 2 if point_type in [4, 5, 6] else -1 / 2
                    b = 0 if point_type in [0, 2, 5, 9] else 1 / 2 if point_type in [1, 4, 7] else -1 / 2

                    if obstacle_type in [0, 1]:
                        obstacle_type = 1
                        obstacle_type_text = 'vehicle'
                    elif obstacle_type == 2:
                        obstacle_type_text = 'pedestrian'
                    else:
                        obstacle_type_text = 'cyclist'

                    ped_yaws = {
                        0: np.pi,
                        1: 0,
                        2: np.pi / 2,
                        3: np.pi / 4,
                        4: np.pi / 4 * 3,
                        5: -np.pi / 2,
                        6: -np.pi / 4 * 3,
                        7: -np.pi / 4
                    }

                    yaw = world_info.yaw
                    if obstacle_type_text == 'pedestrian':
                        for category in obstacle_data.category:
                            if category.category_property_conf and category.category_property_type == 0:
                                yaw = ped_yaws[category.category_property]
                                break

                    sub_type = 0
                    if obstacle_type in [0, 1]:
                        for category in obstacle_data.category:
                            if category.category_property_type == 0:
                                # property_name = np.array(category.property_name, dtype=np.uint8).tobytes().decode('ascii')
                                # property_name = ''.join([a for a in property_name if a.isalpha()])
                                # sub_type = f'{category.category_property}-{property_name}'
                                if category.category_property in [1, 10, 3]:
                                    sub_type = 1
                                elif category.category_property == 0:
                                    sub_type = 4
                                elif category.category_property == 2:
                                    sub_type = 5
                                elif category.category_property in [11, 7, 8]:
                                    sub_type = 3
                                elif category.category_property == 5:
                                    sub_type = 11
                                else:
                                    sub_type = 10
                                break

                    length = world_info.length
                    width = world_info.width
                    height = world_info.height
                    x = world_info.position.x - a * length * np.cos(yaw) + b * width * np.sin(yaw)
                    y = world_info.position.y - a * length * np.sin(yaw) - b * width * np.cos(yaw)
                    vx, vx_rel = world_info.vel_abs_world.vx, world_info.vel.vx
                    vy, vy_rel = world_info.vel_abs_world.vy, world_info.vel.vy

                    type_conf = obstacle_data.type_conf
                    if type_conf > 1:
                        type_conf = obstacle_data.type_conf / 100
                    curr_lane = world_info.curr_lane
                    age = obstacle_data.age
                    coverage = world_info.cover_ratio
                    is_cipv = world_info.is_cipv
                    is_mcp = world_info.is_mcp
                    status = world_info.measurement_status

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        obstacle_id, obstacle_type, obstacle_type_text, type_conf, sub_type, curr_lane,
                        x, y, world_info.position.z, vx, vy, vx_rel, vy_rel, yaw, length, width, height, age, coverage,
                        is_cipv, is_mcp, status,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic in self.getTopics('pilot_perception_msg/msg/ObstaclesDet'):
            time_stamp = msg.time_stamp / 1000
            frame_id = msg.frame_num
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq
                veh_info = msg.veh
                veh_num = msg.veh_num
                ped_cyc_info = msg.ped_cyc
                ped_cyc_num = msg.ped_cyc_num
                ob_id = (frame_id * 64) % 200000

                for veh_id in range(veh_num):
                    obj_data = veh_info[veh_id]
                    ob_id += 1
                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        ob_id, 1, 'vehicle', obj_data.obj_confidence, obj_data.obj_type, 0,
                        obj_data.center_x, obj_data.center_y, obj_data.center_z,
                        obj_data.velocity_x, obj_data.velocity_y, 0, 0,
                        obj_data.yaw, obj_data.obj_length, obj_data.obj_width, obj_data.obj_height,
                        0, 0, 0, 0, 0,
                    ])

                for ped_cyc_id in range(ped_cyc_num):
                    obj_data = ped_cyc_info[ped_cyc_id]
                    ob_id += 1
                    if obj_data.obj_type == 18:
                        obstacle_type_text = 'cyclist'
                        ob_type = 18
                    else:
                        obstacle_type_text = 'pedestrian'
                        ob_type = 2
                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        ob_id, ob_type, obstacle_type_text, obj_data.obj_confidence, -1, 0,
                        obj_data.center_x, obj_data.center_y, obj_data.center_z,
                        obj_data.velocity_x, obj_data.velocity_y, 0, 0,
                        obj_data.yaw, obj_data.obj_length, obj_data.obj_width, obj_data.obj_height,
                        0, 0, 0, 0, 0,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic in ['/VA/FrontWideObstacles2dDet', '/VA/BackViewObstacles2dDet']:
            time_stamp = msg.timestamp / 1000
            frame_id = msg.framenum
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            ob_id = (frame_id * 64) % 200000
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq
                obstacles2d_list = msg.obstacles2d_list
                for obstacles2d in obstacles2d_list:
                    veh_num = obstacles2d.veh_num
                    veh_info_list = obstacles2d.veh
                    ped_cyc_num = obstacles2d.ped_cyc_num
                    ped_cyc_info_list = obstacles2d.ped_cyc
                    cone_num = obstacles2d.cone_num
                    cone_info_list = obstacles2d.cone

                    for veh_id in range(min(veh_num, 32)):
                        obj_data = veh_info_list[veh_id]
                        ob_id += 1
                        world_info = obj_data.world_info
                        if world_info.center_x != 0 and world_info.center_y != 0:
                            if obj_data.veh_type == 0:
                                sub_type = 1
                            elif obj_data.veh_type == 1:
                                sub_type = 4
                            elif obj_data.veh_type == 2:
                                sub_type = 5
                            elif obj_data.veh_type == 3:
                                sub_type = 9
                            else:
                                sub_type = 0
                            queue.put([
                                local_time, time_stamp, header_stamp, header_seq, frame_id,
                                ob_id, 1, 'vehicle', obj_data.obj_conf, sub_type, 0,
                                world_info.center_x, world_info.center_y, world_info.center_z,
                                world_info.velocity_x, world_info.velocity_y, 0, 0,
                                world_info.yaw, world_info.obj_length, world_info.obj_width, world_info.obj_height,
                                0, 0, 0, 0, 0,
                            ])

                    for ped_cyc_id in range(min(ped_cyc_num, 32)):
                        obj_data = ped_cyc_info_list[ped_cyc_id]
                        ob_id += 1
                        world_info = obj_data.world_info
                        if world_info.center_x != 0 and world_info.center_y != 0:
                            if obj_data.veh_type == 1:
                                obj_type_text = 'cyclist'
                                ob_type = 18
                            elif obj_data.veh_type == 0:
                                obj_type_text = 'pedestrian'
                                ob_type = 2
                            else:
                                obj_type_text = 'pedestrian'
                                ob_type = 2
                            queue.put([
                                local_time, time_stamp, header_stamp, header_seq, frame_id,
                                ob_id, ob_type, obj_type_text, obj_data.obj_conf, -1, 0,
                                world_info.center_x, world_info.center_y, world_info.center_z,
                                world_info.velocity_x, world_info.velocity_y, 0, 0,
                                world_info.yaw, world_info.obj_length, world_info.obj_width, world_info.obj_height,
                                0, 0, 0, 0, 0,
                            ])

                    self.last_timestamp[topic] = time_stamp

        elif topic in ['/PI/FS/ObjTracksHorizon']:
            time_stamp = msg.time_stamp
            frame_id = 0
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for obj_info in msg.objects:
                    if obj_info.obj_is_valid:
                        NaturalAttr = obj_info.attr
                        FusedQuality = obj_info.quality
                        FusedState = obj_info.state

                        obstacle_id = FusedState.track_id
                        x = FusedState.long_position
                        y = FusedState.lat_position
                        vx, vx_rel = FusedState.long_velocity_abs, FusedState.long_velocity_relative
                        vy, vy_rel = FusedState.lat_velocity_abs, FusedState.lat_velocity_relative
                        ax, ax_rel = FusedState.long_acceleration_abs, FusedState.long_acceleration_relative
                        ay, ay_rel = FusedState.lat_acceleration_abs, FusedState.lat_acceleration_relative
                        yaw = FusedState.yaw
                        length = NaturalAttr.length
                        width = NaturalAttr.width
                        height = NaturalAttr.height
                        obstacle_type = NaturalAttr.obj_type
                        if obstacle_type == 1:
                            obj_type = 2
                            obj_type_text = 'pedestrian'
                            sub_type = -1
                        elif obstacle_type == 2:
                            obj_type = 18
                            obj_type_text = 'cyclist'
                            sub_type = -1
                        elif obstacle_type == 3:
                            obj_type = 1
                            obj_type_text = 'vehicle'
                            sub_type = 1
                        elif obstacle_type in [4, 5]:
                            obj_type = 1
                            obj_type_text = 'vehicle'
                            sub_type = 4
                        elif obstacle_type in [6, 7]:
                            obj_type = 1
                            obj_type_text = 'vehicle'
                            sub_type = 5
                        elif obstacle_type == 8:
                            obj_type = 1
                            obj_type_text = 'vehicle'
                            sub_type = 9
                        elif obstacle_type == 9:
                            obj_type = 1
                            obj_type_text = 'vehicle'
                            sub_type = 3
                        elif obstacle_type == 10:
                            obj_type = 1
                            obj_type_text = 'vehicle'
                            sub_type = 10
                        else:
                            continue

                        type_conf = NaturalAttr.type_conf
                        is_cipv = NaturalAttr.is_cipv
                        is_mcp = NaturalAttr.is_mcp
                        queue.put([
                            local_time, time_stamp, header_stamp, header_seq, frame_id,
                            obstacle_id, obj_type, obj_type_text, type_conf, sub_type, 0,
                            x, y, 0, vx, vy, vx_rel, vy_rel, yaw, length, width, height, 0, 0,
                            is_cipv, is_mcp, 0,
                        ])

                self.last_timestamp[topic] = time_stamp

        elif topic in ['/VA/Lines', '/Groundtruth/VA/Lines', 'VA/FusLines']:
            time_stamp = msg.exposure_time_stamp / 1000
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                lines_num = msg.lines_num
                for line_idx in range(lines_num):
                    line_data = msg.lines[line_idx]
                    line_id = line_data.line_id
                    if line_id == 0:
                        continue

                    line_type = line_data.line_type
                    line_position = line_data.line_position
                    conf = line_data.confidence

                    lines_3d_num = line_data.lines_3d_num
                    for line3d_idx in range(lines_3d_num):
                        line3d_data = line_data.lines_3d[line3d_idx]
                        if line3d_data.width == 0 and line3d_data.t_max == 0:
                            continue

                        start_x = line3d_data.start_pt.x
                        start_y = line3d_data.start_pt.y

                        c_x_0 = line3d_data.x_coeffs[0]
                        c_x_1 = line3d_data.x_coeffs[1]
                        c_x_2 = line3d_data.x_coeffs[2]
                        c_x_3 = line3d_data.x_coeffs[3]

                        c_y_0 = line3d_data.y_coeffs[0]
                        c_y_1 = line3d_data.y_coeffs[1]
                        c_y_2 = line3d_data.y_coeffs[2]
                        c_y_3 = line3d_data.y_coeffs[3]

                        length = line3d_data.t_max
                        width = line3d_data.width
                        line_color = line3d_data.line_color
                        line_marking = line3d_data.line_marking
                        curve_type = line3d_data.curve_type

                        queue.put([
                            local_time, time_stamp, header_stamp, header_seq, frame_id,
                            line_id, line_type, conf, line_position, line_marking, line_color,
                            start_x, start_y, c_x_0, c_x_1, c_x_2, c_x_3,
                            c_y_0, c_y_1, c_y_2, c_y_3, length, width, curve_type,
                        ])

                self.last_timestamp[topic] = time_stamp

        elif topic in ['/VA/BevLines']:
            time_stamp = msg.exposure_time_stamp / 1e3
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                lines_num = msg.lines_num
                for line_idx in range(lines_num):
                    line_data = msg.lines[line_idx]
                    line_id = line_data.line_id
                    if line_id == 0:
                        continue

                    line_type = line_data.line_type
                    line_position = line_data.line_position
                    conf = line_data.confidence
                    extra_type = line_data.extra_type

                    line3d_data = line_data.lines_3d[0]
                    points_num = line3d_data.points_num
                    if points_num >= 2:
                        x_points = []
                        y_points = []
                        for i in range(points_num):
                            pt = line3d_data.points[i]
                            x_points.append(pt.x)
                            y_points.append(pt.y)

                        # start_x = x_points[0]
                        # start_y = y_points[0]
                        #
                        # c_x_0 = line3d_data.x_coeffs[0]
                        # c_x_1 = line3d_data.x_coeffs[1]
                        # c_x_2 = line3d_data.x_coeffs[2]
                        # c_x_3 = line3d_data.x_coeffs[3]
                        #
                        # c_y_0 = line3d_data.y_coeffs[0]
                        # c_y_1 = line3d_data.y_coeffs[1]
                        # c_y_2 = line3d_data.y_coeffs[2]
                        # c_y_3 = line3d_data.y_coeffs[3]
                        #
                        # end_x = x_points[points_num - 1]
                        # width = 0.2

                        line_color = line3d_data.line_color
                        if line_type == 2:
                            line_marking = 2
                        elif line_type == 1:
                            line_marking = extra_type & 0xF
                        else:
                            line_marking = 0

                        x_point_str = ','.join([f'{x:.3f}' for x in x_points])
                        y_point_str = ','.join([f'{y:.3f}' for y in y_points])

                        queue.put([
                            local_time, time_stamp, header_seq, header_stamp, frame_id,
                            line_id, line_type, conf,
                            line_position, line_marking, line_color,
                            extra_type, x_point_str, y_point_str,
                        ])

                self.last_timestamp[topic] = time_stamp

        elif topic in ['/PI/FS/LaneMarkingsHorizonDebug', '/PI/FS/LaneMarkingsHorizon']:
            time_stamp = msg.time_stamp / 1000
            frame_id = 0
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for line_data in msg.lines:
                    line_id = line_data.line_id
                    if line_id == 0:
                        continue

                    line_type = line_data.line_type

                    if line_data.line_position == 1:
                        line_position = 0
                    elif line_data.line_position == 2:
                        line_position = 2
                    elif line_data.line_position == 3:
                        line_position = 4
                    elif line_data.line_position == 4:
                        line_position = 8
                    elif line_data.line_position == 5:
                        line_position = 16
                    elif line_data.line_position == 6:
                        line_position = 32
                    elif line_data.line_position == 7:
                        line_position = 64
                    elif line_data.line_position == 8:
                        line_position = 128
                    else:
                        line_position = -1

                    conf = line_data.confidence

                    lines_3d_num = line_data.lines_3d_num
                    for line3d_idx in range(lines_3d_num):
                        line3d_data = line_data.lines_3d[line3d_idx]
                        if line3d_data.width == 0 and line3d_data.t_max == 0:
                            continue

                        start_x = line3d_data.start_pt.x
                        start_y = line3d_data.start_pt.y

                        c_x_0 = line3d_data.x_coeffs[0]
                        c_x_1 = line3d_data.x_coeffs[1]
                        c_x_2 = line3d_data.x_coeffs[2]
                        c_x_3 = line3d_data.x_coeffs[3]

                        c_y_0 = line3d_data.y_coeffs[0]
                        c_y_1 = line3d_data.y_coeffs[1]
                        c_y_2 = line3d_data.y_coeffs[2]
                        c_y_3 = line3d_data.y_coeffs[3]

                        length = line3d_data.t_max
                        width = line3d_data.width

                        if line3d_data.line_color == 1:
                            line_color = 2
                        elif line3d_data.line_color == 2:
                            line_color = 4
                        elif line3d_data.line_color == 3:
                            line_color = 8
                        elif line3d_data.line_color == 4:
                            line_color = 16
                        elif line3d_data.line_color == 5:
                            line_color = 32
                        elif line3d_data.line_color == 6:
                            line_color = 64
                        elif line3d_data.line_color == 7:
                            line_color = 128
                        elif line3d_data.line_color == 8:
                            line_color = 256
                        else:
                            line_color = 0

                        if line3d_data.line_marking == 1:
                            line_marking = 2
                        elif line3d_data.line_marking == 2:
                            line_marking = 4
                        elif line3d_data.line_marking == 3:
                            line_marking = 8
                        elif line3d_data.line_marking == 4:
                            line_marking = 16
                        elif line3d_data.line_marking == 5:
                            line_marking = 32
                        elif line3d_data.line_marking == 6:
                            line_marking = 64
                        elif line3d_data.line_marking == 7:
                            line_marking = 128
                        else:
                            line_marking = 0

                        curve_type = 0

                        queue.put([
                            local_time, time_stamp, header_stamp, header_seq, frame_id,
                            line_id, line_type, conf, line_position, line_marking, line_color,
                            start_x, start_y, c_x_0, c_x_1, c_x_2, c_x_3,
                            c_y_0, c_y_1, c_y_2, c_y_3, length, width, curve_type,
                        ])

        elif topic in ['/SA/VisionLaneMarkList']:
            time_stamp = msg.time_stamp / 1000000
            frame_id = msg.frame_num
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                idx2position = [0, 2, 4, 8, 64, 128]
                color_converter = {
                    0: 2,
                    1: 4,
                    2: 8,
                    3: -1,
                }
                marker_converter = {
                    0: 0,
                    1: 2,
                    2: 64,
                    3: 8,
                    4: 128,
                    5: 16,
                    6: 32,
                    7: 0,
                    8: 16384,
                    9: 0,
                }
                for idx, line in enumerate([msg.left_host_individual, msg.right_host_individual,
                                            msg.left_neighbor_lane, msg.right_neighbor_lane,
                                            msg.left_roadedge, msg.right_roadedge]):
                    line_id = line.lane_id

                    if line_id != 0:
                        if idx <= 3:
                            line_type = 2
                        else:
                            line_type = 64

                        conf = line.lane_exist_prob
                        line_position = idx2position[idx]
                        line_marker = marker_converter[line.lane_mark_type]
                        line_color = color_converter[line.lane_marker_color]
                        start_x = line.curve_function[0].start_point.x_0
                        start_y = - line.curve_function[0].start_point.y_0
                        c_x_0, c_x_1, c_x_2, c_x_3 = 0, 0, 0, 0
                        c_y_0 = - line.curve_function[0].lat_distance_zero_order_coeff
                        c_y_1 = - line.curve_function[0].lat_distance_first_order_coeff
                        c_y_2 = - line.curve_function[0].lat_distance_second_order_coeff
                        c_y_3 = - line.curve_function[0].lat_distance_third_order_coeff
                        length = line.curve_function[0].long_distance_to_end
                        width = line.lane_width
                        curve_type = 2

                        queue.put([
                            local_time, time_stamp, header_stamp, header_seq, frame_id,
                            line_id, line_type, conf, line_position, line_marker, line_color,
                            start_x, start_y, c_x_0, c_x_1, c_x_2, c_x_3,
                            c_y_0, c_y_1, c_y_2, c_y_3, length, width, curve_type,
                        ])

                self.last_timestamp[topic] = time_stamp

        elif topic in ['/VA/Objects', '/Groundtruth/VA/Objects', '/VA/FusObjects']:
            time_stamp = msg.exposure_time_stamp / 1000
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for object_data in msg.objects:
                    object_id = object_data.id
                    if object_id == 0:
                        continue

                    object_type = object_data.type
                    type_conf = object_data.conf / 100
                    sub_type = object_data.sub_type
                    border = object_data.border
                    x0, y0 = border.points[0].x, border.points[0].y
                    x1, y1 = border.points[1].x, border.points[1].y
                    x2, y2 = border.points[2].x, border.points[2].y
                    x3, y3 = border.points[3].x, border.points[3].y
                    position = object_data.position
                    x = position.x
                    y = position.y
                    z = position.z
                    curr_lane_flag = object_data.in_cur_lane
                    value = object_data.attr.value
                    age = object_data.age
                    child_types = object_data.child_types

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        object_id, object_type, type_conf, sub_type, curr_lane_flag,
                        x, y, z, x0, y0, x1, y1, x2, y2, x3, y3,
                        value, age, child_types,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic in self.getTopics('sensor_abstraction_msgs/msg/RadarObjectArray'):
            time_stamp = msg.timestamp / 1000
            frame_id = msg.frame_number
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for object_data in msg.radar_obj_list:
                    object_id = object_data.obj_id
                    if object_id == 0:
                        continue

                    obj_type = object_data.obj_type
                    obstacle_conf = object_data.obj_obstacle_prob
                    exist_conf = object_data.obj_exsit_prob
                    x = object_data.obj_xpos
                    y = object_data.obj_ypos
                    yaw = object_data.obj_heading_angle
                    length = object_data.obj_length
                    width = object_data.obj_width
                    vx = object_data.obj_xvel_rel
                    vy = object_data.obj_yvel_rel

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        object_id, obj_type, exist_conf, obstacle_conf, x, y, vx, vy,
                        length, width, yaw,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/PI/EG/EgoMotionInfo':
            time_stamp = msg.timestamp
            frame_id = 0
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                steering_wheel_angle = msg.steering_wheel_angle
                front_wheel_angle = msg.front_wheel_angle
                x = msg.pose.position.x
                y = msg.pose.position.y
                vx = msg.twist.linear.x
                vy = msg.twist.linear.y

                queue.put([
                    local_time, time_stamp, frame_id,
                    x, y, vx, vy, steering_wheel_angle, front_wheel_angle
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/PK/DR/Result':
            time_stamp = msg.timestamp
            frame_id = 0
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq
                x = msg.position_x
                y = msg.position_y
                vx = msg.velocity_x
                vy = msg.velocity_y
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.headrate

                queue.put([
                    local_time, time_stamp, header_stamp, header_seq, frame_id,
                    x, y, vx, vy, roll, pitch, yaw
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/VA/WeatherType':
            time_stamp = msg.timestamp / 1000
            self.time_saver[topic].append(time_stamp)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                category = msg.category
                category_property_type = category.category_property_type
                category_property = category.category_property
                category_property_conf = category.category_property_conf

                queue.put([
                    local_time, time_stamp, header_stamp, header_seq, category_property_type, category_property,
                    category_property_conf,
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/VA/LightType':
            time_stamp = msg.timestamp / 1000
            self.time_saver[topic].append(time_stamp)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                category = msg.category
                category_property_type = category.category_property_type
                category_property = category.category_property
                category_property_conf = category.category_property_conf

                queue.put([
                    local_time, time_stamp, header_stamp, header_seq, category_property_type, category_property,
                    category_property_conf,
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic in self.getTopics('proto_horizon_msgs/msg/Freespaces'):  # 可行使区域
            time_stamp = msg.time_stamp / 1000
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.head.stamp.sec + msg.head.stamp.nanosec / 1e9
                header_seq = msg.head.seq
                for points in msg.freespace_points:
                    points_num = msg.freespace_points_num
                    if valid := points.valid:
                        label = points.label
                        x = points.point_vcs.x
                        y = points.point_vcs.y
                        z = points.point_vcs.z

                        queue.put([
                            local_time, time_stamp, header_stamp, header_seq, frame_id, points_num, label, x, y, z,
                            valid
                        ])

                obs_counter = 1
                for obs in msg.obs_points:
                    points_num = msg.obs_points_num
                    if obs_valid := obs.valid:
                        contours_id = obs.contours_id
                        obj_id = obs.matched_obj.id
                        obj_type = obs.matched_obj.type

                        for points in obs.points:
                            if valid := points.valid:
                                label = points.label
                                x = points.point_vcs.x
                                y = points.point_vcs.y
                                z = points.point_vcs.z

                                queue.put([
                                    local_time, time_stamp, header_stamp, header_seq, frame_id, obs_counter, label, x,
                                    y, z, obs_valid, contours_id, obj_id, obj_type
                                ])
                        obs_counter += 1

                self.last_timestamp[topic] = time_stamp

        elif topic in self.getTopics('proto_horizon_msgs/msg/Slots'):  # 后处理输出车位
            frame_id = msg.frame_id
            time_stamp = msg.timestamp / 1000

            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                for object_data in msg.objects_array:
                    obj_id = object_data.id
                    if obj_id == 0:
                        # id和conf为0表示无此车位
                        continue

                    obj_type = object_data.type  # 静态障碍物的类型，Type=10表示停车位
                    obj_conf = object_data.conf
                    obj_life_time = object_data.life_time
                    obj_age = object_data.age
                    slot_attr = object_data.attr.parking_slot_attr  # 单个车位的属性
                    slot_type = slot_attr.type  # 0:unknown 未知;1:vertical 垂直;2:parallel 水平;3:oblique 斜列

                    slot_pose_width = slot_attr.pose.width
                    slot_pose_length = slot_attr.pose.length
                    slot_pose_angle = slot_attr.pose.angle
                    slot_heading = slot_attr.pose.yaw_vcs
                    slot_center_x = slot_attr.pose.center_vcs.x
                    slot_center_y = slot_attr.pose.center_vcs.y
                    pt_0_x = slot_attr.pose.poly_vcs[0].x
                    pt_0_y = slot_attr.pose.poly_vcs[0].y
                    pt_1_x = slot_attr.pose.poly_vcs[1].x
                    pt_1_y = slot_attr.pose.poly_vcs[1].y
                    pt_2_x = slot_attr.pose.poly_vcs[2].x
                    pt_2_y = slot_attr.pose.poly_vcs[2].y
                    pt_3_x = slot_attr.pose.poly_vcs[3].x
                    pt_3_y = slot_attr.pose.poly_vcs[3].y

                    slot_occupy_info = slot_attr.occupy_info
                    occupy_status = slot_occupy_info.occupy_status  # 0: Unknown; 1: Occupied; 2: Not occupied
                    slot_occupy_lock_info_valid = slot_occupy_info.lock_info.valid  # 0:invalid; 1: valid
                    if not slot_occupy_lock_info_valid:
                        lock_status = 0
                    else:
                        lock_status = slot_occupy_info.lock_info.lock_status
                    lock_center_x = slot_occupy_info.lock_info.position_vcs.x
                    lock_center_y = slot_occupy_info.lock_info.position_vcs.y

                    if slot_attr.rod_info.valid:
                        stopper_valid = 1
                        stopper_0_x = slot_attr.rod_info.end_vcs[0].x
                        stopper_0_y = slot_attr.rod_info.end_vcs[0].y
                        stopper_1_x = slot_attr.rod_info.end_vcs[1].x
                        stopper_1_y = slot_attr.rod_info.end_vcs[1].y
                    else:
                        stopper_valid = 0
                        stopper_0_x = 0
                        stopper_0_y = 0
                        stopper_1_x = 0
                        stopper_1_y = 0

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id, obj_id,
                        obj_type, obj_conf, obj_life_time, obj_age, slot_type,
                        slot_pose_length, slot_pose_width,
                        slot_pose_angle, slot_heading, slot_center_x, slot_center_y,
                        pt_0_x, pt_0_y, pt_1_x, pt_1_y,
                        pt_2_x, pt_2_y, pt_3_x, pt_3_y,
                        occupy_status, lock_status, lock_center_x, lock_center_y,
                        stopper_valid, stopper_0_x, stopper_0_y, stopper_1_x, stopper_1_y,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/PK/PER/VisionSlotDecodingList':  # 模型输出车位
            frame_id = msg.frame_num
            time_stamp = msg.time_stamp / 1000

            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                slot_num = msg.slot_num
                for i in range(slot_num):
                    slot = msg.slot[i]
                    p1 = slot.p1
                    pt_0_x = p1.x
                    pt_0_y = p1.y
                    p2 = slot.p2
                    pt_1_x = p2.x
                    pt_1_y = p2.y
                    p3 = slot.p3
                    pt_2_x = p3.x
                    pt_2_y = p3.y
                    p4 = slot.p4
                    pt_3_x = p4.x
                    pt_3_y = p4.y
                    slot_center_x = (pt_0_x + pt_1_x + pt_2_x + pt_3_x) / 4
                    slot_center_y = (pt_0_y + pt_1_y + pt_2_y + pt_3_y) / 4

                    p5 = slot.p5
                    stopper_0_x = p5.x
                    stopper_0_y = p5.y
                    p6 = slot.p6
                    stopper_1_x = p6.x
                    stopper_1_y = p6.y
                    if stopper_1_x * stopper_1_y != 0:
                        stopper_valid = 1
                    else:
                        stopper_valid = 0

                    slot_type = slot.type  # 0:unknown 未知;1:vertical 垂直;2:parallel 水平;3:oblique 斜列
                    obj_conf = slot.conf_slot
                    lock_status = slot.lock_status
                    occupy_status = slot.status  # 0: Unknown; 1: Occupied; 2: Not occupied

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id, -1,
                        10, obj_conf, slot_type, slot_center_x, slot_center_y,
                        pt_0_x, pt_0_y, pt_1_x, pt_1_y,
                        pt_2_x, pt_2_y, pt_3_x, pt_3_y,
                        occupy_status, lock_status,
                        stopper_valid, stopper_0_x, stopper_0_y, stopper_1_x, stopper_1_y,
                    ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/PI/CUS/HMIOutputES33':
            header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            header_seq = msg.header.seq
            time_stamp = header_stamp
            frame_id = header_seq
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                acc_status = msg.acc_status
                acc_fault_status = msg.acc_fault_status
                acc_tja_sw_status = msg.acc_tja_sw_status
                lks_status = msg.lks_status
                zop_sw_cmd = msg.zop_sw_cmd
                pilot_sw_status = msg.pilot_sw_status
                distance = msg.distance
                acc_cannot_acive_msg = msg.acc_cannot_acive_msg
                pilot_exit_takeover_msg = msg.pilot_exit_takeover_msg
                pilot_req_eps_status = msg.pilot_req_eps_status
                pilot_req_eps_status_validity = msg.pilot_req_eps_status_validity
                auto_lane_chng_left_status = msg.auto_lane_chng_left_status
                auto_lane_chng_right_status = msg.auto_lane_chng_right_status
                auto_lane_chng_msg_req = msg.auto_lane_chng_msg_req
                auto_lane_chng_style_set_response = msg.auto_lane_chng_style_set_response
                cc_speed_voice_cmd = msg.cc_speed_voice_cmd
                cc_speed_voice_cmd_confirm_status = msg.cc_speed_voice_cmd_confirm_status
                front_video_ctrl_monitor_fault_status = msg.front_video_ctrl_monitor_fault_status
                zop_system_status = msg.zop_system_status
                auto_lane_chng_sw_cmd = msg.auto_lane_chng_sw_cmd
                lane_change_recmed_status = msg.lane_change_recmed_status
                lane_chng_rcmd_in_hmi_status = msg.lane_chng_rcmd_in_hmi_status
                lane_chng_msg_req = msg.lane_chng_msg_req
                tja_ica_sys_sts = msg.tja_ica_sys_sts
                tja_ica_sys_fault_status = msg.tja_ica_sys_fault_status
                hod_status_detection = msg.hod_status_detection
                hod_status_detection_valid = msg.hod_status_detection_valid
                distance_level = msg.distance_level
                slif_mode_resp = msg.slif_mode_resp
                slif_status = msg.slif_status
                ai_pilot_sys_flt_sts = msg.ai_pilot_sys_flt_sts
                lane_change_lamps_ctrl = msg.lane_change_lamps_ctrl
                tts_req = msg.tts_req
                zop_cannot_active_msg = msg.zop_cannot_active_msg
                zop_status = msg.zop_status
                slif_limit_ahead_source = msg.slif_limit_ahead_source
                ilc_sw_cmd = msg.ilc_sw_cmd
                ilc_active_dir_cmd = msg.ilc_active_dir_cmd
                ilc_status = msg.ilc_status
                alc_status = msg.alc_status
                acc_seld_trgt_dist_lvl_es37 = msg.acc_seld_trgt_dist_lvl_es37
                spd_lmt_ofstset_resp = msg.spd_lmt_ofstset_resp
                icp_status_es37 = msg.icp_status_es37
                ilc_status_es37 = msg.ilc_status_es37
                fct_brake_status = msg.fct_brake_status
                noa_quick_sw_msg = msg.noa_quick_sw_msg
                noa_quick_sw_cmd = msg.noa_quick_sw_cmd
                noa_decrease_dist = msg.noa_decrease_dist
                ilc_velocity_dsp = msg.ilc_velocity_dsp
                icp_sw_cmd = msg.icp_sw_cmd
                icp_status = msg.icp_status
                icp_direction = msg.icp_direction
                icp_req_msg = msg.icp_req_msg
                tsr_status = msg.tsr_status
                tsr_ban_sign = msg.tsr_ban_sign
                tsr_warn_sign = msg.tsr_warn_sign
                ldp_ldw_audio_req = msg.ldp_ldw_audio_req
                ldp_ldw_display_cmd = msg.ldp_ldw_display_cmd
                ldp_ldw_audio_warning_disp_cmd = msg.ldp_ldw_audio_warning_disp_cmd
                ldp_ldw_haptic_warning_disp_cmd = msg.ldp_ldw_haptic_warning_disp_cmd
                ldp_ldw_sensitivity_lvl = msg.ldp_ldw_sensitivity_lvl
                ldp_ldw_system_sts = msg.ldp_ldw_system_sts
                ldp_ldw_system_fault_sts = msg.ldp_ldw_system_fault_sts
                ldp_ldw_vibration_lvl_req = msg.ldp_ldw_vibration_lvl_req
                ldp_ldw_left_vis_req = msg.ldp_ldw_left_vis_req
                ldp_ldw_right_vis_req = msg.ldp_ldw_right_vis_req
                camr_sas_req_sts = msg.camr_sas_req_sts
                spd_lmt_fuc_sts = msg.spd_lmt_fuc_sts
                lane_keep_assist_sys_sts = msg.lane_keep_assist_sys_sts
                lcc_noa_switching_sw = msg.lcc_noa_switching_sw
                mad_sys_sts = msg.mad_sys_sts
                lads_progress = msg.lads_progress
                map_construction_status = msg.map_construction_status
                build_map_valid_size = msg.build_map_valid_size
                navigation_info1 = msg.navigation_info1
                navigation_info2 = msg.navigation_info2
                mad_navi_report_size = msg.mad_navi_report_size
                build_map_resp = msg.build_map_resp
                route_cancel_resp = msg.route_cancel_resp
                super_ecall = msg.super_ecall
                noa_lane_set_cmd = msg.noa_lane_set_cmd
                noa_report_set_cmd = msg.noa_report_set_cmd
                auto_main_beam_light_req = msg.auto_main_beam_light_req
                ldw_lka_hpt_wrm_dsp_cmd = msg.ldw_lka_hpt_wrm_dsp_cmd
                tja_ica_msg_req = msg.tja_ica_msg_req
                elk_dsp_cmd_ipd = msg.elk_dsp_cmd_ipd
                elk_sys_flt_sts = msg.elk_sys_flt_sts
                elk_sys_sts_ipd = msg.elk_sys_sts_ipd
                lka_sys_flt_sts = msg.lka_sys_flt_sts
                elk_warning = msg.elk_warning
                pilot_system_message = msg.pilot_system_message
                tlr_sys_sts = msg.tlr_sys_sts
                tlr_md_sw_dsp_cmd = msg.tlr_md_sw_dsp_cmd
                tlr_warning = msg.tlr_warning
                tlr_msg_req = msg.tlr_msg_req

                queue.put([
                    local_time, header_stamp, header_seq,
                    acc_status, acc_fault_status, acc_tja_sw_status, lks_status, zop_sw_cmd, pilot_sw_status, distance,
                    acc_cannot_acive_msg, pilot_exit_takeover_msg, pilot_req_eps_status, pilot_req_eps_status_validity,
                    auto_lane_chng_left_status, auto_lane_chng_right_status, auto_lane_chng_msg_req,
                    auto_lane_chng_style_set_response, cc_speed_voice_cmd, cc_speed_voice_cmd_confirm_status,
                    front_video_ctrl_monitor_fault_status, zop_system_status, auto_lane_chng_sw_cmd,
                    lane_change_recmed_status, lane_chng_rcmd_in_hmi_status, lane_chng_msg_req, tja_ica_sys_sts,
                    tja_ica_sys_fault_status, hod_status_detection, hod_status_detection_valid, distance_level,
                    slif_mode_resp, slif_status, ai_pilot_sys_flt_sts, lane_change_lamps_ctrl, tts_req,
                    zop_cannot_active_msg, zop_status, slif_limit_ahead_source, ilc_sw_cmd, ilc_active_dir_cmd,
                    ilc_status,
                    alc_status, acc_seld_trgt_dist_lvl_es37, spd_lmt_ofstset_resp, icp_status_es37, ilc_status_es37,
                    fct_brake_status, noa_quick_sw_msg, noa_quick_sw_cmd, noa_decrease_dist, ilc_velocity_dsp,
                    icp_sw_cmd,
                    icp_status, icp_direction, icp_req_msg, tsr_status, tsr_ban_sign, tsr_warn_sign, ldp_ldw_audio_req,
                    ldp_ldw_display_cmd, ldp_ldw_audio_warning_disp_cmd, ldp_ldw_haptic_warning_disp_cmd,
                    ldp_ldw_sensitivity_lvl, ldp_ldw_system_sts, ldp_ldw_system_fault_sts, ldp_ldw_vibration_lvl_req,
                    ldp_ldw_left_vis_req, ldp_ldw_right_vis_req, camr_sas_req_sts, spd_lmt_fuc_sts,
                    lane_keep_assist_sys_sts, lcc_noa_switching_sw, mad_sys_sts, lads_progress, map_construction_status,
                    build_map_valid_size, navigation_info1, navigation_info2, mad_navi_report_size, build_map_resp,
                    route_cancel_resp, super_ecall, noa_lane_set_cmd, noa_report_set_cmd, auto_main_beam_light_req,
                    ldw_lka_hpt_wrm_dsp_cmd, tja_ica_msg_req, elk_dsp_cmd_ipd, elk_sys_flt_sts, elk_sys_sts_ipd,
                    lka_sys_flt_sts, elk_warning, pilot_system_message, tlr_sys_sts, tlr_md_sw_dsp_cmd, tlr_warning,
                    tlr_msg_req
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/PP/CUS/FctDebug':
            header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            header_seq = msg.header.seq
            time_stamp = header_stamp
            frame_id = header_seq
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                AccDebug = msg.acc_debug
                acc_debug = AccDebug.acc_debug
                is_torq_resp_ok = AccDebug.is_torq_resp_ok
                is_acc_dec_resp_ok = AccDebug.is_acc_dec_resp_ok
                is_torq_resp = AccDebug.is_torq_resp
                is_acc_dec_resp = AccDebug.is_acc_dec_resp
                is_no_whl_brk_prs_too_long = AccDebug.is_no_whl_brk_prs_too_long
                is_epb_ok = AccDebug.is_epb_ok
                is_epb_applied = AccDebug.is_epb_applied
                is_sensor_ok = AccDebug.is_sensor_ok
                is_acc_ok = AccDebug.is_acc_ok
                acc_passive_error_code = AccDebug.acc_passive_error_code
                acc_active_error_code = AccDebug.acc_active_error_code
                is_all_doors_ok = AccDebug.is_all_doors_ok
                is_controller_ok = AccDebug.is_controller_ok
                is_brake_applied = AccDebug.is_brake_applied
                is_accel_actu_pos_ok = AccDebug.is_accel_actu_pos_ok
                is_drive_off_cond_true = AccDebug.is_drive_off_cond_true
                is_overtaking_control_valid = AccDebug.is_overtaking_control_valid
                is_overtaking_control_activated = AccDebug.is_overtaking_control_activated
                is_overtaking_control_too_long = AccDebug.is_overtaking_control_too_long
                acc_passive_cond_group_id = AccDebug.acc_passive_cond_group_id
                set_spd_chng_reason = AccDebug.set_spd_chng_reason
                target_state_of_active_trans = AccDebug.target_state_of_active_trans
                off_reason = AccDebug.off_reason

                LckDebugData = msg.lck_debug
                eps_degrade_b = LckDebugData.eps_degrade_b
                eps_ready_b = LckDebugData.eps_ready_b
                ego_lane_l = LckDebugData.ego_lane_l
                ego_lane_r = LckDebugData.ego_lane_r
                ego_lane = LckDebugData.ego_lane
                hands_on = LckDebugData.hands_on
                lat_ovrd = LckDebugData.lat_ovrd
                tja_on = LckDebugData.tja_on
                off_error_code = LckDebugData.off_error_code
                passive_error_code = LckDebugData.passive_error_code
                degrade_error_code = LckDebugData.degrade_error_code
                curv_ul_hold_dist = LckDebugData.curv_ul_hold_dist
                curv_ul_strt_idx = LckDebugData.curv_ul_strt_idx
                curv_ul_end_idx = LckDebugData.curv_ul_end_idx
                ego_curvature_ahead = LckDebugData.ego_curvature_ahead
                ego_curvature_idx = LckDebugData.ego_curvature_idx
                toll_idx = LckDebugData.toll_idx
                lane_merge_warning_flag = LckDebugData.lane_merge_warning_flag
                dist_to_merge_thrs = LckDebugData.dist_to_merge_thrs

                ZopDebugData = msg.zop_debug
                nav_on = ZopDebugData.nav_on
                hd_map_ok = ZopDebugData.hd_map_ok
                toll_ok = ZopDebugData.toll_ok
                zop_miss_merge_point_b = ZopDebugData.zop_miss_merge_point_b
                errcode = ZopDebugData.errcode

                IlcDebugData = msg.ilc_debug
                ilc_status = IlcDebugData.ilc_status
                tos_obs_l3_id = IlcDebugData.tos_obs_l3_id
                tos_obs_l2_id = IlcDebugData.tos_obs_l2_id
                tos_obs_r3_id = IlcDebugData.tos_obs_r3_id
                tos_obs_r2_id = IlcDebugData.tos_obs_r2_id
                tos_id_k1 = IlcDebugData.tos_id_k1
                ilc_left_b = IlcDebugData.ilc_left_b
                ilc_right_b = IlcDebugData.ilc_right_b
                ilc_offset_hold_id = IlcDebugData.ilc_offset_hold_id
                ilc_set_speed_kph = IlcDebugData.ilc_set_speed_kph
                ilc_tos_fl_cnt = IlcDebugData.tos_fl_cnt
                ilc_tos_fr_cnt = IlcDebugData.tos_fr_cnt
                ilc_tos_rl_cnt = IlcDebugData.tos_rl_cnt
                ilc_tos_rr_cnt = IlcDebugData.tos_rr_cnt
                ilc_offset_m = IlcDebugData.ilc_offset_m
                ilc_quit_flg = IlcDebugData.ilc_quit_flg

                HppDebugData = msg.hpp_debug
                brake_during_cruising_count = HppDebugData.brake_during_cruising_count
                hpp_suspend_status = HppDebugData.hpp_suspend_status
                hpp_abort_status = HppDebugData.hpp_abort_status
                hpp_abnormal_to_standby_status = HppDebugData.hpp_abnormal_to_standby_status
                hpp_to_apa_flag = HppDebugData.hpp_to_apa_flag
                turn_hpp_off_button = HppDebugData.turn_hpp_off_button
                start_or_return_cruising_btn = HppDebugData.start_or_return_cruising_btn
                target_slot_status = HppDebugData.target_slot_status
                apa_return_to_hpp_flag = HppDebugData.apa_return_to_hpp_flag
                brake_overtime_to_standby_during_cruising = HppDebugData.brake_overtime_to_standby_during_cruising
                hpp_reserve_place_one = HppDebugData.hpp_reserve_place_one
                hpp_reserve_place_two = HppDebugData.hpp_reserve_place_two
                hpp_reserve_place_three = HppDebugData.hpp_reserve_place_three
                hpp_reserve_place_four = HppDebugData.hpp_reserve_place_four
                hpp_reserve_place_five = HppDebugData.hpp_reserve_place_five
                hpp_reserve_place_six = HppDebugData.hpp_reserve_place_six
                hpp_reserve_place_seven = HppDebugData.hpp_reserve_place_seven

                ApaDebugData = msg.apa_debug
                replan_count = ApaDebugData.replan_count
                pi_button = ApaDebugData.pi_button
                po_button = ApaDebugData.po_button
                slot_selected = ApaDebugData.slot_selected
                start_parking_button = ApaDebugData.start_parking_button
                parking_cancel_button = ApaDebugData.parking_cancel_button
                parking_suspend_button = ApaDebugData.parking_suspend_button
                parking_continue_button = ApaDebugData.parking_continue_button
                is_car_moving = ApaDebugData.is_car_moving
                get_path_info_success = ApaDebugData.get_path_info_success
                parking_suspend_status = ApaDebugData.parking_suspend_status
                parking_abort_status = ApaDebugData.parking_abort_status
                slot_id = ApaDebugData.slot_id
                slot_id_previous = ApaDebugData.slot_id_previous
                current_path_step_num = ApaDebugData.current_path_step_num
                total_path_steps_num = ApaDebugData.total_path_steps_num
                apa_total_percent = ApaDebugData.apa_total_percent
                suspend_ctn_shift_time_count = ApaDebugData.suspend_ctn_shift_time_count
                gear_shift_count = ApaDebugData.gear_shift_count
                gear_first_shift_to_r_flag = ApaDebugData.gear_first_shift_to_r_flag
                abnormal_pi_to_standby_status = ApaDebugData.abnormal_pi_to_standby_status
                abnormal_po_to_standby_status = ApaDebugData.abnormal_po_to_standby_status
                pk_out_direc_status = ApaDebugData.pk_out_direc_status
                handshake_succeed = ApaDebugData.handshake_succeed
                pk_out_direc_details = ApaDebugData.pk_out_direc_details

                HmiDebug = msg.hmi_debug
                is_spd_adjust_lc_done_over_10s = HmiDebug.is_spd_adjust_lc_done_over_10s
                is_dir_lamp_switch_activated = HmiDebug.is_dir_lamp_switch_activated
                is_in_same_ramp = HmiDebug.is_in_same_ramp
                is_ego_crossed_laneline = HmiDebug.is_ego_crossed_laneline

                LdpDebugData = msg.ldp_debug
                ldp_state = LdpDebugData.ldp_state
                ldp_direction = LdpDebugData.ldp_direction
                ldp_departure_stage_l = LdpDebugData.ldp_departure_stage_l
                ldp_departure_stage_r = LdpDebugData.ldp_departure_stage_r
                ldp_front_left2border = LdpDebugData.front_left2border
                ldp_front_right2border = LdpDebugData.front_right2border
                ldp_inhibit_flag = LdpDebugData.ldp_inhibit_flag

                IcpDebugData = msg.icp_debug
                icp_state = IcpDebugData.icp_state
                avg_spd_left = IcpDebugData.avg_spd_left
                avg_spd_right = IcpDebugData.avg_spd_right
                avg_spd_host = IcpDebugData.avg_spd_host
                icp_off_error_code = IcpDebugData.icp_off_error_code
                icp_standby_error_code = IcpDebugData.icp_standby_error_code
                icp_leftlane_error_code = IcpDebugData.icp_leftlane_error_code
                icp_rightlane_error_code = IcpDebugData.icp_rightlane_error_code
                icp_passive_error_code = IcpDebugData.icp_passive_error_code
                tar_obj_idx = IcpDebugData.tar_obj_idx
                tar_obj_intention = IcpDebugData.tar_obj_intention
                tar_obj_heading = IcpDebugData.tar_obj_heading

                LdwDebugData = msg.ldw_debug
                ldw_state = LdwDebugData.ldw_state
                ldw_direction = LdwDebugData.ldw_direction
                ldw_departure_stage_l = LdwDebugData.ldw_departure_stage_l
                ldw_departure_stage_r = LdwDebugData.ldw_departure_stage_r
                ldw_front_left2border = LdwDebugData.front_left2border
                ldw_front_right2border = LdwDebugData.front_right2border
                ldw_inhibit_flag = LdwDebugData.ldw_inhibit_flag

                ElkDebugData = msg.elk_debug
                elk_state = ElkDebugData.elk_state
                elk_departure_stage_l = ElkDebugData.elk_departure_stage_l
                elk_departure_stage_r = ElkDebugData.elk_departure_stage_r
                front_left2border = ElkDebugData.front_left2border
                front_right2border = ElkDebugData.front_right2border
                front_left2edge_l = ElkDebugData.front_left2edge_l
                front_right2edge_r = ElkDebugData.front_right2edge_r
                roadedge_thrsld = ElkDebugData.roadedge_thrsld
                line_thrsld_left = ElkDebugData.line_thrsld_left
                line_thrsld_right = ElkDebugData.line_thrsld_right
                elk_overtaking_idx = ElkDebugData.elk_overtaking_idx
                elk_overtaking_ttc = ElkDebugData.elk_overtaking_ttc
                elk_basic_errcode = ElkDebugData.elk_basic_errcode
                elk_driver_errcode = ElkDebugData.elk_driver_errcode

                CommonDebugData = msg.common_debug
                tos_fl_cnt = CommonDebugData.tos_fl_cnt
                tos_fr_cnt = CommonDebugData.tos_fr_cnt
                tos_rl_cnt = CommonDebugData.tos_rl_cnt
                tos_rr_cnt = CommonDebugData.tos_rr_cnt
                lat_vel_ego = CommonDebugData.lat_vel_ego
                ego_lane_width = CommonDebugData.ego_lane_width

                NraDebugData = msg.nra_debug
                is_front_side_narrow = NraDebugData.is_front_side_narrow
                is_rear_side_narrow = NraDebugData.is_rear_side_narrow
                is_nra_close_btn_has_pressed = NraDebugData.is_nra_close_btn_has_pressed
                is_apa_hpp_active = NraDebugData.is_apa_hpp_active
                nra_abort_status = NraDebugData.nra_abort_status
                visual_assist_count = NraDebugData.visual_assist_count
                driver_close_visual_count = NraDebugData.driver_close_visual_count
                uss_dist_narrow_count = NraDebugData.uss_dist_narrow_count

                IhcDebugData = msg.ihc_debug
                ihc_state = IhcDebugData.ihc_state
                ihc_off_error_code = IhcDebugData.ihc_off_error_code
                ihc_standby_error_code = IhcDebugData.ihc_standby_error_code
                ihc_beam_off_error_code = IhcDebugData.ihc_beam_off_error_code
                ihc_ego_curvature_ahead = IhcDebugData.ego_curvature_ahead

                queue.put([
                    local_time, header_stamp, header_seq,
                    acc_debug, is_torq_resp_ok, is_acc_dec_resp_ok, is_torq_resp, is_acc_dec_resp,
                    is_no_whl_brk_prs_too_long, is_epb_ok, is_epb_applied, is_sensor_ok, is_acc_ok,
                    acc_passive_error_code, acc_active_error_code, is_all_doors_ok, is_controller_ok, is_brake_applied,
                    is_accel_actu_pos_ok, is_drive_off_cond_true, is_overtaking_control_valid,
                    is_overtaking_control_activated, is_overtaking_control_too_long, acc_passive_cond_group_id,
                    set_spd_chng_reason, target_state_of_active_trans, off_reason,
                    eps_degrade_b, eps_ready_b, ego_lane_l, ego_lane_r, ego_lane, hands_on, lat_ovrd, tja_on,
                    off_error_code, passive_error_code, degrade_error_code, curv_ul_hold_dist, curv_ul_strt_idx,
                    curv_ul_end_idx, ego_curvature_ahead, ego_curvature_idx, toll_idx, lane_merge_warning_flag,
                    dist_to_merge_thrs,
                    nav_on, hd_map_ok, toll_ok, zop_miss_merge_point_b, errcode,
                    ilc_status, tos_obs_l3_id, tos_obs_l2_id, tos_obs_r3_id, tos_obs_r2_id, tos_id_k1, ilc_left_b,
                    ilc_right_b, ilc_offset_hold_id, ilc_set_speed_kph, ilc_tos_fl_cnt, ilc_tos_fr_cnt, ilc_tos_rl_cnt,
                    ilc_tos_rr_cnt,
                    ilc_offset_m, ilc_quit_flg,
                    brake_during_cruising_count, hpp_suspend_status, hpp_abort_status, hpp_abnormal_to_standby_status,
                    hpp_to_apa_flag, turn_hpp_off_button, start_or_return_cruising_btn, target_slot_status,
                    apa_return_to_hpp_flag, brake_overtime_to_standby_during_cruising, hpp_reserve_place_one,
                    hpp_reserve_place_two, hpp_reserve_place_three, hpp_reserve_place_four, hpp_reserve_place_five,
                    hpp_reserve_place_six, hpp_reserve_place_seven,
                    replan_count, pi_button, po_button, slot_selected, start_parking_button, parking_cancel_button,
                    parking_suspend_button, parking_continue_button, is_car_moving, get_path_info_success,
                    parking_suspend_status, parking_abort_status, slot_id, slot_id_previous, current_path_step_num,
                    total_path_steps_num, apa_total_percent, suspend_ctn_shift_time_count, gear_shift_count,
                    gear_first_shift_to_r_flag, abnormal_pi_to_standby_status, abnormal_po_to_standby_status,
                    pk_out_direc_status, handshake_succeed, pk_out_direc_details,
                    is_spd_adjust_lc_done_over_10s, is_dir_lamp_switch_activated, is_in_same_ramp,
                    is_ego_crossed_laneline,
                    ldp_state, ldp_direction, ldp_departure_stage_l, ldp_departure_stage_r, ldp_front_left2border,
                    ldp_front_right2border, ldp_inhibit_flag,
                    icp_state, avg_spd_left, avg_spd_right, avg_spd_host, icp_off_error_code, icp_standby_error_code,
                    icp_leftlane_error_code, icp_rightlane_error_code, icp_passive_error_code, tar_obj_idx,
                    tar_obj_intention, tar_obj_heading,
                    ldw_state, ldw_direction, ldw_departure_stage_l, ldw_departure_stage_r, ldw_front_left2border,
                    ldw_front_right2border, ldw_inhibit_flag,
                    elk_state, elk_departure_stage_l, elk_departure_stage_r, front_left2border, front_right2border,
                    front_left2edge_l, front_right2edge_r, roadedge_thrsld, line_thrsld_left, line_thrsld_right,
                    elk_overtaking_idx, elk_overtaking_ttc, elk_basic_errcode, elk_driver_errcode,
                    tos_fl_cnt, tos_fr_cnt, tos_rl_cnt, tos_rr_cnt, lat_vel_ego, ego_lane_width,
                    is_front_side_narrow, is_rear_side_narrow, is_nra_close_btn_has_pressed, is_apa_hpp_active,
                    nra_abort_status, visual_assist_count, driver_close_visual_count, uss_dist_narrow_count,
                    ihc_state, ihc_off_error_code, ihc_standby_error_code, ihc_beam_off_error_code,
                    ihc_ego_curvature_ahead
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/SA/INSPVA':
            time_stamp = msg.time_stamp.time_stamp_ns / 1e9 + msg.time_stamp.time_stamp_s
            frame_id = 0
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq
                utc_time_stamp = msg.utc_time_us / 1e3
                latitude = msg.latitude
                longitude = msg.longitude
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.yaw

                queue.put([
                    local_time, time_stamp, header_stamp, header_seq, frame_id,
                    utc_time_stamp, latitude, longitude,
                    roll, pitch, yaw
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/VA/VehicleMotionIpd':
            time_stamp = msg.wheel_pulse_ts / 1e6
            frame_id = 0
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq
                vehicle_speed = msg.vehicle_speed / 3.6
                FL_wheel_speed = msg.driven_left_wheel_speed / 3.6
                FR_wheel_speed = msg.driven_right_wheel_speed / 3.6
                RL_wheel_speed = msg.undriven_left_wheel_speed / 3.6
                RR_wheel_speed = msg.undriven_right_wheel_speed / 3.6
                shaft_spd = msg.ept_input_shaft_spd
                front_wheel_angle = msg.front_wheel_angle
                rear_wheel_angle = msg.rear_wheel_angle

                queue.put([
                    local_time, time_stamp, header_stamp, header_seq, frame_id,
                    vehicle_speed, shaft_spd,
                    FL_wheel_speed, FR_wheel_speed, RL_wheel_speed, RR_wheel_speed,
                    front_wheel_angle, rear_wheel_angle,
                ])

                self.last_timestamp[topic] = time_stamp

        elif topic == '/Camera/FrontWide/H265':
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            time_stamp = sec + nanosec / 1e9
            frame_id = msg.header.frame_id
            format_ = msg.format
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)
            if time_stamp != self.last_timestamp[topic]:
                queue.put([
                    local_time, time_stamp, frame_id, format_
                ])
                print(f'{time_stamp:03f}', msg.data.shape, msg.data.tolist()[:20])

                self.last_timestamp[topic] = time_stamp

        # Qcaft
        elif topic in self.getTopics('qc_perception_msgs/msg/QcObstacles'):
            time_stamp = msg.timestamp / 1000
            frame_id = msg.frame_id
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(frame_id)

            if time_stamp != self.last_timestamp[topic]:
                header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                header_seq = msg.header.seq

                obstacles_num = msg.obstacles_num
                for i in range(obstacles_num):
                    obstacle_data = msg.obstacles[i]
                    measurement_type = obstacle_data.measurement_type
                    if measurement_type == 1:
                        obstacle_type = 1
                        sub_type = 1
                    elif measurement_type == 3:
                        obstacle_type = 2
                        sub_type = -1
                    elif measurement_type == 4:
                        obstacle_type = 18
                        sub_type = -1
                    elif measurement_type == 14:
                        obstacle_type = 1
                        sub_type = 9
                    elif measurement_type == 27:
                        obstacle_type = 1
                        sub_type = 4
                    else:
                        continue

                    measurement = obstacle_data.obstacle_data
                    obstacle_id = measurement.track_id
                    x = measurement.pos_x
                    y = measurement.pos_y
                    z = measurement.pos_z
                    yaw = measurement.heading
                    age = measurement.tracking_period
                    vx = measurement.vel_x
                    vy = measurement.vel_y
                    length = measurement.length
                    width = measurement.width
                    height = measurement.height
                    conf = measurement.existence_confidence
                    track_status = measurement.track_status

                    queue.put([
                        local_time, time_stamp, header_stamp, header_seq, frame_id,
                        obstacle_id, obstacle_type, conf, sub_type,
                        x, y, z, vx, vy, yaw, length, width, height, age, track_status,
                    ])

                    self.last_timestamp[topic] = time_stamp

        elif topic in self.getTopics('qc_perception_msgs/msg/QcLines'):
            pass

        elif topic in self.getTopics('qc_perception_msgs/msg/QcPose'):
            header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            header_seq = msg.header.seq
            time_stamp = msg.header.timestamp
            self.time_saver[topic].append(time_stamp)
            self.frame_id_saver[topic].append(header_seq)
            if time_stamp != self.last_timestamp[topic]:
                roll = msg.roll
                pitch = msg.pitch
                yaw = msg.yaw
                vx = msg.vel_smooth.x
                vy = msg.vel_smooth.y
                vel = msg.speed
                trip_distance = msg.trip_distance

                queue.put([
                    local_time, time_stamp, header_stamp, header_seq,
                    roll, pitch, yaw, vx, vy, vel, trip_distance,
                ])

                self.last_timestamp[topic] = time_stamp


class MsgSaver:

    def __init__(self, topic, folder, tag, queue, data_columns):
        self.save_process = None
        self.topic = topic.replace('/', '')
        self.data_tag = tag
        self.data_folder = folder
        self.datas = []
        self.data_file_index = 0
        self.data_row_per_file = 0
        self.data_queue = queue
        self.data_file = None
        self.datas_columns = data_columns
        while not self.data_queue.empty():
            self.data_queue.get()
        self.process_start = mp.Event()
        self.process_end = mp.Event()

    def create_data_file(self):
        self.data_file_index += 1
        data_file = os.path.join(self.data_folder, '{:s}_{:s}_{:0>3d}_data.csv'
                                 .format(self.topic, self.data_tag, self.data_file_index))
        with open(data_file, 'w', encoding='utf8', newline='') as csvfile:
            myWriter = csv.writer(csvfile)
            myWriter.writerow(self.datas_columns)
        self.data_file = data_file
        print('{:s} starts saving data to {:s}'.format(self.topic, self.data_file))
        self.data_row_per_file = 0

    def save_data_once(self):
        with open(self.data_file, 'a+', encoding='utf8', newline='') as csvfile:
            myWriter = csv.writer(csvfile)
            for csv_row in self.datas:
                myWriter.writerow(csv_row)
        self.datas = []

    def save_data(self):
        self.process_start.wait()
        while self.process_start.is_set():
            while self.data_queue.empty():
                if not self.process_start.is_set():
                    break
                time.sleep(0.05)
            while not self.data_queue.empty():
                self.datas.append(self.data_queue.get())
            if len(self.datas) > 100:
                self.data_row_per_file += len(self.datas)
                if os.path.getsize(self.data_file) > 5242880:
                    self.create_data_file()
                self.save_data_once()
        self.save_data_once()
        self.process_end.set()

    def start_save_process(self):
        self.create_data_file()
        self.save_process = mp.Process(target=self.save_data)
        self.save_process.start()
        self.process_start.set()
        self.process_end.clear()

    def terminate_save_process(self):
        self.process_start.clear()
        self.process_end.wait()
        self.save_process.terminate()
        self.save_process.join()
        print('{:s} ends saving data'.format(self.topic))


class Ros2bagPretreatment:

    def __init__(self, folder):
        self.folder = folder
        time_offset = []
        time_offset_group = {}
        self.injection_folder = {}
        for injection_id in os.listdir(folder):
            time_offset_group[injection_id] = []
            injection_folder = os.path.join(folder, injection_id)
            if not os.path.isdir(injection_folder):
                continue

            hz_files = []
            raw_data_folder = os.path.join(folder, injection_id, 'RawData')
            self.injection_folder[injection_id] = raw_data_folder
            fs = glob.glob(os.path.join(raw_data_folder, 'VAObstacles_*_hz.csv'))
            if len(fs):
                hz_files.append(fs[0])
            fs = glob.glob(os.path.join(raw_data_folder, 'VALines_*_hz.csv'))
            if len(fs):
                hz_files.append(fs[0])
            fs = glob.glob(os.path.join(raw_data_folder, 'VAObjects_*_hz.csv'))
            if len(fs):
                hz_files.append(fs[0])

            for hz_file in hz_files:
                hz_data = pd.read_csv(hz_file, index_col=False)
                if len(hz_data) and '_0.00_' in hz_file:
                    hz_data['time_diff'] = hz_data['time_stamp'].diff(1).fillna(0)
                    highest_freq_value = hz_data['time_diff'].round(4).value_counts().idxmax()
                    # if hz_data['time_diff'].max() > 1700000000:
                    #     offset = hz_data['time_diff'].max() - highest_freq_value
                    #     print(offset)
                    offset = 0
                    if hz_data['time_diff'].min() < -1700000000:
                        offset = highest_freq_value - hz_data['time_diff'].min()
                        time_offset_group[injection_id].append(offset)
                        time_offset.append(offset)
                    print(injection_id, os.path.basename(hz_file), highest_freq_value, offset)

                    # 删除不正常的时间戳重新保存
                    hz_data.drop('time_diff', axis=1, inplace=True)
                    hz_data = hz_data[hz_data['time_stamp'] > 1700000000]
                    hz_file_basename = os.path.basename(hz_file)
                    new_file_name = os.path.join(raw_data_folder, '{:s}_{:.2f}_{:s}'.format(
                        hz_file_basename.split('_')[0],
                        1 / highest_freq_value,
                        hz_file_basename.split('_')[-1]))
                    print(hz_file)
                    print(new_file_name)
                    hz_data.to_csv(new_file_name, index=False)
                    os.remove(hz_file)

        self.time_offset_group = {}
        for in_id, t_o in time_offset_group.items():
            if len(t_o):
                self.time_offset_group[in_id] = round(np.mean(t_o), 3)
            else:
                self.time_offset_group[in_id] = round(np.mean(time_offset), 3)

        self.change_ego_timestamp()
        self.analyze_raw_data()

    def change_ego_timestamp(self):
        for injection_id, raw_data in self.injection_folder.items():
            ego_file = glob.glob(os.path.join(raw_data, 'PIEGEgoMotionInfo*csv'))
            for f in ego_file:
                data = pd.read_csv(f, index_col=False).iloc[5:]
                data['time_stamp'] += self.time_offset_group[injection_id]
                data.to_csv(f, index=False)
    
    def analyze_raw_data(self):
        rows = []
        index = []
        columns = []
        for injection_id, raw_data in self.injection_folder.items():
            if not os.path.exists(os.path.join(raw_data, 'TestTopicInfo.yaml')):
                continue

            with open(os.path.join(raw_data, 'TestTopicInfo.yaml')) as f:
                test_topic = yaml.load(f, Loader=yaml.FullLoader)

            row = []
            columns = []
            valid_count = 0
            i = 0
            for topic in test_topic['topics_for_parser']:
                topic_tag = topic.replace('/', '')
                hz_data = pd.read_csv(glob.glob(os.path.join(raw_data, f'{topic_tag}*hz.csv'))[0],
                                      index_col=False)
                row.append('{:d}/{:d}/{:.3f}-{:.3f}'.format(
                    len(hz_data), hz_data['count'].sum(), hz_data['time_stamp'].min(), hz_data['time_stamp'].max()))

                if len(hz_data) > 500:
                    valid_count += 1
                columns.append(topic)
                i += 1

            if valid_count < i:
                valid_flag = 0
            else:
                valid_flag = 1
            # 暂时
            valid_flag = 1
            row.append(valid_flag)
            index.append(injection_id)
            rows.append(row)

        if columns:
            res = pd.DataFrame(rows, columns=columns + ['valid'], index=index)
            res.to_csv(os.path.join(self.folder, 'topic_output_statistics.csv'))


class Ros2BagClip:

    def __init__(self, workspace):
        self.last_timestamp = None
        self.frame_id_saver = None
        self.time_saver = None
        self.install_folder = os.path.join(workspace, 'install')
        self.typestore = get_typestore(Stores.LATEST)
        msg_list = []
        for root, dirs, files in os.walk(self.install_folder):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        # for root, dirs, files in os.walk('/opt/ros/rolling'):
        #     for f in files:
        #         ff = os.path.join(root, f)
        #         if 'share' in ff and '.msg' in ff and 'detail' not in ff:
        #             msg_list.append(ff)

        for pathstr in msg_list:
            msg_path = Path(pathstr)
            msg_def = msg_path.read_text(encoding='utf-8')
            temp = get_types_from_msg(msg_def, self.getMsgType(msg_path))
            self.typestore.register(temp)

    def getMsgType(self, path: Path) -> str:
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    def cutRosbag(self, src, dst, topic_list, time_range) -> None:
        with Reader(src) as reader, Writer(dst) as writer:
            conn_map = {}
            for conn in reader.connections:
                if conn.topic not in topic_list:
                    continue

                conn_map[conn.id] = writer.add_connection(
                    topic=conn.topic,
                    msgtype=conn.msgtype,
                    typestore=self.typestore,
                )

            for conn, timestamp, data in reader.messages(connections=list(conn_map.values())):
                if time_range[0] < timestamp / 1e9 <= time_range[1]:
                    writer.write(conn_map[conn.id], timestamp, data)


if __name__ == "__main__":
    workspace = '/home/byj/ZONE/TestProject/ParkingDebug/03_Workspace'
    ros2bag_path = '/home/byj/ZONE/TestProject/ParkingDebug/01_Prediction/20250324_144918_n000001/rosbag2_2025_04_24-15_56_18'
    # ros2bag_path = '/home/byj/ZONE/debug/rosbag2_2025_01_22-13_14_41'
    folder = '/home/byj/ZONE/TestProject/ParkingDebug/01_Prediction/20250324_144918_n000001/RawData'
    ES39_topic_list = [
            # '/PI/EG/EgoMotionInfo',
            '/VA/VehicleMotionIpd',
            # '/VA/BevObstaclesDet',
            # '/VA/FrontWideObstacles2dDet',
            # '/VA/BackViewObstacles2dDet',
            # '/VA/Lines',
            # '/VA/Obstacles',
            # '/PI/FS/ObjTracksHorizon',
            # '/PK/DR/Result',
            '/SA/INSPVA',
            # '/Camera/FrontWide/H265',
            '/PK/PER/VisionSlotDecodingList',
            # '/VA/QC/BEVObstaclesTracks',
            # '/VA/QC/MonoObstaclesTracks',
            # '/VA/QC/FsObstacles',
            # '/VA/QC/Lines',
            # '/VA/QC/Objects',
            # '/VA/QC/Pose',
    ]

    RBP = Ros2BagParser(workspace)
    RBP.getMsgInfo(ros2bag_path, ES39_topic_list, folder, '20250324_144918_n000001')