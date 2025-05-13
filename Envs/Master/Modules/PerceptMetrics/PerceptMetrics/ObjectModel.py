"""  
@Author: BU YUJUN
@Date: 2025/4/2 09:51  
"""
import numpy as np


def convert_slot_pts_to_ego_pts(slots_pts: np.array, ctr_to_wheel_arc: float = 1.39698315):
    # slot_pts 维度(n, 4, 2) n为车位个数,  4为车位4个角点,  2为像素值u, v
    W, H = 1088, 1216  # AVM图像原始大小为1088*1216 车位角点像素值也是在这个尺度上
    center_pix = np.array([[[W / 2, H / 2]]], dtype=np.float32)  # 算出图像中心的像素主
    slots_pts -= center_pix  # 将AVM图像坐标系原点移动到自车中心
    slots_pts = -slots_pts[:, :, ::-1]  # 颠倒uv以及其正负 配合自车坐标系定义(车辆后轴中心对地投影点为原点 车辆前进方向为X轴 指向车身左侧为Y轴 Z轴指向天空方向)
    slots_pts *= 0.02  # 尺度转换:AVM图上 1pixel 代表 自车坐标系中0.02m
    slots_pts[:, :, 0] += ctr_to_wheel_arc  # 将原点由自车中心移动到后轴中心
    return slots_pts


def convert_AVM_to_ego(u, v):
    W, H = 704, 800
    if u == v == -1 or u == v == 0:
        return 0, 0
    x = (H / 2 + 47.19 - v) * 0.03
    y = (W / 2  - u) * 0.03
    return x, y

def convert_AVM_to_ego_gt(u, v, ctr_to_wheel_arc: float = 1.39698315):
    W, H = 1088, 1216
    if u == v == -1 or u == v == 0:
        return 0, 0
    x = (H / 2 - v) * 0.02 + ctr_to_wheel_arc
    y = (W / 2 - u) * 0.02
    return x, y


class Slot:

    def __init__(self, slot_type, pt_0_x, pt_0_y, pt_1_x, pt_1_y, pt_2_x, pt_2_y, pt_3_x, pt_3_y,
                 stopper_0_x=0, stopper_0_y=0, stopper_1_x=0, stopper_1_y=0):
        self.pt = {
            0: np.array([pt_0_x, pt_0_y]),
            1: np.array([pt_1_x, pt_1_y]),
            2: np.array([pt_2_x, pt_2_y]),
            3: np.array([pt_3_x, pt_3_y]),
        }

        self.stopper_flag = 1
        if stopper_0_x + stopper_0_y + stopper_1_x + stopper_1_y == 0:
            self.stopper_flag = 0
        self.stp_pt = {
            0 : np.array([stopper_0_x, stopper_0_y]),
            1 : np.array([stopper_1_x, stopper_1_y]),
        }
        self.stp_center = (self.stp_pt[0] + self.stp_pt[1]) / 2

        self.in_border_pt = (self.pt[0] + self.pt[1]) / 2
        self.end_border_pt = (self.pt[2] + self.pt[3]) / 2
        self.border03_pt = (self.pt[0] + self.pt[3]) / 2
        self.border12_pt = (self.pt[1] + self.pt[2]) / 2

        self.d = {i: np.linalg.norm(self.pt[i]) for i in range(4)}
        self.in_border_distance = np.linalg.norm(self.in_border_pt)
        self.end_border_distance = np.linalg.norm(self.end_border_pt)
        self.in_border_length = np.linalg.norm(self.pt[0] - self.pt[1])
        self.slot_length = (np.linalg.norm(self.pt[0] - self.pt[2]) + np.linalg.norm(self.pt[1] - self.pt[3])) / 2

        if slot_type in [1, 3]:
            self.slot_heading = np.arccos(
                np.dot(self.in_border_pt - self.end_border_pt, np.array([1, 0])) / np.linalg.norm(
                    self.in_border_pt - self.end_border_pt))
        else:
            self.slot_heading = np.arccos(
                np.dot(self.border03_pt - self.border12_pt, np.array([1, 0])) / np.linalg.norm(
                    self.border03_pt - self.border12_pt))

        if not self.stopper_flag:
            self.stopper_depth = 0
        else:
            if slot_type in [1, 3]:
                self.stopper_depth = np.linalg.norm(self.stp_center - self.in_border_pt)
            else:
                self.stopper_depth = np.linalg.norm(self.stp_center - self.border03_pt)

    def get_slot_distance(self):

        def check_point_in_quad(point, quad):
            """
            判断一个点是否在四边形内。
            参数:
            point -- 要判断的点 (x, y)
            quad -- 四边形的四个顶点 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
            """

            def cross_product(v1, v2):
                """计算两个向量的叉乘。"""
                return v1[0] * v2[1] - v1[1] * v2[0]

            def vector(p1, p2):
                """从点p1到点p2的向量"""
                return p2[0] - p1[0], p2[1] - p1[1]

            # 确保四边形顶点按顺序（顺时针或逆时针）
            quad.append(quad[0])  # 将第一个点加到最后，以形成闭合循环

            signs = []
            for ijk in range(4):
                edge_vec = vector(quad[ijk], quad[ijk + 1])
                point_vec = vector(quad[ijk], point)
                cross_prod = cross_product(edge_vec, point_vec)
                signs.append(cross_prod)

            # 判断所有叉乘符号是否一致（全部大于0或全部小于0）
            return all(s > 0 for s in signs) or all(s < 0 for s in signs)

        point = [-1.053, 0]
        quad = [[self.pt[0][0], self.pt[0][1]], [self.pt[1][0], self.pt[1][1]], [self.pt[2][0], self.pt[2][1]],
                [self.pt[3][0], self.pt[3][1]]]
        if check_point_in_quad(point, quad):
            sign = -1
        else:
            sign = 1

        return sign * np.linalg.norm(np.array(point) - self.in_border_pt)


if __name__ == '__main__':
    data = np.array([
        [[227.427551269531, 331.182525634766], [155.377395629883, 305.9287109375], [100.616882324219, 468.429077148438], [171.55192565918, 492.236572265625]],
        [[227.427551269531, 331.182525634766], [155.377395629883, 305.9287109375], [100.616882324219, 468.429077148438], [171.55192565918, 492.236572265625]],
    ])

    print(convert_slot_pts_to_ego_pts(data))

    u, v = 227.427551269531, 331.182525634766
    print(convert_AVM_to_ego(u, v))