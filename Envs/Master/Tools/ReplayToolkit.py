#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : ReplayToolkit.py
# @Time     : 12/1/24 11:03 PM
# @Author   : Bu Yujun

import functools
import os
import signal
import sys

import pandas as pd
import yaml
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QComboBox, QLineEdit
from PyQt5.QtWidgets import QHeaderView, QTableWidgetItem, QAbstractItemView, QApplication
from PyQt5.QtWidgets import QMainWindow, QFileDialog, QMessageBox

from Envs.Master.Uiforms.ReplayToolkit import Ui_replay_toolkit
from Utils.Libs import project_path

import warnings
warnings.filterwarnings("ignore")


def timeout(sec):
    """
    timeout decorator
    :param sec: function raise TimeoutError after ? seconds
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapped_func(*args, **kwargs):

            def _handle_timeout(signum, frame):
                err_msg = f'Function {func.__name__} timed out after {sec} seconds'
                raise TimeoutError(err_msg)

            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(sec)
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
            return result

        return wrapped_func

    return decorator


def edit_table(table, data_df, header):
    table.setRowCount(len(data_df))
    table.setColumnCount(len(header))
    table.setHorizontalHeaderLabels(header)
    table.horizontalHeader().setDefaultAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
    table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
    table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
    table.setEditTriggers(QAbstractItemView.NoEditTriggers)
    for i, row in data_df.iterrows():
        for j, item in enumerate(row):
            content = QTableWidgetItem(str(item))
            content.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            table.setItem(i, j, content)


class ReplayToolKit(QMainWindow):
    progress_signal = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_replay_toolkit()
        self.ui.setupUi(self)
        self.setFixedSize(self.width(), self.height())
        self.setWindowFlags(Qt.WindowCloseButtonHint)

        # 变量初始化
        self.replay_config = None

        # 界面初始化
        self.config_yaml = os.path.join(project_path, 'Envs', 'Master', 'Tools', 'ReplayToolkit.yaml')
        self.load_config()
        self.ui.toolkit_tab.setCurrentIndex(1)

        # self.jira_toolkit = JiraToolGUI()
        # self.ui.jira_container.setWidget(self.jira_toolkit)
        # self.ui.jira_container.setWidgetResizable(True)
        # self.jira_toolkit.setMinimumSize(548, 909)

        # 文件夹设置
        self.ui.browse_bug.clicked.connect(self.browse_bug)

    def browse_path(self):
        folder = QFileDialog.getExistingDirectory(self, '选择文件夹', '/home')
        if not os.path.exists(folder):
            QMessageBox.warning(self, 'warning', '请选择有效的地址',
                                QMessageBox.Ok, QMessageBox.Ok)
            return
        path_name = self.sender().objectName().split('_')[1] + '_path'
        exec('self.ui.{:s}.setText(folder)'.format(path_name))
        self.replay_config[path_name] = folder
        self.save_config()

    def browse_bug(self):
        fileName, filetype = QFileDialog.getOpenFileName(self, "选取文件", '',
                                                         "All Files (*);;Text Files (*.txt)")
        if not os.path.exists(fileName):
            QMessageBox.warning(self, 'warning', '请选择有效的地址',
                                QMessageBox.Ok, QMessageBox.Ok)
            return
        self.replay_config['bug_path'] = fileName
        self.save_config()
        self.load_bug_table(fileName)

    def load_bug_table(self, bug_path):

        def update_bug_table():
            idx = int(self.sender().objectName().split('_')[1])
            bug_table_data.at[idx, 'is_valid'] = bug_cbx_dict[idx].currentIndex()
            bug_table_data.to_csv(bug_path, index=False, encoding='utf_8_sig')

        def open_bug_report(item):
            idx = item.row()
            col = item.column()
            report_path = bug_table_data.at[idx, 'attachment_path']
            if col == 5:
                cmd = f'evince "{report_path}"'
                os.system(
                    f'''
                gnome-terminal -- bash -c '{cmd}'
                    '''
                )
            # elif col == 4:
            #     tmp = os.path.dirname(os.path.dirname(report_path))
            #     big_topic = os.path.basename(tmp)
            #     dir = os.path.dirname(os.path.dirname(tmp))
            #     csv_path = os.path.join(dir, 'match-wide_conf-0', 'Data', big_topic, 'corresponding_data.csv')
            #     cmd = f'libreoffice --calc "{csv_path}"'
            #     os.system(
            #         f'''
            #     gnome-terminal -- bash -c '{cmd}'
            #         '''
            #     )

        def save_bug():
            for idx, le in bug_le_dict.items():
                bug_table_data.at[idx, 'comment'] = le.text()
            bug_table_data.to_csv(bug_path, index=False, encoding='utf_8_sig')

        if not os.path.exists(bug_path):
            print('bug_path 不存在')
            return

        self.ui.bug_path.setText(self.replay_config['bug_path'])
        self.ui.bug_table.clear()
        try:
            self.ui.bug_table.itemDoubleClicked.disconnect()
            self.ui.save_bug.clicked.disconnect()
        except:
            pass

        self.ui.save_bug.clicked.connect(save_bug)
        self.ui.bug_table.itemDoubleClicked.connect(open_bug_report)
        bug_table_data = pd.read_csv(bug_path, index_col=None)
        if 'comment' not in bug_table_data:
            bug_table_data['comment'] = ''
        col = ['scenario_type', 'topic', 'bug_type', 'target_type', 'gt_target_id', 'scenario_id', 'uuid', 'is_valid', 'comment']
        data = bug_table_data[col]
        data['gt_target_id'] = data['gt_target_id'].astype(int)
        header = ['场景类型', 'topic', '异常类型', '目标类型', '真值id', '场景编号', '报告编号', '问题归属', '备注']
        edit_table(self.ui.bug_table, data, header)
        self.ui.bug_table.setColumnWidth(len(header) - 1, 200)

        # 增加combox
        bug_cbx_dict = {}
        bug_le_dict = {}
        vld_list = ['未定义', '感知', '真值', '其他']
        for i, row in data.iterrows():
            cbx = QComboBox(self)
            cbx.addItems(vld_list)
            cbx.setCurrentIndex(row['is_valid'])
            cbx.setObjectName(f'cbx_{i}')
            cbx.currentTextChanged.connect(update_bug_table)
            self.ui.bug_table.setCellWidget(i, len(header) - 2, cbx)
            bug_cbx_dict[i] = cbx

            le = QLineEdit(self)
            if str(row['comment']) != 'nan':
                le.setText(str(row['comment']))
            self.ui.bug_table.setCellWidget(i, len(header) - 1, le)
            bug_le_dict[i] = le

    def load_config(self):
        if not os.path.exists(self.config_yaml):
            self.replay_config = {}
            with open(self.config_yaml, 'w') as f:
                yaml.dump({}, f, default_flow_style=False, allow_unicode=True)
        else:
            with open(self.config_yaml, 'r') as f:
                self.replay_config = yaml.safe_load(f)
            if 'bug_path' in self.replay_config and os.path.exists(self.replay_config['bug_path']):
                self.load_bug_table(self.replay_config['bug_path'])


    def save_config(self):
        with open(self.config_yaml, 'w') as f:
            yaml.dump(self.replay_config, f, default_flow_style=False, allow_unicode=True)


if __name__ == '__main__':
    app = QApplication([])
    jko = ReplayToolKit()
    jko.show()
    sys.exit(app.exec_())