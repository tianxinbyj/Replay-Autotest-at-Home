import json
import os
import sys

import pandas as pd
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QHeaderView, QAbstractItemView, QTableWidgetItem
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtWidgets import QPushButton, QWidget

from Envs.Master.Uiforms.TestResultToolkit import Ui_test_result_toolkit


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


class ResultToolkit(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_test_result_toolkit()
        self.ui.setupUi(self)
        self.setFixedSize(self.width(), self.height())
        self.setWindowFlags(Qt.WindowCloseButtonHint)

        self.last_folder = None
        self.data_folder_list = {}
        self.data = None
        self.specified_items = []
        self.tag_value = {}
        self.button = {}
        self.combox = {}

        self.ui.add_data_folder.clicked.connect(self.add_data_folder)
        self.ui.delete_data_folder.clicked.connect(self.delete_data_folder)

    def add_data_folder(self):
        initial_dir = self.last_folder if self.last_folder else ''
        folder_name = QFileDialog.getExistingDirectory(self, '选择文件夹', initial_dir)
        if not (folder_name and os.path.exists(os.path.join(folder_name, 'output_result.csv'))):
            return

        product, version = None, None
        for f in os.listdir(folder_name):
            if 'json' in f:
                json_file = os.path.join(folder_name, f)
                with open(json_file, 'r') as file:
                    data = json.load(file)
                    product = data['Product']
                    version = data['Version']
                    break

        if product and version:
            self.last_folder = folder_name
            key = f'{product} -> {version}: {folder_name}'
            self.data_folder_list[key] = folder_name
            self.ui.data_folder_list.addItem(key)
            self.get_data()
            self.specify_tag_value()

    def delete_data_folder(self):
        current_item = self.ui.data_folder_list.currentItem()
        if current_item:
            key = current_item.text()
            row = self.ui.data_folder_list.row(current_item)
            self.ui.data_folder_list.takeItem(row)
            del self.data_folder_list[key]
            self.get_data()
            self.specify_tag_value()

    def get_data(self):
        # 清空标签内容
        for child in self.ui.filter_group.findChildren(QWidget):
            child.setParent(None)
            child.deleteLater()

        tag_info = {}
        test_result = []
        self.button = {}
        self.combox = {}
        for data_folder in self.data_folder_list.values():
            test_result.append(pd.read_csv(os.path.join(data_folder, 'output_result.csv'), index_col=None))

        if not len(test_result):
            return

        self.data = pd.concat(test_result)
        for col in self.data.columns:
            if col not in ['index', 'sample_count', 'result']:
                tag_info[col] = self.data[col].unique().tolist()

        # 重新增加内容
        row = 0
        for key, values in tag_info.items():
            button = QPushButton(key)
            button.setCheckable(True)
            button.clicked.connect(self.specify_item)
            self.button[key] = button
            combo = QComboBox()
            combo.addItems(values)
            combo.currentIndexChanged.connect(self.specify_tag_value)
            self.combox[key] = combo

            self.ui.filter_layout.addWidget(button, row, 0)
            self.ui.filter_layout.addWidget(combo, row, 1)
            row += 1

    def specify_tag_value(self):
        self.tag_value = {}
        for key, combox in self.combox.items():
            self.tag_value[key] = combox.currentText()

        self.filter_data()

    def specify_item(self):
        self.specified_items = []
        for key, button in self.button.items():
            if key == 'metric':
                button.setChecked(False)
            else:
                if button.isChecked():
                    button.setStyleSheet("background-color: limegreen;")
                    self.specified_items.append(button.text())
                    self.combox[key].setEnabled(False)
                else:
                    button.setStyleSheet("background-color: white;")
                    self.combox[key].setEnabled(True)

        self.filter_data()

    def filter_data(self):
        data = self.data
        same_condition = []
        ind_condition = []
        for key, value in self.tag_value.items():
            if key not in self.specified_items:
                data = data[data[key] == value]
                if key != 'metric':
                    same_condition.append(value)
            else:
                ind_condition.append(key)

        self.ui.data_count.setText((str(len(data))))
        self.ui.result_des.clear()
        if 'metric' in self.combox:
            self.ui.result_des.addItem(f'TestMetric -> {self.combox["metric"].currentText()}')
        self.ui.result_des.addItem(f'Controlled Condition -> {",".join(same_condition)}')
        self.ui.result_des.addItem(f'Independent Condition -> {",".join(ind_condition)}')

        if len(data):
            columns = ind_condition + list(json.loads(data.iloc[0]['result']).keys())
            rows = []
            for idx, row in data.iterrows():
                rows.append(
                    [row[col] for col in ind_condition] + list(json.loads(row['result']).values())
                )
            show_data = pd.DataFrame(rows, columns=columns)
            edit_table(self.ui.result_table, show_data, columns)


if __name__ == '__main__':
    app = QApplication([])
    jko = ResultToolkit()
    jko.show()
    sys.exit(app.exec_())