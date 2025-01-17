#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2023/9/27 上午10:00
# @Author  : Cen Zhengfeng

import os
from copy import deepcopy
from functools import partial

import requests
import yaml
import sys
from PyQt5.QtCore import QDir, QThread, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QWidget, QMessageBox, QFileDialog, QLabel, QLineEdit, QPushButton, QComboBox, \
    QListWidget

from Envs.Master.Uiforms.jira_toolkit import Ui_Form
from Envs.Master.Modules.JiraToolkit import get_user, write_yaml_config, data, main, check_field_meta
from enum import Enum


class Goal(Enum):
    NOT_CHECKED = 0
    PERCEPTION = 1
    GROUND_TRUTH = 2
    ELSE = 3


jira_option_name_mapping = {
    "issuetype": "问题类型",
    "customfield_11568": "子系统模块",
    "customfield_11425": "问题责任方",
    "customfield_11567": "系统功能",
    "customfield_11008": "Category",
    "customfield_10689": "Occurrence",
    "customfield_10684": "Severity",
    "customfield_10200": "VehicleType",
    "customfield_10687": "Test Level",
    "versions": "影响版本",
    "priority": "Priority",
    "labels": "Labels"
}

variable_mapping = {
    "Assignee": "assignee_template",
    "Task Template": "task_template",
    "Bug Template": "bug_template",
    "Source": "source_template"
}


class MsgBoxWorkerThread(QThread):
    finished = pyqtSignal(dict, dict, dict, dict)

    def __init__(self):
        super().__init__()

    def run(self):
        # Load Jira options via checking field meta of the field type.
        bug_option_1j5 = check_field_meta(product_type='1j5', issue_type_id=10403)
        task_option_1j5 = check_field_meta(product_type='1j5', issue_type_id=10401)
        bug_option_2j5 = check_field_meta(product_type='2j5', issue_type_id=10403)
        task_option_2j5 = check_field_meta(product_type='2j5', issue_type_id=10401)
        self.finished.emit(bug_option_1j5, task_option_1j5, bug_option_2j5, task_option_2j5)


class MainWorkerThread(QThread):
    error_occurred = pyqtSignal(Exception)

    def __init__(self, data):
        super().__init__()
        self.data = data

    def run(self):
        try:
            main(self.data)
        except Exception as e:
            self.error_occurred.emit(e)


class JiraToolGUI(QWidget, Ui_Form):
    initial_data = deepcopy(data)  # Keep clean

    def __init__(self):
        self.jira_task_option_mapping = None
        self.jira_bug_option_mapping = None
        self.jira_bug_option_mapping_1j5 = None
        self.jira_task_option_mapping_1j5 = None
        self.jira_bug_option_mapping_2j5 = None
        self.jira_task_option_mapping_2j5 = None
        if os.path.exists('/home/zhangliwei01/ZONE/PythonProject/Replay-Autotest-at-Home/Docs/Resources/token/jira_config.yaml'):
            # Load data from the YAML file
            with open('/home/zhangliwei01/ZONE/PythonProject/Replay-Autotest-at-Home/Docs/Resources/token/jira_config.yaml', 'r') as yaml_file:
                config_data = yaml.load(yaml_file, Loader=yaml.FullLoader)

            # Update the 'data' dictionary with values from the YAML file
            data.update(config_data)

        self.value_labels = None
        super().__init__()
        self.setupUi(self)
        try:
            jira_name, _ = get_user()
        except ValueError as e:
            print(e)
            exit(-1)
        self.jira_username.setText(jira_name)

        try:
            confluence_name, _ = get_user('/home/zhangliwei01/ZONE/PythonProject/Replay-Autotest-at-Home/Docs/Resources/token/confluence_token.txt')
        except ValueError as e:
            print(e)
            exit(-1)
        self.confluence_name = confluence_name
        self.selected_files = []

        self.reset_config.clicked.connect(self.reset_data)
        self.radioButton.clicked.connect(self.switch_product)
        self.radioButton_2.clicked.connect(self.switch_product)
        self.confluence_true.clicked.connect(self.switch_confluence)
        self.confluence_false.clicked.connect(self.switch_confluence)
        self.goal_perception.clicked.connect(self.switch_goal)
        self.goal_ground_truth.clicked.connect(self.switch_goal)
        self.create_issues.clicked.connect(self.show_confirmation_dialog)

        self.product = data['jira_type']
        self.use_confluence = data['use_confluence']
        self.goal = data['goal']

        self.load_options.clicked.connect(lambda: self.load_options_data())
        self.gen_config.clicked.connect(lambda: write_yaml_config(data))
        self.terminate.clicked.connect(lambda: sys.exit())

        if self.source_box:
            self.list_widget = QListWidget()
            self.source_box.layout().addWidget(self.list_widget, 0, 0, 2, 1)
            browse_button = QPushButton('Import')
            delete_button = QPushButton('Delete')
            self.source_box.layout().addWidget(browse_button, 0, 1)
            self.source_box.layout().addWidget(delete_button, 1, 1)
            browse_button.clicked.connect(lambda: self.import_items(self.list_widget))
            delete_button.clicked.connect(lambda: self.delete_selected_item(self.list_widget))

        self.load_json()

        # Configure logging
        # self.configure_logging()

    def load_options_data(self):
        url = 'http://jira.z-onesoftware.com:8080'

        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("Load Options")
        try:
            ok_button = msg_box.addButton(QMessageBox.Ok)
            response = requests.get(url, timeout=2)

            # Check if the response status code indicates success (2xx)
            if response.status_code // 100 == 2:
                msg_box.setIcon(QMessageBox.Information)
                msg_box.setText('正在加载...')
                ok_button.setDisabled(True)
                worker_thread = MsgBoxWorkerThread()
                worker_thread.finished.connect(partial(self.on_task_finished, msg_box, ok_button))

                worker_thread.start()
            else:
                print(f"Failed to access the website {url}. Status code: {response.status_code}")
                msg_box.setIcon(QMessageBox.Warning)
                msg_box.setText("Connection Error.")

        except Exception as e:
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setText("Connection Error.")
            print(e)

        center_point = self.rect().center()
        msg_box_rect = msg_box.frameGeometry()
        msg_box_rect.moveCenter(center_point)
        msg_box.move(msg_box_rect.topLeft())
        msg_box.exec_()

    def on_task_finished(self, msg_box, ok_button, bug_option_1j5, task_option_1j5, bug_option_2j5, task_option_2j5):
        self.jira_bug_option_mapping_1j5 = bug_option_1j5
        self.jira_task_option_mapping_1j5 = task_option_1j5
        self.jira_bug_option_mapping_2j5 = bug_option_2j5
        self.jira_task_option_mapping_2j5 = task_option_2j5
        self.load_json()
        msg_box.setText('加载选项成功!')
        ok_button.setEnabled(True)

    def reset_data(self):
        data.update(deepcopy(self.initial_data))
        self.load_json()

    def switch_product(self):
        if self.radioButton.isChecked() and self.product != '1j5':
            self.product = '1j5'
            data['jira_type'] = self.product
            self.load_json()
        elif self.radioButton_2.isChecked() and self.product != '2j5':
            self.product = '2j5'
            data['jira_type'] = self.product
            self.load_json()

    def switch_confluence(self):
        if self.confluence_true.isChecked():
            self.confluence_username.setText('Enabled')
            self.bug_box.setDisabled(True)
        elif self.confluence_false.isChecked():
            self.confluence_username.setText('Disabled')
            self.bug_box.setDisabled(False)

        if self.confluence_true.isChecked() and self.use_confluence.lower() != 'true':
            self.use_confluence = 'True'
            data['use_confluence'] = self.use_confluence
        elif self.confluence_false.isChecked() and self.use_confluence.lower() != 'false':
            self.use_confluence = 'False'
            data['use_confluence'] = self.use_confluence

    def switch_goal(self):
        if self.goal_perception.isChecked() and self.goal != Goal.PERCEPTION.value:
            self.goal = Goal.PERCEPTION.value
            data['goal'] = self.goal
        elif self.goal_ground_truth.isChecked() and self.goal != Goal.GROUND_TRUTH.value:
            self.goal = Goal.GROUND_TRUTH.value
            data['goal'] = self.goal

    # Function to recursively process JSON data
    def _process_json(self, data):
        if isinstance(data, dict):
            # If it's a dictionary, return the first value
            return next(iter(data.values()))
        elif isinstance(data, list):
            # If it's a list, check if it's empty and return empty if it is
            if len(data) == 0:
                return ''
            else:
                # Process the first element (if any) and return
                return self._process_json(data[0])
        else:
            # Handle other types here (e.g., int, str, float)
            return data

    def _get_items(self, label, group_title):
        mapping = self.jira_task_option_mapping if group_title == 'Task Template' else self.jira_bug_option_mapping

        if mapping:
            if label in mapping:
                return mapping[label]
            else:
                return None
        else:
            return None

    def _add_layout(self, data, group):
        # Clear the existing widgets from the layout
        for i in reversed(range(group.layout().count())):
            widget = group.layout().itemAt(i).widget()
            if widget:
                widget.deleteLater()

        self.value_labels[group.title()] = {}
        for row, (key, value) in enumerate(data.items()):
            key_label = QLabel(jira_option_name_mapping.get(key, key))
            if group.title() != 'Task Template' and group.title() != 'Bug Template':
                if key == 'default':
                    continue
                value_label = QLineEdit(self._process_json(value))
                value_label.textChanged.connect(
                    lambda text, label=key, g=group: self.handle_text_changed(label, g, text))
            else:
                if key not in jira_option_name_mapping.keys():
                    continue
                value_label = QComboBox()
                if items := self._get_items(key, group.title()):
                    for item in items:
                        value_label.addItem(item)
                else:
                    value_label.addItem(self._process_json(value))

                value_label.setCurrentText(self._process_json(value))
                value_label.activated[str].connect(
                    lambda text, label=key, g=group: self.handle_text_changed(label, g, text))

            self.value_labels[group.title()][key] = value_label

            if (group.title() == 'Assignee' and self.product == '2j5' and key != '2j5') or \
                    (group.title() == 'Assignee' and self.product == '1j5' and key == '2j5') or \
                    (key == "labels" or key == "issuetype" or key == 'default'):
                value_label.setDisabled(True)

            group.layout().addWidget(key_label, row, 0)
            group.layout().addWidget(value_label, row, 1)

    def load_json(self):
        self.radioButton.click() if self.product.lower() == '1j5' else self.radioButton_2.click()
        self.confluence_true.click() if self.use_confluence.lower() == 'true' else self.confluence_false.click()
        if self.goal == Goal.PERCEPTION.value:
            self.goal_perception.click()
        elif self.goal == Goal.GROUND_TRUTH.value:
            self.goal_ground_truth.click()

        if self.product == '1j5':
            self.jira_bug_option_mapping = self.jira_bug_option_mapping_1j5
            self.jira_task_option_mapping = self.jira_task_option_mapping_1j5
        else:
            self.jira_bug_option_mapping = self.jira_bug_option_mapping_2j5
            self.jira_task_option_mapping = self.jira_task_option_mapping_2j5

        # Load history from the data.
        self.selected_files = data['source_template']
        self.list_widget.clear()
        self.list_widget.addItems(self.selected_files)

        self.value_labels = {}
        self._add_layout(data['assignee_template'], self.assignee_box)
        self._add_layout(data['task_template'][self.product], self.task_box)
        self._add_layout(data['bug_template'][self.product], self.bug_box)

    def import_items(self, list_widget):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.ReadOnly
        options |= QFileDialog.DirectoryOnly  # Optional: Display only directories

        file_names, _ = QFileDialog.getOpenFileNames(self, "Open CSV File", QDir.homePath(),
                                                     "CSV Files (*.csv);;All Files (*)", options=options)

        for file_name in file_names:
            if file_name not in self.selected_files:
                self.selected_files.append(file_name)
                list_widget.addItem(file_name)
        data['source_template'] = self.selected_files

    def delete_selected_item(self, list_widget):
        selected_item = list_widget.currentItem()
        if selected_item:
            self.selected_files.remove(selected_item.text())
            row = list_widget.row(selected_item)
            list_widget.takeItem(row)
        data['source_template'] = self.selected_files

    def _process_new_value(self, origin, new_value):
        if isinstance(origin, list):
            if isinstance(origin[0], dict):
                return [{list(origin[0].keys())[0]: new_value}]
        elif isinstance(origin, dict):
            return {list(origin.keys())[0]: new_value}
        else:
            return new_value

    def replace_json_value(self, data, key_to_replace, new_value):
        def recursive_replace(obj):
            if isinstance(obj, dict):
                for key, value in obj.items():
                    if key == key_to_replace:
                        obj[key] = self._process_new_value(data[key], new_value)
                    elif isinstance(value, (dict, list)):
                        recursive_replace(value)
            elif isinstance(obj, list):
                for i, item in enumerate(obj):
                    recursive_replace(item)

        recursive_replace(data)

    def handle_text_changed(self, label, group, text):
        # This slot will be called whenever the specified QLineEdit widget's text changes
        template = variable_mapping[group.title()]
        if template == "assignee_template":
            data[template][label] = text
        elif template == "source_template":
            data[template][label] = [text]
        else:
            self.replace_json_value(data[template][self.product], label, text)

    def show_confirmation_dialog(self):
        # Create a confirmation dialog
        confirmation_dialog = QMessageBox(self)
        confirmation_dialog.setWindowTitle('Confirmation')
        confirmation_dialog.setText('Are you sure you want to create issues?')
        confirmation_dialog.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        confirmation_dialog.setDefaultButton(QMessageBox.No)

        # Set the confirmation dialog box to the center.
        center_point = self.rect().center()
        confirmation_dialog_rect = confirmation_dialog.frameGeometry()
        confirmation_dialog_rect.moveCenter(center_point)
        confirmation_dialog.move(confirmation_dialog_rect.topLeft())

        # Execute the dialog and check the result
        result = confirmation_dialog.exec_()
        if result == QMessageBox.Yes:
            # User clicked Yes
            self.worker_thread = MainWorkerThread(data)
            self.worker_thread.error_occurred.connect(
                lambda e: QMessageBox.critical(self, 'Error', f"An error occurred: {str(e)}"))
            self.worker_thread.start()
        else:
            # User clicked No or closed the dialog
            pass

    def closeEvent(self, event):
        # Handle the close event to quit the application
        sys.exit()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app_icon = QIcon('../Config/Resource/UiIcons/icons/Config.png')
    window = JiraToolGUI()
    window.setWindowIcon(app_icon)
    window.setWindowTitle('Jira Toolkit')
    window.show()
    window.setFixedSize(549, 898)

    sys.exit(app.exec_())
