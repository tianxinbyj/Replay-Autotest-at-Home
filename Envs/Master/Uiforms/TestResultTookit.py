# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'TestResultTookit.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_test_result_tookit(object):
    def setupUi(self, test_result_tookit):
        test_result_tookit.setObjectName("test_result_tookit")
        test_result_tookit.resize(519, 499)
        self.groupBox = QtWidgets.QGroupBox(test_result_tookit)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 501, 161))
        self.groupBox.setObjectName("groupBox")
        self.delete_data_folder = QtWidgets.QPushButton(self.groupBox)
        self.delete_data_folder.setGeometry(QtCore.QRect(350, 70, 141, 31))
        self.delete_data_folder.setObjectName("delete_data_folder")
        self.add_data_folder = QtWidgets.QPushButton(self.groupBox)
        self.add_data_folder.setGeometry(QtCore.QRect(350, 30, 141, 31))
        self.add_data_folder.setObjectName("add_data_folder")
        self.data_folder_list = QtWidgets.QListWidget(self.groupBox)
        self.data_folder_list.setGeometry(QtCore.QRect(10, 30, 331, 121))
        self.data_folder_list.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.data_folder_list.setObjectName("data_folder_list")
        self.groupBox_2 = QtWidgets.QGroupBox(test_result_tookit)
        self.groupBox_2.setGeometry(QtCore.QRect(10, 190, 501, 71))
        self.groupBox_2.setObjectName("groupBox_2")
        self.topic_combox = QtWidgets.QComboBox(self.groupBox_2)
        self.topic_combox.setGeometry(QtCore.QRect(70, 30, 141, 31))
        self.topic_combox.setObjectName("topic_combox")
        self.label = QtWidgets.QLabel(self.groupBox_2)
        self.label.setGeometry(QtCore.QRect(20, 30, 51, 31))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.groupBox_2)
        self.label_2.setGeometry(QtCore.QRect(260, 30, 67, 31))
        self.label_2.setObjectName("label_2")
        self.metric_combox = QtWidgets.QComboBox(self.groupBox_2)
        self.metric_combox.setGeometry(QtCore.QRect(330, 30, 161, 31))
        self.metric_combox.setObjectName("metric_combox")

        self.retranslateUi(test_result_tookit)
        QtCore.QMetaObject.connectSlotsByName(test_result_tookit)

    def retranslateUi(self, test_result_tookit):
        _translate = QtCore.QCoreApplication.translate
        test_result_tookit.setWindowTitle(_translate("test_result_tookit", "TestResultTookit"))
        self.groupBox.setTitle(_translate("test_result_tookit", "1. Load Data"))
        self.delete_data_folder.setText(_translate("test_result_tookit", "Delete Data Folder"))
        self.add_data_folder.setText(_translate("test_result_tookit", "Add Data Folder"))
        self.groupBox_2.setTitle(_translate("test_result_tookit", "2. Select MetricType"))
        self.label.setText(_translate("test_result_tookit", "Topic"))
        self.label_2.setText(_translate("test_result_tookit", "Metric"))
