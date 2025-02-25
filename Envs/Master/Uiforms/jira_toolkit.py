# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'jira_toolkit.ui'
#
# Created by: PyQt5 UI code generator 5.15.11
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.setEnabled(True)
        Form.resize(549, 898)
        self.assignee_box = QtWidgets.QGroupBox(Form)
        self.assignee_box.setGeometry(QtCore.QRect(110, 190, 421, 261))
        self.assignee_box.setObjectName("assignee_box")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.assignee_box)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.create_issues = QtWidgets.QPushButton(Form)
        self.create_issues.setGeometry(QtCore.QRect(10, 830, 251, 51))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(14)
        self.create_issues.setFont(font)
        self.create_issues.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.create_issues.setObjectName("create_issues")
        self.reset_config = QtWidgets.QPushButton(Form)
        self.reset_config.setGeometry(QtCore.QRect(10, 730, 251, 41))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(14)
        self.reset_config.setFont(font)
        self.reset_config.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.reset_config.setObjectName("reset_config")
        self.task_box = QtWidgets.QGroupBox(Form)
        self.task_box.setGeometry(QtCore.QRect(10, 460, 251, 211))
        self.task_box.setObjectName("task_box")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.task_box)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.gen_config = QtWidgets.QPushButton(Form)
        self.gen_config.setGeometry(QtCore.QRect(10, 780, 251, 41))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(14)
        self.gen_config.setFont(font)
        self.gen_config.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.gen_config.setObjectName("gen_config")
        self.product_box = QtWidgets.QGroupBox(Form)
        self.product_box.setGeometry(QtCore.QRect(10, 190, 81, 80))
        self.product_box.setObjectName("product_box")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.product_box)
        self.verticalLayout.setObjectName("verticalLayout")
        self.radioButton = QtWidgets.QRadioButton(self.product_box)
        self.radioButton.setObjectName("radioButton")
        self.verticalLayout.addWidget(self.radioButton)
        self.radioButton_2 = QtWidgets.QRadioButton(self.product_box)
        self.radioButton_2.setObjectName("radioButton_2")
        self.verticalLayout.addWidget(self.radioButton_2)
        self.bug_box = QtWidgets.QGroupBox(Form)
        self.bug_box.setGeometry(QtCore.QRect(280, 460, 251, 361))
        self.bug_box.setObjectName("bug_box")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.bug_box)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.source_box = QtWidgets.QGroupBox(Form)
        self.source_box.setGeometry(QtCore.QRect(10, 60, 521, 121))
        self.source_box.setObjectName("source_box")
        self.gridLayout = QtWidgets.QGridLayout(self.source_box)
        self.gridLayout.setObjectName("gridLayout")
        self.terminate = QtWidgets.QPushButton(Form)
        self.terminate.setGeometry(QtCore.QRect(280, 830, 251, 51))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(14)
        self.terminate.setFont(font)
        self.terminate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.terminate.setObjectName("terminate")
        self.confluence_box = QtWidgets.QGroupBox(Form)
        self.confluence_box.setGeometry(QtCore.QRect(10, 275, 81, 80))
        self.confluence_box.setObjectName("confluence_box")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.confluence_box)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.confluence_true = QtWidgets.QRadioButton(self.confluence_box)
        self.confluence_true.setObjectName("confluence_true")
        self.verticalLayout_3.addWidget(self.confluence_true)
        self.confluence_false = QtWidgets.QRadioButton(self.confluence_box)
        self.confluence_false.setObjectName("confluence_false")
        self.verticalLayout_3.addWidget(self.confluence_false)
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(10, 10, 251, 41))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_22 = QtWidgets.QLabel(self.frame)
        self.label_22.setObjectName("label_22")
        self.gridLayout_3.addWidget(self.label_22, 0, 0, 1, 1)
        self.jira_username = QtWidgets.QLabel(self.frame)
        self.jira_username.setEnabled(True)
        self.jira_username.setObjectName("jira_username")
        self.gridLayout_3.addWidget(self.jira_username, 0, 1, 1, 1)
        self.frame_3 = QtWidgets.QFrame(Form)
        self.frame_3.setGeometry(QtCore.QRect(280, 10, 251, 41))
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.frame_3)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.confluence_username = QtWidgets.QLabel(self.frame_3)
        self.confluence_username.setEnabled(True)
        self.confluence_username.setObjectName("confluence_username")
        self.gridLayout_2.addWidget(self.confluence_username, 1, 1, 1, 1)
        self.label_23 = QtWidgets.QLabel(self.frame_3)
        self.label_23.setObjectName("label_23")
        self.gridLayout_2.addWidget(self.label_23, 1, 0, 1, 1)
        self.goal_box = QtWidgets.QGroupBox(Form)
        self.goal_box.setGeometry(QtCore.QRect(10, 360, 81, 91))
        self.goal_box.setObjectName("goal_box")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.goal_box)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.goal_perception = QtWidgets.QRadioButton(self.goal_box)
        self.goal_perception.setObjectName("goal_perception")
        self.verticalLayout_4.addWidget(self.goal_perception)
        self.goal_ground_truth = QtWidgets.QRadioButton(self.goal_box)
        self.goal_ground_truth.setObjectName("goal_ground_truth")
        self.verticalLayout_4.addWidget(self.goal_ground_truth)
        self.load_options = QtWidgets.QPushButton(Form)
        self.load_options.setGeometry(QtCore.QRect(10, 680, 251, 41))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(14)
        self.load_options.setFont(font)
        self.load_options.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.load_options.setObjectName("load_options")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.assignee_box.setTitle(_translate("Form", "Assignee"))
        self.create_issues.setText(_translate("Form", "Create Issues"))
        self.reset_config.setText(_translate("Form", "Reset Config"))
        self.task_box.setTitle(_translate("Form", "Task Template"))
        self.gen_config.setText(_translate("Form", "Generate Config"))
        self.product_box.setTitle(_translate("Form", "Product"))
        self.radioButton.setText(_translate("Form", "es39"))
        self.radioButton_2.setText(_translate("Form", "2j5"))
        self.bug_box.setTitle(_translate("Form", "Bug Template"))
        self.source_box.setTitle(_translate("Form", "Source"))
        self.terminate.setText(_translate("Form", "Exit"))
        self.confluence_box.setTitle(_translate("Form", "Confluence"))
        self.confluence_true.setText(_translate("Form", "True"))
        self.confluence_false.setText(_translate("Form", "False"))
        self.label_22.setText(_translate("Form", "Username:"))
        self.jira_username.setText(_translate("Form", "?"))
        self.confluence_username.setText(_translate("Form", "Disabled"))
        self.label_23.setText(_translate("Form", "Confluence:"))
        self.goal_box.setTitle(_translate("Form", "Goal"))
        self.goal_perception.setText(_translate("Form", "感知"))
        self.goal_ground_truth.setText(_translate("Form", "真值"))
        self.load_options.setText(_translate("Form", "Load Options"))
