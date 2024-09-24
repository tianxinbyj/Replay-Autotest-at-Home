import glob
import json
import os.path
import sys
from PyQt5.QtCore import QTimer, Qt, QSize, QDate, pyqtSignal
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtWidgets import QHeaderView, QTableWidgetItem, QAbstractItemView, QMenu
from PyQt5.QtWidgets import QMainWindow, QFileDialog, QMessageBox
from PyQt5.QtWidgets import QPushButton, QWidget, QTreeWidget, QCheckBox
from PyQt5.QtWidgets import QTreeWidgetItem, QComboBox, QLineEdit
from PyQt5.QtWidgets import QApplication
from Envs.Master.Uiforms.TestResultTookit import Ui_test_result_tookit


class TestResultToolkit(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_test_result_tookit()
        self.ui.setupUi(self)
        self.setFixedSize(self.width(), self.height())
        self.setWindowFlags(Qt.WindowCloseButtonHint)

        self.last_folder = None
        self.data_folder_list = {}

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

    def delete_data_folder(self):
        current_item = self.ui.data_folder_list.currentItem()
        if current_item:
            key = current_item.text()
            row = self.ui.data_folder_list.row(current_item)
            self.ui.data_folder_list.takeItem(row)
            del self.data_folder_list[key]

    def get_json_list(self):
        for data_folder in self.data_folder_list.values():
            for f in os.listdir(data_folder):





if __name__ == '__main__':
    app = QApplication([])
    jko = TestResultToolkit()
    jko.show()
    sys.exit(app.exec_())