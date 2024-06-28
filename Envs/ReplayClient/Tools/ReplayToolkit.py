"""
# File      :ReplayToolkit.py
# Time      :2023/4/11 下午1:14
# Author    :Bu Yujun
"""
import os
import sys
import time
from datetime import datetime
from functools import partial

from PyQt5.QtCore import QSize, QUrl, QObject, pyqtSignal, QThread, QTimer, pyqtSlot
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QPushButton, QLabel, QSpacerItem, QSizePolicy
from ping3 import ping
from qfluentwidgets import setThemeColor, SwitchButton

from Envs.ReplayClient.Uiforms.replay_controller import Ui_InjectionController
from Envs.ReplayClient.Modules.DataSelector import DataSelector
from Envs.ReplayClient.Modules.CameraMonitor import CameraMonitor
from Envs.ReplayClient.Modules.PowerControl.power_controller import PowerSupplyObjectManager
from Envs.ReplayClient.Modules.ReplayManagement import ReplayManagement
from Utils.Libs import bench_config, test_encyclopaedia, project_path


def set_icon(Qw, img, x, y):
    Qw.setIcon(QIcon(img))
    Qw.setIconSize(QSize(x, y))


def clear_all_widgets(ui):
    for i in reversed(range(ui.cam_viewer.layout().count())):
        widget = ui.cam_viewer.layout().itemAt(i).widget()
        if widget is not None:
            widget.deleteLater()


def update_gui(ui, result):
    # Clear the existing widgets from the layout
    clear_all_widgets(ui)

    data = result['data']
    fluctuating_input = result['fluctuating_input']
    if len(data) > 0:
        for row, (key, value) in enumerate(data.items()):
            key_label = QLabel(key)
            key_label.setStyleSheet("font-size: 16px; text-align: center;")
            value_label = QLabel(value)
            if key not in fluctuating_input:
                value_label.setStyleSheet("font-size: 16px; text-align: center;")
            else:
                value_label.setStyleSheet("font-size: 50px; color: red; text-align: center;")

            # Add key_label to the layout
            ui.cam_viewer.layout().addWidget(key_label, row, 0)

            # Add a spacer item to create space between key_label and value_label
            spacer_item = QSpacerItem(50, 1, QSizePolicy.Fixed, QSizePolicy.Fixed)
            ui.cam_viewer.layout().addItem(spacer_item, row, 1)

            # Add value_label to the layout
            ui.cam_viewer.layout().addWidget(value_label, row, 2)

        # if len(data) < 10:
        #     print('Camera data does not match the expected count, check the board status!')
        #     print(f'Current camera data: {data}')

    else:
        print('No camera data, check the board status!')


class DataProcessor(QObject):
    data_ready = pyqtSignal(dict)
    start_timer_signal = pyqtSignal()
    stop_timer_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.timer = QTimer(self)
        self.timer.setInterval(1000)

        self.save_result = False

        # Connect signals to slots
        self.start_timer_signal.connect(self.timer.start)
        self.stop_timer_signal.connect(self.timer.stop)

        # Connect the timeout signal to the get_data method
        self.timer.timeout.connect(self.get_data)

    @pyqtSlot(bool, object)
    def start_processing(self, start_client_value, cm_class):
        # Start the timer when processing is requested
        if start_client_value:
            self.start_timer_signal.emit()
        else:
            self.stop_timer_signal.emit()

        # Store the cm_class for later use in the get_data method
        self.timer.setProperty("cm_class", cm_class)

    @pyqtSlot()
    def get_data(self):
        # This method will be called at regular intervals
        cm_class = self.timer.property("cm_class")
        if cm_class is not None and cm_class.is_connected():
            self.process_data(cm_class)

    def process_data(self, cm_class):
        cm_class.get_camera_fps_once(save_result=self.save_result)

        # Get fps of cameras, and specify the cameras with fluctuating data.
        data = cm_class.cam_input_fps
        fluctuating_input = cm_class.fluctuating_input
        result = {'data': data, 'fluctuating_input': fluctuating_input}
        self.data_ready.emit(result)

    def stop_processing(self):
        # Stop the timer when processing is no longer needed
        self.stop_timer_signal.emit()


class ConnectionWorker(QObject):
    connection_established = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.force_stop = False
        self.ip_addr = None

    def stop(self):
        self.force_stop = True


class PowerSupplyConnWorker(ConnectionWorker):
    power_status_values = pyqtSignal(dict)

    def __init__(self, instruments_manager, serial_number):
        super().__init__()
        self.instruments_manager = instruments_manager
        self.serial_number = serial_number

    def try_connect(self):
        while not self.force_stop:
            power_status = self.instruments_manager.perform_instrument_action(self.serial_number, 'get_status')
            self.power_status_values.emit(power_status)

            time.sleep(1)


class BoardConnWorker(ConnectionWorker):
    start_client = pyqtSignal(bool)

    def __init__(self, ip_addr):
        super().__init__()
        self.ip_addr = ip_addr
        self.board_connected = False
        self.client_connected = False

    def is_connected(self):
        if ping(self.ip_addr) is None:
            return False
        else:
            return True

    def try_connect(self):
        while not self.force_stop:
            if self.is_connected() != self.board_connected:
                self.board_connected = not self.board_connected
                self.connection_established.emit(self.board_connected)

                # If from connected becomes not connected.
                if not self.board_connected:
                    print('Connection with the board lost.')
                    self.client_connected = False
                    self.start_client.emit(self.client_connected)

            if self.board_connected and not self.client_connected:
                self.client_connected = True
                self.start_client.emit(self.client_connected)
                self.connection_established.emit(self.client_connected)

            if not self.board_connected:
                self.connection_established.emit(False)

            time.sleep(0.1)


class ReplayToolkit(QMainWindow):
    stopDataProcessorTimer = pyqtSignal()

    def __init__(self, parent=None, product='ES37'):
        super().__init__(parent)
        self.ui = Ui_InjectionController()
        self.ui.setupUi(self)

        # 配置参数
        self.BOARD_IP_ADDR = list(test_encyclopaedia[product]['ip'].values())[1:]

        # 在启动的时候隐藏播放器的控制栏
        self.ui.CAM_BACK.playBar.fadeOut()
        self.ui.CAM_BACK_LEFT.playBar.fadeOut()
        self.ui.CAM_BACK_RIGHT.playBar.fadeOut()
        self.ui.CAM_FRONT_120.playBar.fadeOut()
        self.ui.CAM_FRONT_LEFT.playBar.fadeOut()
        self.ui.CAM_FRONT_RIGHT.playBar.fadeOut()

        # Set Theme Color of fluent widgets
        setThemeColor('#0E96FE')

        # 选择数据文件夹
        self.dataSelector = DataSelector()
        self.segment_btn = {}
        self.media_player = {}
        self.segment_selected = None
        self.start_time_data = 0

        # http Api控制设备
        set_icon(self.ui.startORstop, os.path.join(project_path, 'Docs', 'Resources', 'Icon', 'start.png'), 60, 60)
        self.replay_management = ReplayManagement()
        self.is_injection = False
        self.start_time_inject = 0
        self.task_id = 0
        self.update_injection_timer = QTimer(self)
        self.update_injection_timer.timeout.connect(self.get_replay_process)
        self.ui.injection_status.setStyleSheet(
            'background-color: steelblue;'
        )

        # 刷新页面状态
        updateTimer = QTimer(self)
        updateTimer.timeout.connect(self.update_status)
        updateTimer.start(1000)

        self.load_data()

        '''
        Establish ssh connection to the board, and get the input fps of all cameras.
        '''
        self.hostname, self.username, self.password = self.BOARD_IP_ADDR, 'root', ''

        # If more than one board ip is using, we only check the first one, which is enough for now.
        self.checking_hostname = self.hostname[0] if isinstance(self.hostname, list) else self.hostname

        self.ui.host_label.setText(self.checking_hostname)
        self.cm_class = CameraMonitor(self.hostname, self.username, self.password)

        self.board_monitor = BoardConnWorker(self.checking_hostname)
        self.board_monitor_thread = QThread()
        self.board_monitor.moveToThread(self.board_monitor_thread)
        self.board_monitor_thread.started.connect(self.board_monitor.try_connect)
        self.board_monitor_thread.start()
        self.board_monitor.connection_established.connect(
            partial(self.connection_worker_signal_handler, widget=self.ui.ssh_status_label))
        self.board_monitor.start_client.connect(self.start_client_handler)

        self.data_processor = DataProcessor()
        self.data_processor_thread = QThread()
        self.data_processor.moveToThread(self.data_processor_thread)
        self.data_processor_thread.start()
        self.board_monitor.start_client.connect(
            lambda start_client_value: self.data_processor.start_processing(start_client_value, self.cm_class))
        self.data_processor.data_ready.connect(self.gui_update_handler)

        '''
        Add a monitor of the power supply.
        '''
        self.instrument_manager = PowerSupplyObjectManager()  # 创建一个电源管理器
        if len(self.instrument_manager.serial_numbers) > 0:
            self.serial_number = self.instrument_manager.serial_numbers[0]  # 默认只有一个电源，用于控制

            self.power_supply_monitor = PowerSupplyConnWorker(self.instrument_manager, self.serial_number)
            self.power_supply_monitor_thread = QThread()
            self.power_supply_monitor.moveToThread(self.power_supply_monitor_thread)
            self.power_supply_monitor_thread.started.connect(self.power_supply_monitor.try_connect)
            self.power_supply_monitor_thread.start()
            self.power_supply_monitor.power_status_values.connect(
                partial(self.power_status_handler, widgets=(
                    self.ui.power_status_toggle, self.ui.power_voltage_value_label, self.ui.power_current_value_label)))
            self.ui.power_status_toggle.checkedChanged.connect(self.power_toggle_handler)
            self.power_supply_monitor_thread.finished.connect(self.power_supply_monitor.stop)
        else:
            print('No power supply found!')
            self.ui.connection_viewer_2.setDisabled(True)

        self.board_monitor_thread.finished.connect(self.board_monitor.stop)
        self.data_processor_thread.finished.connect(self.data_processor.stop_processing)

        self.closeEvent = self.on_window_close  # Stop the timer of DataProcessor

    def start_client_handler(self, state):
        if state:
            self.cm_class.connect()
            # self.cm_class.client.connect(self.hostname, username=self.username, password=self.password, timeout=0.5)
        else:
            self.cm_class.disconnect()

    def gui_update_handler(self, result):
        update_gui(self.ui, result)

    def connection_worker_signal_handler(self, signal, widget=None):
        if isinstance(widget, QLabel):
            if signal:
                widget.setStyleSheet("color: green; text-align: center;")
                widget.setText('Success')
            else:
                widget.setStyleSheet("color: red; text-align: center;")
                widget.setText('Failed')

        if isinstance(widget, SwitchButton):
            widget.blockSignals(True)
            widget.setChecked(not widget.isChecked()) if signal != widget.isChecked() else None
            widget.blockSignals(False)

    def power_status_handler(self, values: dict, widgets: tuple = None):
        if widgets[0].isChecked() != values['power_is_on']:
            widgets[0].blockSignals(True)
            widgets[0].setChecked(values['power_is_on'])
            widgets[0].blockSignals(False)

        for i, key in enumerate(['voltage_value', 'current_value']):
            value = values.get(key, 0)
            widgets[i + 1].setText(str(value))
            if key == 'current_value' and value >= 2.0:
                widgets[i + 1].setStyleSheet("color: green; text-align: center;")

    def power_toggle_handler(self, checked):
        self.instrument_manager.perform_instrument_action(self.serial_number, 'switch_to_remote_mode')
        if checked:
            self.instrument_manager.perform_instrument_action(self.serial_number, 'power_on')
        else:
            self.instrument_manager.perform_instrument_action(self.serial_number, 'power_off')
            self.ui.power_voltage_value_label.setText('N/A')
            self.ui.power_current_value_label.setText('N/A')
            self.ui.power_current_value_label.setStyleSheet("")
        self.instrument_manager.perform_instrument_action(self.serial_number, 'switch_to_panel_mode')

    def on_window_close(self, event):
        if self.is_injection:
            self.stop_replay()

        self.data_processor.stop_processing()
        self.instrument_manager.close_all_instruments()

        event.accept()

    def set_save_result(self, save_result):
        self.data_processor.save_result = save_result

    def load_data(self):
        # 把数据切片罗列出来
        for qw in [QPushButton, QLabel]:
            for cw in self.ui.segment_container.findChildren(qw):
                cw.deleteLater()

        if self.dataSelector.data is not None:
            # 放置数据切片选择按钮
            height = 0
            for seg in self.dataSelector.data.index:
                btn = QPushButton(self.ui.segment_container)
                btn.setText(f'{seg}')
                btn.setObjectName(seg)
                btn.setStyleSheet(
                    'font: 10pt;'
                )
                btn.clicked.connect(self.select_segment)
                btn.setCheckable(True)
                btn.resize(200, 25)
                btn.move(0, height)
                height += 30
                btn.show()
                self.segment_btn[f'{seg}'] = btn
            self.ui.segment_container.setMinimumSize(200, height)

            self.ui.startORstop.clicked.connect(self.start_replay)

            # 放入播放器
            # for camera in self.dataSelector.data.columns:
            #     if 'CAM_' in camera and 'FISHEYE' not in camera and '30' not in camera:
            #         # self.media_player[camera] = QMediaPlayer(self, QMediaPlayer.StreamPlayback)
            #         self.media_player[camera] = QMediaPlayer(self, QMediaPlayer.VideoSurface)
            #         if hasattr(self.ui, camera):
            #             exec('self.media_player[camera].setVideoOutput(self.ui.{:s})'.format(camera))

            # 放入播放器
            for camera in self.dataSelector.data.columns:
                if 'CAM_' in camera and 'FISHEYE' not in camera and '30' not in camera:
                    if hasattr(self.ui, camera):
                        self.media_player[camera] = getattr(self.ui, camera)

    def select_segment(self, event=None, segment=None):
        if segment is None:
            segment = self.sender().text()
        self.segment_btn[segment].setChecked(True)
        self.segment_btn[segment].setStyleSheet(
            'background-color: limegreen;'
            'font: 10pt;'
        )
        for seg in self.segment_btn.keys():
            if seg != segment:
                self.segment_btn[seg].setChecked(False)
                self.segment_btn[seg].setStyleSheet(
                    'background-color: white;'
                    'font: 10pt;'
                )
        self.segment_selected = self.segment_btn[segment].objectName()
        self.start_time_data = time.mktime(time.strptime(segment[:-9], "%Y%m%d_%H%M%S"))

        timestamp = time.time()
        dt = datetime.fromtimestamp(timestamp)
        formatted_time = dt.strftime('%Y_%m_%d_%H_%M_%S')
        self.cm_class.csv_filename = f"{formatted_time}_{segment}.csv"

        pic_path = self.dataSelector.gen_video_shot(self.segment_selected)
        pix = QPixmap(pic_path)
        self.ui.video_pic.raise_()
        self.ui.video_pic.setPixmap(pix)
        self.ui.video_pic.setScaledContents(True)
        for camera, media_player in self.media_player.items():
            media_player.stop()

    def gen_replay_config(self):
        video = self.dataSelector.gen_video_config(self.segment_selected)
        self.dataSelector.gen_can_config(self.segment_selected)
        # 将视频映射到播放器
        for camera, media_player in self.media_player.items():
            media_player.setVideo(QUrl.fromLocalFile(video[camera]))

    def start_replay(self):
        # 修改config文件
        self.gen_replay_config()
        self.ui.video_pic.lower()

        # 开始回灌
        res = self.replay_management.start_replay()

        if res:
            # 开始储存相机帧率信息
            self.set_save_result(True)
            self.start_time_inject = time.time()
            self.task_id = self.replay_management.task_id
            print('操作成功，开始回灌{:d}!'.format(self.task_id))
            self.is_injection = True
            set_icon(self.ui.startORstop, os.path.join(project_path, 'Docs', 'Resources', 'Icon', 'stop.png'), 60, 60)
            self.ui.startORstop.clicked.disconnect(self.start_replay)
            self.ui.startORstop.clicked.connect(self.stop_replay)
            self.update_injection_timer.start(2789)
            # 播放6V or Front
            for camera, media_player in self.media_player.items():
                # media_player.play()
                if 'FRONT_120' in camera:
                    media_player.play()

            for seg_btn in self.segment_btn.values():
                seg_btn.setEnabled(False)

        else:
            self.task_id = 0
            self.replay_management.stop_replay()
            QMessageBox.warning(self, 'warning', '回灌失败，请重新尝试！',
                                QMessageBox.Ok, QMessageBox.Ok)

    def stop_replay(self):
        for seg_btn in self.segment_btn.values():
            seg_btn.setEnabled(True)

        # 结束回灌
        self.replay_management.stop_replay()
        # 停止储存相机帧率信息
        self.set_save_result(False)
        self.is_injection = False

        set_icon(self.ui.startORstop, os.path.join(project_path, 'Docs', 'Resources', 'Icon', 'start.png'), 60, 60)
        self.ui.startORstop.clicked.disconnect(self.stop_replay)
        self.ui.startORstop.clicked.connect(self.start_replay)
        self.ui.injection_status.setStyleSheet(
            'background-color: steelblue;'
        )

        self.update_injection_timer.stop()
        for camera, media_player in self.media_player.items():
            if 'EYE' not in camera:
                media_player.pause()

        if self.replay_management.task_id:
            QMessageBox.information(self, 'information', '操作成功，中止回灌{:d}'.format(self.task_id),
                                    QMessageBox.Ok, QMessageBox.Ok)
        else:
            QMessageBox.warning(self, 'warning', '没有可以终止的回灌！',
                                QMessageBox.Ok, QMessageBox.Ok)

    def get_replay_process(self):
        res = self.replay_management.get_replay_process()
        if res:
            self.ui.injection_status.setStyleSheet(
                'background-color: limegreen;'
            )
            self.ui.injection_timing.setText('{:.0f} sec'.format(time.time() - self.start_time_inject))
            self.ui.injection_progress.setText('{:.1%}'.format(res))
        else:
            self.ui.injection_status.setStyleSheet(
                'background-color: lightcoral;'
            )
            self.ui.injection_timing.setText('N/A')
            self.ui.injection_progress.setText('N/A')

        if res > 0.99:
            self.stop_replay()

    def update_status(self):
        if not self.is_injection:
            self.ui.startORstop.setEnabled(self.segment_selected is not None)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ReplayToolkit().show()
    sys.exit(app.exec_())
