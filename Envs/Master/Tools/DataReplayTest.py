"""
@Author: BU YUJUN
@Date: 2024/8/9 上午12:00
"""
import threading

from Envs.Master.Modules.DataGrinder import *
from Envs.Master.Modules.ReplayController import ReplayController
from Utils.Logger import UDPLogServer


class DataReplayTestPilot:

    def __init__(self, test_project_path):
        self.log_server = None
        self.test_project_path = test_project_path
        workspace_folder = os.path.join(test_project_path, '03_Workspace')
        if not os.path.exists(os.path.join(workspace_folder, 'install')):
            send_log(self, '未找到 03_Workspace/install 文件！')
            return

        # 寻找所有TestConfig.yaml
        test_config_yaml_list = glob.glob(os.path.join(test_project_path, '04_TestData', '*', 'TestConfig.yaml'))
        if not test_config_yaml_list:
            send_log(self, '未找到TestConfig.yaml文件！')
            return

        self.test_project_path = test_project_path
        pred_folder = os.path.join(test_project_path, '01_Prediction')
        gt_folder = os.path.join(test_project_path, '02_GroundTruth')

        self.feature_group_list = []
        self.task_folder_dict = {}
        self.test_config_dict = {}
        for test_config_yaml in test_config_yaml_list:
            with open(test_config_yaml) as f:
                test_config = yaml.load(f, Loader=yaml.FullLoader)
            test_topic = test_config['test_topic']
            test_config['pred_folder'] = pred_folder
            test_config['gt_folder'] = gt_folder
            self.feature_group_list.append(test_config['feature_group'])

            with open(test_config_yaml, 'w', encoding='utf-8') as f:
                yaml.dump(test_config,
                          f, encoding='utf-8', allow_unicode=True, sort_keys=False)

            if test_topic not in self.task_folder_dict:
                self.task_folder_dict[test_topic] = []
            self.task_folder_dict[test_topic].append(os.path.dirname(test_config_yaml))

            if test_topic not in self.test_config_dict:
                self.test_config_dict[test_topic] = []
            self.test_config_dict[test_topic].append(test_config)

    def start_log_server(self):
        self.log_server = UDPLogServer()
        t = threading.Thread(target=self.log_server.start)
        t.daemon = True
        t.start()

    def replay_and_record(self):

        scenario_list = []
        for test_topic in self.test_config_dict.keys():
            send_log(self, f'录制 {test_topic} ros2bag')
            for test_config in self.test_config_dict[test_topic]:
                for scenario_tag in test_config['scenario_tag']:
                    scenario_list.extend(scenario_tag['scenario_id'])

        # 使用第一个test_config的值作为录包的test_action依据
        test_topic = list(self.test_config_dict.keys())[0]
        test_config = self.test_config_dict[test_topic][0]
        if (test_config['test_action']['ros2bag']['record']
                or test_config['test_action']['ros2bag']['truth']
                or test_config['test_action']['ros2bag']['video_info']):

                replay_config = {
                    'product': test_config['product'],
                    'feature_group': test_config['feature_group'],
                    'replay_action': test_config['test_action']['ros2bag'],
                    'pred_folder': test_config['pred_folder'],
                    'gt_folder': test_config['gt_folder'],
                    'workspace': os.path.join(self.test_project_path, '03_Workspace'),
                    'scenario_id': sorted(set(scenario_list)),
                }

                ReplayController(replay_config).start()

    def data_grinder(self):
        for test_topic in self.test_config_dict.keys():
            for task_folder in self.task_folder_dict[test_topic]:
                if not os.path.exists(os.path.join(self.test_project_path, '01_Prediction', 'topic_output_statistics.csv')):
                    send_log(self, f'{task_folder}不存在topic_output_statistics')
                else:
                    if test_topic == 'Obstacles':
                        DataGrinderObstaclesOneTask(task_folder).start()
                    elif test_topic == 'Lines':
                        DataGrinderLinesOneTask(task_folder).start()
                    elif test_topic == 'SLots':
                        DataGrinderSlotsOneTask(task_folder).start()

    def start(self):
        if len(list(set(self.feature_group_list))) > 1:
            send_log(self, '有超过一个的feature_group存在, 请检查test_config')
            return

        send_log(self, f'{list(set(self.feature_group_list))[0]} 开始测试')
        self.start_log_server()
        self.replay_and_record()
        self.data_grinder()
        self.log_server.stop()
