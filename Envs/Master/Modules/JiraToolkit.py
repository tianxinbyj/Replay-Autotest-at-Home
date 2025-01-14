#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2023/8/17 上午9:28
# @Author  : Cen Zhengfeng

import html
import json
import logging
import os.path
import re
from datetime import datetime, timedelta
from typing import Union, Dict, Tuple, Any

import requests
import ruamel.yaml
import ruamel.yaml.comments

import pandas as pd
from atlassian import Jira, Confluence
from Envs.Master.Modules.Libs import get_project_path

class CustomFormatter(logging.Formatter):
    def format(self, record):
        if record.levelno == logging.WARNING:
            record.levelname = 'SUCCESS'
        return super().format(record)


log = logging.getLogger(__name__)
atlassian_logger = logging.getLogger('atlassian.jira')

# Set the logging level
log.setLevel(logging.DEBUG)
atlassian_logger.setLevel(logging.DEBUG)

# Create a formatter
formatter = CustomFormatter('[%(levelname)s] %(message)s')
atlassian_formatter = CustomFormatter('[%(levelname)s] %(message)s')

# Create a console handler and set the formatter
ch = logging.StreamHandler()
atlassian_ch = logging.StreamHandler()

ch.setFormatter(formatter)
atlassian_ch.setFormatter(atlassian_formatter)

# Add the handler to the logger
log.addHandler(ch)
atlassian_logger.addHandler(atlassian_ch)

yaml = ruamel.yaml.YAML()

"""
所有信息都在这里改
"""
data = {
    # Jira issue type, expect: 1j5 or 2j5.
    'jira_type': '1j5',
    'use_confluence': 'True',

    # The csv path of issues.
    'source_template': [],

    # Goal: 0:没看; 1:感知; 2:真值; 3:其他.
    'goal': 1,

    # Config List, the order is important, means the format of the task/bug.
    'config_list_1j5': ['sw_version', 'test_object', 'scenario_type', 'topic', 'bug_type'],
    'config_list_2j5': ['sw_version', 'test_object', 'scenario_id', 'bug_type'],

    # Assignee Template
    'assignee_template': {
        "/PI/FS/LaneMarkingsHorizonDebug": "jinxi",
        "/VA/Lines": "jinxi",
        "/VA/FrontViewObstacles": "renweiping",
        "/VA/BevObstaclesDet": "wangtong02",
        "/VA/Obstacles": "hanchao05",
        "2j5": "wuxubin01",
    },

    # Bug Template
    "bug_template": {
        "1j5": {
            # 类型
            "issuetype": {"id": "10403"},
            # 子系统模块
            "customfield_11568": [{"value": "感知"}],
            # 问题责任方
            "customfield_11425": [{"value": "SMSC-AC"}],
            # 系统功能
            "customfield_11567": [{"value": "其他"}],
            # Category
            "customfield_11008": {"value": "性能"},
            # Occurrence
            "customfield_10689": {"value": "低(<20%)"},
            # 影响版本
            "versions": [{"name": "daily08/01"}],
            # Severity
            "customfield_10684": {"value": "B"},
            # VehicleType
            "customfield_10200": [{"value": "IPD平台"}],
            # Test Level
            "customfield_10687": {"value": "软件合格性测试"},
            # Priority
            "priority": {"name": "Medium"},
            # Labels
            "labels": []
        },
        "2j5": {
            # 类型
            "issuetype": {"id": "10403"},
            # 子系统模块
            "customfield_11568": [{"value": "感知"}],
            # 问题责任方
            "customfield_11425": [{"value": "SMSC-AC"}],
            # 系统功能
            "customfield_11567": [{"value": "其他"}],
            # Category
            "customfield_11008": {"value": "性能"},
            # Occurrence
            "customfield_10689": {"value": "低(<20%)"},
            # 影响版本
            "versions": [{"name": "daily08/01"}],
            # Severity
            "customfield_10684": {"value": "B"},
            # VehicleType
            "customfield_10200": [{"value": "IPD平台"}],
            # Test Level
            "customfield_10687": {"value": "软件合格性测试"},
            # Priority
            "priority": {"name": "Medium"},
            # Labels
            "labels": []
        }
    },
    "task_template": {
        "1j5": {
            # 类型
            "issuetype": {"id": "10401"},
            # 子系统模块
            "customfield_11568": [{"value": "感知"}],
            # 影响版本
            # "versions": [{"name": "V1.1.0(8/1)"}],
            # VehicleType
            "customfield_10200": [{"value": "IPD平台"}],
            # Priority
            "priority": {"name": "Medium"},
            # Labels
            "labels": [],
        },
        "2j5": {
            # 类型
            "issuetype": {"id": "10401"},
            # 子系统模块
            "customfield_11568": [{"value": "感知"}],
            # 影响版本
            # "versions": [{"name": "V1.1.0(8/1)"}],
            # VehicleType
            "customfield_10200": [{"value": "IPD平台"}],
            # Priority
            "priority": {"name": "Medium"},
            # Labels
            "labels": [],
        }
    }
}


def get_user(file_path=os.path.join(get_project_path(), 'Docs/Resources/token/jira_token.txt')) -> Tuple[str, str]:
    # Open the text file for reading
    with open(file_path, 'r') as file:
        content = file.read()

    # Check if there are at least two lines in the file
    strings = re.split(r'\s*,\s*', content)
    if len(strings) != 2:
        raise ValueError(f"Missing {file_path}\n"
                         f"Make sure the format be:name,token\n"
                         f"Name must be the username(e.g. tianlong01)")

    if len(strings[0]) > len(strings[1]):
        jira_token = strings[0]
        jira_name = strings[1]
    else:
        jira_name = strings[0]
        jira_token = strings[1]
    return jira_name, jira_token


# Write to a YAML file while preserving comments
def write_yaml_config(data) -> None:
    jira_name, jira_token = get_user()
    data['token'] = jira_token
    data['assignee_template']['default'] = 'lisen04'

    # Define a list of required fields
    fields_to_check = ['token']

    # Check if all required fields are present in the data dictionary
    fields_to_check = [field for field in fields_to_check if not data.get(field)]

    if fields_to_check:
        # If any required fields are missing, raise an exception or exit the program
        raise ValueError(f"Missing required fields: {', '.join(fields_to_check)}")
    with open('/home/zhangliwei01/ZONE/PythonProject/Replay-Autotest-at-Home/Docs/Resources/token/jira_config.yaml', 'w') as jira_config:
        yaml.dump(data, jira_config)


def _field_ctor(config_type, issue_type, project_id, summary, description, assignee) -> json:
    # Construct a field for creating Jira issue.
    if issue_type.lower() == 'bug':
        template = data['bug_template'][config_type]
    else:
        template = data['task_template'][config_type]
        new_due_date = datetime.now() + timedelta(days=60)
        new_due_date_str = new_due_date.strftime("%Y-%m-%d")
        template["customfield_10655"] = new_due_date_str

    template["project"] = ruamel.yaml.comments.CommentedMap()
    template["project"]["id"] = project_id

    template["summary"] = summary
    template["description"] = description

    template["assignee"] = ruamel.yaml.comments.CommentedMap()
    template["assignee"]["name"] = assignee

    return template


def _update_ctor(i_key, c_key) -> json:
    update = {
        "type": {
            "name": "Contains",
            "inward": "is contained by",
            "outward": "contains"
        },
        "inwardIssue": {"key": c_key},
        "outwardIssue": {"key": i_key}
    }

    return update


class JiraToolKit:
    def __init__(self, jira_token: str, csv_path: str, config_type: str):
        self.gen_stats = {'tasks': [], 'bugs': [], 'total_attachments': 0, 'attachment_errors': []}
        self.response = None
        self.info_attachments = None
        self.element_lists = None

        self.jira_url = 'http://jira.z-onesoftware.com:8080'
        self.jira_token = jira_token
        self.jira = Jira(
            url=self.jira_url,
            token=self.jira_token
        )

        self.csv_path = csv_path
        self.config_type = config_type
        self.config_list = data['config_list_1j5'] if config_type == "1j5" else data['config_list_2j5']
        self.checked_issues = []  # Speed up the process of checking issue

        self.use_confluence = True if data['use_confluence'].lower() == 'true' else False
        try:
            _, confluence_token = get_user(os.path.join(get_project_path(), 'Docs/Resources/token/confluence_token.txt'))
        except Exception as e:
            log.error(str(e))
            exit(-1)

        self.confluence_url = 'http://confluence.z-onesoftware.com:8080'
        self.confluence_token = confluence_token
        self.confluence = Confluence(url=self.confluence_url,
                                     token=self.confluence_token) if self.use_confluence else None
        self.checked_pages = []  # Speed up the process of checking Confluence page

        '''
        On Confluence,
        pageId=221979116: ES39感知测试;
        pageId=139089591: 2J5感知测试;
        作为创建页面时使用的Parent Page.
        '''
        self.pageId_1j5 = '221979116'
        self.pageId_2j5 = '139089591'

    def gen_element_list(self) -> None:
        df = pd.read_csv(self.csv_path)
        column_names = df.columns
        element_lists = {}

        for column_name in column_names:
            values = df[column_name].unique().tolist()
            element_lists[column_name] = values

        self.element_lists = element_lists

    def _create_tasks(self, elements, prefix, index, container):
        task_summary = '-'.join(prefix)
        if task_summary != "" and index < len(elements):
            self._create_issue(flag=1, container=container, issue_type='task', summary=task_summary)
            container = self.response['key']

        if index == len(elements):
            self._create_issue(flag=1, container=container, issue_type='bug', summary=task_summary)
            return

        current_element = elements[index]
        for value in self.element_lists[current_element]:
            prefix.append(value)
            self._create_tasks(elements, prefix, index + 1, container)
            prefix.pop()

    def create_jira_rec(self) -> Dict[str, Any]:
        """
        Recursively create jira issues.
        """
        self.preprocess_csv()
        if len(self.info_attachments) > 0:
            self._create_tasks(self.config_list, [], 0, None)
        return self.gen_stats

    def preprocess_csv(self) -> None:
        csv_data = pd.read_csv(self.csv_path)

        # Preprocess, to extract the values for the specified columns into a dictionary.
        columns_to_include = list(self.config_list)
        columns_to_include.append('project_id')
        columns_to_include.append('summary')
        columns_to_include.append('description')
        self.info_attachments = {}

        # Iterate through the DataFrame rows
        for _, row in csv_data.iterrows():

            info_dict = {column: row[column] for column in columns_to_include}
            info = tuple(info_dict.items())
            attachment = row['attachment_path']

            if row['is_valid'] == data['goal']:
                if info in self.info_attachments and attachment not in self.info_attachments[info]:
                    self.info_attachments[info].append(attachment)
                else:
                    self.info_attachments[info] = [attachment]

    def _create_issue(self, flag: int = 0, container: str = None, issue_type: str = None, summary: str = None) -> Union[
        Dict[str, str], None]:
        """
        Add attachment to Issue
        :param issue_type: str, The type of the issue to be created.
        :param flag: int, 0: Do not create issue link; 1: Create issue link via csv; 2: Create via input.
        :param container: str, Enabled only when flag is 1.
        :param summary: str, The customized summary text for the issue.
        """
        for row_tuple, attachments in self.info_attachments.items():
            row = dict(row_tuple)
            project_id = row['project_id']

            summary = row['summary'] if summary is None else summary
            summary_content = summary.split('-')

            task_description = f"数据回灌感知测试-{summary}"
            # The last task, the summary longer than it will be a bug or Confluence page.
            if len(summary_content) == len(self.config_list) - 1:
                task_description = f"{task_description}-异常记录及状态跟踪请见相关Confluence pages."

            description = row['description'] if issue_type.lower() == 'bug' else task_description
            if self.use_confluence:
                description = html.escape(description)  # Replace all the html special symbols
                description = description.replace('\n', '<br/>')  # Replace all the \n to <br/>

            # Set assignee for the template.
            assignee = data['assignee_template']['default']
            if issue_type.lower() == 'bug' or (self.use_confluence and issue_type.lower() == 'task'):
                check_length = len(self.config_list) if issue_type.lower() == 'bug' else len(self.config_list) - 1
                summary_parts = summary.split('-')
                if len(summary_parts) == check_length:
                    if self.config_type == '1j5' and summary_parts[3] in data['assignee_template']:
                        assignee = data['assignee_template'][summary_parts[3]]
                    elif self.config_type == '2j5':
                        assignee = data['assignee_template']['2j5']

            field = _field_ctor(self.config_type, issue_type, project_id, summary, description, assignee)

            # Only create the issue when it does not exist.
            jira_issue_existence = self.check_issue_existence(project_id, summary)

            '''
            Check the summary of the task, only which contains bug(s) will be created.
            Tasks that do not contain bug are useless, and will be ignored.
            '''
            should_be_created = True
            for index in range(len(summary_content)):
                if summary_content[index] != row[self.config_list[index]]:
                    should_be_created = False
                    break

            # Use confluence for bugs.
            if self.use_confluence and should_be_created:
                confluence_page_existence = self.check_page_existence(summary)
                if confluence_page_existence:
                    log.error('Failed, Confluence page with title %s exists, skipped.' % summary)

                if issue_type.lower() == 'bug' and not confluence_page_existence:
                    # Create confluence pages for each bug and link to the parent page and Jira task, respectively.
                    response = None
                    try:
                        # Get parent id.
                        container_title = self.jira.get_issue(container)["fields"]['summary']
                        parent_id = self.confluence.get_page_by_title('AC', container_title)['id']

                        log.warning('Creating Confluence page "%s"', summary)
                        # For description, it requires check and replace all the html special symbols
                        response = self.confluence.create_page('AC', summary, description,
                                                               parent_id, 'page')
                    except Exception as e:
                        log.error('Bug occurred in creating bug page on confluence.')
                        log.error(str(e))

                    if response:
                        self.gen_stats['bugs'].append(response['id'])
                        counter = 0
                        attachment_page_info = "<br/>" + "=" * 30 + "Attachment List" + "=" * 30 + "<br/>"
                        for file_path in attachments:
                            try:
                                # 文件名带中文会乱码
                                # self.confluence.attach_file(file_path, file_path, "application/pdf",
                                #                             page_id=response['id'],
                                #                             comment=f"Uploaded {os.path.basename(file_path)}.")

                                attachment_info = self.confluence.attach_file(file_path, file_path,
                                                                              "application/pdf",
                                                                              page_id=response['id'],
                                                                              comment="None")
                                file_name = attachment_info["results"][0]["title"]
                                hyperlink = f'<ac:link><ri:attachment ri:filename="{file_name}" /></ac:link>'
                                attachment_page_info += hyperlink + "<br/>"
                                counter += 1
                            except FileNotFoundError as e:
                                self.gen_stats['attachment_errors'].append(response['id'])
                                log.error(str(e))
                        attachment_page_info += "=" * 73
                        self.confluence.update_page(
                            page_id=response['id'],
                            title=summary,
                            body=description + "<br/>" + attachment_page_info
                        )

                        log.warning('Added %d attachment for page "%s"', counter, response['title'])
                        self.gen_stats['total_attachments'] += counter
                        del self.info_attachments[row_tuple]

                        if container:
                            # 创建jira wiki link到jira task(container)
                            self._create_confluence_link(container, response["title"], response["id"])

                        return self.response

                elif issue_type.lower() == 'task':
                    if not jira_issue_existence[0]:
                        # Create confluence pages for each task.
                        response = self.jira.issue_create(field)
                    else:
                        log.error('Failed, issue with summary %s and key %s exists, skipped.' % (
                            summary, jira_issue_existence[1]['key']))
                        response = jira_issue_existence[1]

                    if flag == 1 and container is None and not confluence_page_existence:
                        # Create Confluence page under the proper page.
                        parent_id = self.pageId_1j5 if self.config_type == "1j5" else self.pageId_2j5
                        try:
                            log.warning('Creating Confluence page "%s"', summary)
                            self.confluence.create_page('AC', summary, description, parent_id,
                                                        'page')
                        except Exception as e:
                            log.error('Bug occurred in creating main-page on confluence.')
                            log.error(str(e))

                    elif flag == 1 and container is not None and not confluence_page_existence:
                        # Create Confluence page, create pages hierarchically using the container.
                        container_title = self.jira.get_issue(container)["fields"]['summary']
                        parent_id = self.confluence.get_page_by_title('AC', container_title)['id']
                        try:
                            log.warning('Creating Confluence page "%s"', summary)
                            self.confluence.create_page('AC', summary, description, parent_id,
                                                        'page')
                        except Exception as e:
                            log.error('Bug occurred in creating task sub-page on confluence.')
                            log.error(str(e))

                        # Create Jira issue link.
                        update = _update_ctor(response['key'], container)
                        self.jira.create_issue_link(update)

                    if not jira_issue_existence[0]:
                        self.gen_stats['tasks'].append(response['key'])
                        self.response = response
                        return response
                    else:
                        self.response = jira_issue_existence[1]
                        return jira_issue_existence[1]

            # Not use confluence.
            elif not self.use_confluence and should_be_created:
                if not jira_issue_existence[0]:
                    response = self.jira.issue_create(field)

                    if flag == 1 and container is not None:
                        update = _update_ctor(response['key'], container)
                        # log.warning('Creating issue link for "%s"', response['key'])
                        self.jira.create_issue_link(update)

                    if issue_type.lower() == 'bug':
                        self.gen_stats['bugs'].append(response['key'])
                        counter = 0
                        for file_path in attachments:
                            try:
                                self._add_pdf_attachment(response['key'], file_path)
                                counter += 1
                            except FileNotFoundError as e:
                                self.gen_stats['attachment_errors'].append(response['key'])
                                log.error(str(e))

                        log.warning('Added %d attachment for "%s"', counter, response['key'])
                        self.gen_stats['total_attachments'] += counter
                        del self.info_attachments[row_tuple]
                        return self.response
                    elif issue_type.lower() == 'task':
                        self.gen_stats['tasks'].append(response['key'])
                        self.response = response
                        return response
                    else:
                        log.error('Error occurs in issue_type.')
                        exit()

                else:
                    log.error('Failed, issue with summary %s and key %s exists, skipped.' % (
                        summary, jira_issue_existence[1]['key']))
                    self.response = jira_issue_existence[1]

                    issue_content = [item for key in self.info_attachments for item in key]
                    if issue_type.lower() == 'bug':
                        if summary_content != issue_content:
                            return jira_issue_existence[1]
                        else:
                            del self.info_attachments[row_tuple]

                    return jira_issue_existence[1]

    def check_issue_existence(self, project: str, summary: str) -> Tuple[bool, Union[dict, None]]:
        """
        Check if the issue already exists using the summary.
        :param project: str, the key or id of the project.
        :param summary: str, the summary of the issue.
        :return: tuple, it represents the issue exists or not with the key.
        """
        if len(self.checked_issues) > 0 and '-'.join(summary.split('-')[:-1]) in self.checked_issues:
            if summary not in self.checked_issues:
                self.checked_issues.append(summary)
            return False, None

        project_info = project
        summary_info = re.sub(r'(["\'\\{}\[\]])', r'\\\\\1', summary)
        search_result = self.jira.jql(
            jql="Project = {project} AND text ~ '{summary}'".format(project=project_info, summary=summary_info))

        if len(search_result['issues']) == 0:
            if summary not in self.checked_issues:
                self.checked_issues.append(summary)
            return False, None

        for issues in search_result['issues']:
            if issues['fields']['summary'] == summary:
                return True, {'key': issues['key']}
        else:
            return False, None

    def check_page_existence(self, summary: str) -> bool:
        if len(self.checked_pages) > 0 and '-'.join(summary.split('-')[:-1]) in self.checked_pages:
            if summary not in self.checked_pages:
                self.checked_pages.append(summary)
            return False

        if not self.confluence.page_exists('AC', summary) and summary not in self.checked_pages:
            self.checked_pages.append(summary)
            return False
        else:
            return True

    def _add_pdf_attachment(self, issue_key, file_name) -> None:
        url = f"{self.jira_url}/rest/api/2/issue/{issue_key}/attachments"
        api_token = self.jira_token

        headers = {
            'Authorization': f'Bearer {api_token}',
            "Accept": "application/json",
            "X-Atlassian-Token": "no-check"
        }

        try:
            response = requests.request(
                "POST",
                url,
                headers=headers,
                files={
                    "file": (
                        file_name, open(file_name, "rb"), "application/pdf")
                }
            )
        except FileNotFoundError:
            raise FileNotFoundError(f"The file {file_name} does not exist for issue {issue_key}")

    def _create_confluence_link(self, issue_key, title, page_id, relationship='Wiki Page') -> None:
        url = f"{self.jira_url}/rest/api/2/issue/{issue_key}/remotelink"
        api_token = self.jira_token

        link_url = f'http://confluence.z-onesoftware.com:8080/pages/viewpage.action?pageId={page_id}'
        data = {"object": {"url": link_url, "title": title}}
        global_id = f'appId=54380d2a-fb95-3b4e-ac54-cfc0694eb0ea&pageId={page_id}'
        data['globalId'] = global_id
        data["relationship"] = relationship
        data['application'] = {
            "type": "com.atlassian.confluence",
            "name": "Confluence"
        }

        headers = {
            'Authorization': f'Bearer {api_token}',
            'Content-Type': 'application/json',
            'X-Atlassian-Token': 'no-check'
        }

        try:
            response = requests.request("POST", url, headers=headers, data=json.dumps(data))

        except FileNotFoundError:
            raise FileNotFoundError(f"Remote link creation failed for issue {issue_key} with page {page_id}")


def _clean_nones(value) -> Any:
    """
    Recursively remove all None values from dictionaries and lists, and returns
    the result as a new dictionary or list.
    :param value: Any
    """
    if isinstance(value, list):
        return [_clean_nones(x) for x in value if x is not None]
    elif isinstance(value, dict):
        return {
            key: _clean_nones(val)
            for key, val in value.items()
            if val is not None
        }
    else:
        return value


def get_issue(issue_id_or_key: str) -> None:
    jira_url = 'http://jira.z-onesoftware.com:8080'
    _, jira_token = get_user(os.path.join(get_project_path(), 'Docs/Resources/token/jira_token.txt'))
    jira = Jira(
        url=jira_url,
        token=jira_token
    )
    result = _clean_nones(jira.get_issue(issue_id_or_key))
    json_dict = json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False)
    print(json_dict)


def delete_page(page_id, recursive=False) -> None:
    _, confluence_token = get_user(os.path.join(get_project_path(), 'Docs/Resources/token/confluence_token.txt'))
    confluence_url = 'http://confluence.z-onesoftware.com:8080'
    confluence_token = confluence_token
    confluence = Confluence(
        url=confluence_url,
        token=confluence_token
    )
    confluence.remove_page(page_id, recursive=recursive)


def check_field_meta(product_type, issue_type_id=10403):
    from Envs.Master.Tools.JiraGUI import jira_option_name_mapping
    project = 'ACZPD' if product_type == '1j5' else 'ACIPD'
    jira_url = 'http://jira.z-onesoftware.com:8080'
    _, jira_token = get_user(os.path.join(get_project_path(), 'Docs/Resources/token/jira_token.txt'))
    jira = Jira(
        url=jira_url,
        token=jira_token
    )
    result = jira.issue_createmeta_fieldtypes(project, issue_type_id)
    json_dict = json.dumps(result, indent=2, ensure_ascii=False)

    object_list = jira_option_name_mapping.keys()
    value_dict = dict.fromkeys(object_list)
    for field in result['values']:
        if field["fieldId"] in object_list:
            value_dict[field["fieldId"]] = []
            if "allowedValues" in field.keys():
                for item in field["allowedValues"]:
                    if "value" in item:
                        value_dict[field["fieldId"]].append(item["value"])
                    elif "name" in item:
                        value_dict[field["fieldId"]].append(item["name"])

    return value_dict


def append_content_to_page(page_id, content) -> None:
    _, confluence_token = get_user(os.path.join(get_project_path(), 'Docs/Resources/token/confluence_token.txt'))
    confluence_url = 'http://confluence.z-onesoftware.com:8080'
    confluence_token = confluence_token
    confluence = Confluence(
        url=confluence_url,
        token=confluence_token
    )
    existing_page_content = None
    try:
        existing_page_content = confluence.get_page_by_id(page_id, expand="body.storage")
    except Exception as e:
        print(e)
    print(existing_page_content)

    confluence.update_page(
        page_id=page_id,
        title=existing_page_content['title'],
        body=existing_page_content['body']['storage']['value'] + "<br/>" + content
    )


def main(data):
    try:
        write_yaml_config(data)
    except ValueError as e:
        log.error(f"Error: {e}")
    with open('/home/zhangliwei01/ZONE/PythonProject/Replay-Autotest-at-Home/Docs/Resources/token/jira_config.yaml', 'r') as yaml_file:
        data = yaml.load(yaml_file)

    token = data['token']

    jira_csv_paths = []
    if 'source_template' in data:
        for csv_path in data['source_template']:
            jira_csv_paths.append(csv_path)
    else:
        log.error('Invalid csv path.')
        exit(-1)

    stats = {}

    # Create jira issues.
    for jira_csv_path in jira_csv_paths:
        jt = JiraToolKit(token, jira_csv_path, data['jira_type'])
        jt.gen_element_list()
        stats[jira_csv_path] = jt.create_jira_rec()

    for jira_csv_path in jira_csv_paths:
        tasks = stats[jira_csv_path]['tasks']
        bugs = stats[jira_csv_path]['bugs']
        attachments = stats[jira_csv_path]['total_attachments']
        errors = stats[jira_csv_path]['attachment_errors']
        log.info('Summary: For %s, created %d Tasks, %d Bugs, %d Attachments; %d Errors occurred' % (
            jira_csv_path, len(tasks), len(bugs), attachments, len(errors)))
        if len(errors) > 0:
            log.info('Error occurs in Bug: ' + ' '.join(errors))


if __name__ == '__main__':
    # pass
    # main(data)
    # delete_page('141862576', recursive=True)
    get_issue('ACZP39-3632')
    # append_content_to_page('141856696', 'addxxx')
