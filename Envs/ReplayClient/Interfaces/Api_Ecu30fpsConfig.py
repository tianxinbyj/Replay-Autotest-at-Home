"""  
Created on 2024/7/31.  
@author: Tian Loong
function: change ECU config about sensor center to 30FPS
"""
import argparse
import json
import os
import subprocess
import time
from doctest import UnexpectedException

import paramiko
from paramiko import SSHClient


j5a_ssh_host = '172.31.131.35'
j5b_ssh_host = '172.31.131.36'
ssh_port = 22
ssh_user = 'root'
ssh_pass = ''

j5a_ssh_client = paramiko.SSHClient()
j5b_ssh_client = paramiko.SSHClient()

j5a_ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
j5b_ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

def change_sensor_30fps():

    record_ssh = {
        'j5a': None,
        'j5b': None
    }

    try:
        j5a_ssh_client.connect(j5a_ssh_host, ssh_port, username=ssh_user, password=ssh_pass, timeout=10)
        j5b_ssh_client.connect(j5b_ssh_host, ssh_port, username=ssh_user, password=ssh_pass, timeout=10)
    except ConnectionError as EEEE:
        print('build connection failed')
        raise EEEE
    else:
        record_ssh = {
            'j5a': j5a_ssh_client,
            'j5b': j5b_ssh_client
        }
    j5a_ssh_client.exec_command('mount -o remount,rw /')
    j5b_ssh_client.exec_command('mount -o remount,rw /')

    j5a_ssh_client.exec_command('stop sensor_center')
    j5b_ssh_client.exec_command('stop sensor_center')
    time.sleep(0.5)

    # 保存 hb_j5dev.json.bak 为备份文件

    # 检测是否有对应的bak文件，没有则备份，有则不继续备份
    j5a_std_in, j5a_std_out, j5a_std_err = j5a_ssh_client.exec_command('ls /system/etc/sensor_center/camera/B1J5A/vio'
                                                                       '/p2_b1_rx20_0418_5v/hb_j5dev.json.bak')
    # print(j5a_std_out.read().decode('utf-8'), '---____________________---', j5a_std_err.read().decode('utf-8'))

    if not j5a_std_out.read().decode('utf-8').strip():
        print('file bak j5a_5v')
        j5a_ssh_client.exec_command('cd /system/etc/sensor_center/camera/B1J5A/vio/p2_b1_rx20_0418_5v;'
                                    'cp hb_j5dev.json hb_j5dev.json.bak')
    # #j5b 2v
    j5b2v_std_in, j5b2v_std_out, j5b2v_std_err = j5b_ssh_client.exec_command('ls /system/etc/sensor_center/camera/B1J5B/vio'
                                                                             '/p2_b1_rx13_0118_2v/hb_j5dev.json.bak')
    if not j5b2v_std_out.read().decode('utf-8').strip():
        print('file bak j5b 2v')
        j5b_ssh_client.exec_command('cd /system/etc/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_2v;'
                                    'cp hb_j5dev.json hb_j5dev.json.bak')
    # j5b 4v
    j5b4v_std_in, j5b4v_std_out, j5b4v_std_err = j5b_ssh_client.exec_command('ls /system/etc/sensor_center/camera/B1J5B/vio'
                                                                             '/p2_b1_rx13_0118_4v/hb_j5dev.json.bak')
    if not j5b4v_std_out.read().decode('utf-8').strip():
        print('file bak j5b 4v')
        j5b_ssh_client.exec_command('cd /system/etc/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_4v;'
                                    'cp hb_j5dev.json hb_j5dev.json.bak')
    print('备份文件生成完成')
    file_path = os.path.abspath(__file__)
    print('file_path:::',file_path)

    # dd hb_j5dev 出来，并进行读取
    dir_name = os.path.dirname(file_path)
    print(dir_name)
    # j5a 修改 hb_j5dev.json
    dd_cmd = (f"sshpass -p '' ssh root@172.31.131.35 \"dd if=/system/etc/sensor_center/camera/B1J5A/vio/p2_b1_rx20_0418_5v"
              f"/hb_j5dev.json\" | dd of={os.path.join(dir_name, 'hb_j5dev.json')} bs=1M")
    print(dd_cmd)
    os.system(dd_cmd)
    print('cmd success')
    time.sleep(2)
    print(os.path.join(dir_name,'hb_j5dev.json'))
    if os.path.exists(os.path.join(dir_name,'hb_j5dev.json')):
        with open(os.path.join(dir_name,'hb_j5dev.json'), 'r') as hb_j5dev_5v:
            j5a_data = json.load(hb_j5dev_5v)
            print('ecu 中原来的period_us为：',j5a_data['config_0']['lpwm_1']['period_us'])
            j5a_data['config_0']['lpwm_0']['period_us'] = [33333, 50000, 33333, 50000]

            j5a_data['config_0']['lpwm_1']['period_us'] = [33333, 50000, 50000, 50000]
            for port in ['port_0', 'port_1', 'port_2', 'port_3', 'port_5']:
                print(j5a_data['config_0'][port]['fps'])
                j5a_data['config_0'][port]['fps'] = 30
        # # 修改完重新推进板子
        with open(os.path.join(dir_name, 'hb_j5dev.json'), 'w') as j5a_change:
            json.dump(j5a_data, j5a_change, indent=8)
        time.sleep(0.5)
        print(os.path.join(dir_name, 'hb_j5dev.json'))
        dd_in_ecu_cmd = (f"sshpass -p '' scp {os.path.join(dir_name, 'hb_j5dev.json')} root@172.31.131.35:/system/etc"
                         f"/sensor_center/camera/B1J5A/vio/p2_b1_rx20_0418_5v/")
        os.system(dd_in_ecu_cmd)
        # #
        time.sleep(1)
        os.system(f"rm {os.path.join(dir_name, 'hb_j5dev.json')}")
        time.sleep(0.5)
        j5a_ssh_client.exec_command('sync')
    else:
        raise FileNotFoundError(f'j5a_5v: can not find file {os.path.join(dir_name,"hb_j5dev.json")}')

    # # j5b 2v
    dd_cmd = (f"sshpass -p '' ssh root@172.31.131.36 \"dd if=/system/etc/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_2v"
              f"/hb_j5dev.json\" | dd of={os.path.join(dir_name, 'hb_j5dev.json')} bs=1M")
    os.system(dd_cmd)
    time.sleep(2)
    if os.path.exists(os.path.join(dir_name,'hb_j5dev.json')):
        with open(os.path.join(dir_name, 'hb_j5dev.json'), 'r') as hb_j5dev_2v:
            j5b2v_data = json.load(hb_j5dev_2v)
            print('ecu 中原来的period_us为：',j5b2v_data['config_0']['lpwm_1']['period_us'])
            j5b2v_data['config_0']['lpwm_0']['period_us'] = [50000, 33333, 50000, 33333]
            # print(j5a_data['config_0']['lpwm_1'])
            j5b2v_data['config_0']['lpwm_1']['period_us'] = [50000, 33333, 50000, 50000]
            for port in ['port_4', 'port_10']:
                print(j5b2v_data['config_0'][port]['fps'], ": FPS")
                j5b2v_data['config_0'][port]['fps'] = 30
        #
        with open(os.path.join(dir_name, 'hb_j5dev.json'), 'w') as j5b2v_change:
            json.dump(j5b2v_data, j5b2v_change, indent=8)

        # patient bro
        time.sleep(1)
        print(os.path.join(dir_name, 'hb_j5dev.json'))
        scp_2v_in_ecu_cmd = (f"sshpass -p '' scp {os.path.join(dir_name, 'hb_j5dev.json')} root@172.31.131.36:/system/etc"
                          f"/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_2v/")
        j5b2v_scp_res = subprocess.run(scp_2v_in_ecu_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True)
        if j5b2v_scp_res.returncode == 0:
            print('success j5b 2v scp')
        else:
            print('failed j5b 2v scp')
        #

        time.sleep(0.5)
        os.system(f"rm {os.path.join(dir_name, 'hb_j5dev.json')}")
        time.sleep(0.2)
        j5b_ssh_client.exec_command('sync')

    else:
        raise FileNotFoundError(f'j5b_2v:can not find file {os.path.join(dir_name,"hb_j5dev.json")}')

    # j5b 4v
    dd_cmd = (f"sshpass -p '' ssh root@172.31.131.36 \"dd if=/system/etc/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_4v"
              f"/hb_j5dev.json\" | dd of={os.path.join(dir_name, 'hb_j5dev.json')} bs=1M")
    os.system(dd_cmd)
    time.sleep(0.5)

    if os.path.exists(os.path.join(dir_name,'hb_j5dev.json')):
        with open(os.path.join(dir_name, 'hb_j5dev.json'), 'r') as hb_j5dev_4v:
            j5b4v_data = json.load(hb_j5dev_4v)
            print('ecu 中原来的period_us为：',j5b4v_data['config_0']['lpwm_0']['period_us'])
            j5b4v_data['config_0']['lpwm_0']['period_us'] = [50000, 33333, 50000, 33333]
            # print(j5a_data['config_0']['lpwm_1'])
            j5b4v_data['config_0']['lpwm_1']['period_us'] = [50000, 33333, 50000, 50000]
            for port in ['port_0', 'port_1', 'port_2', 'port_3']:
                print(j5b4v_data['config_0'][port]['fps'])
                j5b4v_data['config_0'][port]['fps'] = 30

        with open(os.path.join(dir_name, 'hb_j5dev.json'), 'w') as j5b4v_change:
            json.dump(j5b4v_data, j5b4v_change, indent=8)
        # patient bro
        time.sleep(2)
        scp_4v_in_ecu_cmd = (f"sshpass -p '' scp {os.path.join(dir_name, 'hb_j5dev.json')} root@172.31.131.36:/system/etc"
                          f"/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_4v/")

        j5b4v_scp_res = subprocess.run(scp_4v_in_ecu_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True)
        if j5b4v_scp_res.returncode == 0:
            print('success j5b 4v scp')
        else:
            print('failed j5b 4v scp')

        # os.system(scp_4v_in_ecu_cmd)
        time.sleep(1)
        os.system(f"rm {os.path.join(dir_name, 'hb_j5dev.json')}")
        j5b_ssh_client.exec_command('sync')
    else:
        raise FileNotFoundError(f'j5b_4v:can not find file {os.path.join(dir_name,"hb_j5dev.json")}')

    j5a_ssh_client.exec_command('start sensor_center')

    j5b_ssh_client.exec_command('start sensor_center')
    time.sleep(3)
    print('start sensor_center')
    for client in record_ssh.values():
        if client:
            client.close()
    print('ssh client close')


def back_sensor_20fps():
    record_ssh = {
        'j5a': None,
        'j5b': None
    }

    try:
        j5a_ssh_client.connect(j5a_ssh_host, ssh_port, username=ssh_user, password=ssh_pass, timeout=10)
        j5b_ssh_client.connect(j5b_ssh_host, ssh_port, username=ssh_user, password=ssh_pass, timeout=10)
    except ConnectionError as EEEE:
        print('build connection failed')
        raise EEEE
    else:
        record_ssh = {
            'j5a': j5a_ssh_client,
            'j5b': j5b_ssh_client
        }

    j5a_ssh_client.exec_command('mount -o remount,rw /')
    j5b_ssh_client.exec_command('mount -o remount,rw /')

    j5a_ssh_client.exec_command('stop sensor_center')
    j5b_ssh_client.exec_command('stop sensor_center')
    time.sleep(0.5)

    # 将修改为30fps之前的bak文件恢复为正常文件，覆盖之前的文件

    # 检测是否有对应的bak文件，没有则备份，有则不继续备份, 必须三个文件都有才能修改
    # 5v
    _a_, j5a_std_out, j5a_std_err = j5a_ssh_client.exec_command('ls -l /system/etc/sensor_center/camera/B1J5A/vio'
                                                                       '/p2_b1_rx20_0418_5v/hb_j5dev.json.bak')
    # 2v
    _2v_, j5b2v_std_out, j5b2v_std_err = j5b_ssh_client.exec_command('ls -l /system/etc/sensor_center/camera/B1J5B'
                                                                     '/vio/p2_b1_rx13_0118_2v/hb_j5dev.json.bak')

    # j5b 4v
    _4v_, j5b4v_std_out, j5b4v_std_err = j5b_ssh_client.exec_command('ls -l /system/etc/sensor_center/camera/B1J5B/vio'
                                                                             '/p2_b1_rx13_0118_4v/hb_j5dev.json.bak')

    # 不存在 某个 bak文件
    if fff:= j5a_std_err.read().decode('utf-8').strip():
        print('bak file not exists，do not need back 2 20fps j5a_5v')
        print(fff)
    if bbb:= j5b2v_std_err.read().decode('utf-8').strip():
        print('bak file not exists，do not need back 2 20fps j5b_2v')
        print(bbb)
    if bbb_ := j5b4v_std_err.read().decode('utf-8').strip():
        print('bak file not exists，do not need back 2 20fps j5b_4v')
        print(bbb_)

    # 存在 20fps 的备份文件
    if j5a_std_out.read().decode('utf-8').strip() and j5b2v_std_out.read().decode('utf-8').strip() and j5b4v_std_out.read().decode('utf-8').strip() :
        print('back 2 bak file 20fps j5a_5v')
        # 将bak文件重新恢复为当前配置文件
        j5a_ssh_client.exec_command('cd /system/etc/sensor_center/camera/B1J5A/vio/p2_b1_rx20_0418_5v;'
                                    'cp hb_j5dev.json.bak hb_j5dev.json')
        j5b_ssh_client.exec_command('cd /system/etc/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_4v;'
                                    'cp hb_j5dev.json.bak hb_j5dev.json')
        j5b_ssh_client.exec_command('cd /system/etc/sensor_center/camera/B1J5B/vio/p2_b1_rx13_0118_2v;'
                                    'cp hb_j5dev.json.bak hb_j5dev.json')
        time.sleep(0.5)
        j5a_ssh_client.exec_command('sync')
        j5b_ssh_client.exec_command('sync')
    # 其他状况直接报错
    else:

        # 其他情况 报错前关闭ssh 连接
        for client in record_ssh.values():
            if client:
                client.close()
        print('ssh client close unexpectedly, error happened')
        raise UnexpectedException('发生了未知错误，请手动登录ecu进行检查')

    # 正确执行后最终 重启sensor 关闭conn

    j5a_ssh_client.exec_command('start sensor_center')
    j5b_ssh_client.exec_command('start sensor_center')
    time.sleep(3)
    print('start sensor_center')

    for client in record_ssh.values():
        if client:
            client.close()

    print('ssh client close successful')


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Change Camera Config")
    parser.add_argument("-e", "--ecu_type", default='ES37',type=str,required=False,help="ECU TYPE")
    args = parser.parse_args()

    if args.ecu_type not in ['ES37', 'J6E', 'J6', '1J5']:
        raise Exception("Invalid ECU Type, ECU tpye must in ('ES37', 'J6E', 'J6', '1J5')")


    if res:
        print("flash camera config success")
    else:
        print(0)
    change_sensor_30fps()
    back_sensor_20fps()
    pass