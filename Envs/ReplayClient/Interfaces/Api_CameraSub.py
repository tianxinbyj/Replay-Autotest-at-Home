"""  
Author: Jiang Heng Tao
Date: 8/5/24
function: use camera_sub in ecu , save image (.jpeg) from replay FPGA and scp .jpeg to local replay_client
"""
import paramiko
import time
import subprocess

def copy_images(hostname='172.31.131.35', username='root', password='', durable_time = 10, task = 'J5A', remote_folder = '/log'):


    local_folder = '/media/data/ecu_jpeg'  # 本地目录路径

    print("starting task {}".format(task))
    # 创建 SSH 对象
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # 连接服务器
    ssh.connect(hostname=hostname, username=username, password=password)
    print("connected")

    #commands = ['cd /log', 'mount -o remount,rw /', 'touch save_image']
    command = f"cd /log && mount -o remount,rw / && touch save_image"
    stdin, stdout, stderr = ssh.exec_command(command)
    print('{} successfully !!!'.format(command))
    time.sleep(2)
    # for command in commands:
    #     stdin, stdout, stderr = ssh.exec_command(command)
    #     output = stdout.read().decode()
    #     print('{} successfully !!!'.format(command))
    #     time.sleep(2)

    try:
        if task == 'J5A':
            # 使用timeout命令限制命令执行时间
            timeout_command = f"cd /log && export LD_LIBRARY_PATH=/app/lib64 && timeout {durable_time}s /system/bin/camera_sub /system/etc/sensor_client/sensor_client.json camera_0 camera_1 camera_2 camera_3 camera_5"
            #timeout_command = f"timeout {durable_time}s /system/bin/camera_sub /system/etc/sensor_client/sensor_client.json camera_0 camera_1 camera_2 camera_3 camera_5"
            stdin, stdout, stderr = ssh.exec_command(timeout_command)

            # 读取输出
            output = stdout.read().decode()
            print('output:', output)
            error_output = stderr.read().decode()
            print('error_output:', error_output)


            print(f"Command timed out after {durable_time} seconds")
            time.sleep(5)
            copy_command = f"sshpass -p '' scp {username}@{hostname}:{remote_folder}/*.jpeg {local_folder}"

            try:
                subprocess.run(copy_command, shell=True, check=True)
                print("文件传输成功")
                # time.sleep(2)
                # 删除远程文件夹下的所有.jpeg文件
                delete_command ='find /log -type f -name "*.jpeg" -exec rm {} \;'
                stdin, stdout, stderr = ssh.exec_command(delete_command)
                # 等待命令执行完成
                exit_status = stdout.channel.recv_exit_status()

                # 检查是否有错误输出
                if stderr.read().decode('utf-8').strip():
                    print(f"Error: {stderr.read().decode('utf-8')}")
                else:
                    print("Files deleted successfully.")
                    time.sleep(3)

            except subprocess.CalledProcessError as e:
                    print(f"文件传输失败: {e}")

        elif task == 'J5B':
            # 使用timeout命令限制命令执行时间
            timeout_command = f"cd /log && export LD_LIBRARY_PATH=/app/lib64 && timeout {durable_time}s /system/bin/camera_sub /system/etc/sensor_client/sensor_client.json camera_4 camera_6 camera_7 camera_8 camera_9 camera_10"
            stdin, stdout, stderr = ssh.exec_command(timeout_command)

            # 读取输出
            output = stdout.read().decode()
            print('output:', output)
            error_output = stderr.read().decode()
            print('error_output:', error_output)

            print(f"Command timed out after {durable_time} seconds")
            time.sleep(durable_time+5)
            copy_command = f"sshpass -p '' scp {username}@{hostname}:{remote_folder}/*.jpeg {local_folder}"
            # 执行命令
            try:
                subprocess.run(copy_command, shell=True, check=True)
                print("文件传输成功")
                time.sleep(2)
                # 删除远程文件夹下的所有.jpeg文件
                delete_command = 'find /log -type f -name "*.jpeg" -exec rm {} \;'
                stdin, stdout, stderr = ssh.exec_command(delete_command)
                # 等待命令执行完成
                exit_status = stdout.channel.recv_exit_status()

                # 检查是否有错误输出
                if stderr.read().decode('utf-8').strip():
                    print(f"Error: {stderr.read().decode('utf-8')}")
                else:
                    print("Files deleted successfully.")
                    time.sleep(3)
            except subprocess.CalledProcessError as e:
                print(f"文件传输失败: {e}")
        else:
            print('please choose between J5A and J5B')

    except KeyboardInterrupt:
        print("Stopped by user.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        ssh.close()

    print("task {} has finished".format(task))
    time.sleep(3)


def main():
    copy_images(hostname = '172.31.131.35', username = 'root', password = '', durable_time = 10, task = 'J5A', remote_folder = '/log')
    copy_images(hostname = '172.31.131.36', username = 'root', password = '', durable_time = 10, task = 'J5B', remote_folder = '/log')

if __name__ == "__main__":
    main()


