#!/bin/bash

# 检查 tmux 是否安装
if ! command -v tmux &> /dev/null; then
    echo "tmux 未安装，请先安装 tmux。"
    exit 1
fi

# 检查 sshpass 是否安装
if ! command -v sshpass &> /dev/null; then
    echo "sshpass 未安装，请先安装 sshpass。"
    exit 1
fi

# 定义 SSH 服务器和密码
SSH_SERVER="root@172.31.1.40"
PASSWORD=""

# 提取主机名（去掉用户名部分）
HOSTNAME=$(echo "$SSH_SERVER" | cut -d@ -f2)

# 检查主机是否可ping通
echo "正在检查主机 $HOSTNAME 是否可访问..."
if ! ping -c 3 "$HOSTNAME" &> /dev/null; then
    echo "错误：无法ping通主机 $HOSTNAME，请检查网络连接。"
    exit 1
fi
echo "主机 $HOSTNAME 可访问，继续执行..."

# 检查 my_ssh_session 会话是否存在，如果存在则关闭它
tmux has-session -t my_ssh_session 2>/dev/null
if [ $? -eq 0 ]; then
    tmux kill-session -t my_ssh_session
    echo "已关闭旧的 my_ssh_session 会话"
fi

# 创建一个新的 tmux 会话
tmux new-session -d -s my_ssh_session

# 第一个窗口，连接到指定服务器并执行命令
tmux send-keys -t my_ssh_session:0 "sshpass -p '$PASSWORD' ssh $SSH_SERVER" C-m
sleep 1  # 等待 SSH 连接建立
tmux send-keys -t my_ssh_session:0 "cd ../app; blockdev --setrw \`mount | grep by-name/app_ | awk '{print \$1}'\`; mount -o remount,rw /app" C-m

# 垂直分割窗口
tmux split-window -v -t my_ssh_session:0

# 第二个窗口，检查进程是否存在
tmux send-keys -t my_ssh_session:0.1 "sshpass -p '$PASSWORD' ssh $SSH_SERVER" C-m
sleep 1  # 等待 SSH 连接建立
# tmux send-keys -t my_ssh_session:0.1 'PROCESS_CHECK=$(ps -ef | grep "sensor_global_fillback.json" | grep -v grep); if [ -z "$PROCESS_CHECK" ]; then mount -o remount,rw /; stop sensor_center; export PLUGIN_PATH=/app/lib64; export AA_CONFIG_FILES_PATH=/app/etc; export LOG_APP_CONFIG_PATH=/system/etc/sensor_center/logging.json; export LD_LIBRARY_PATH=/app/lib64/oe:/app/lib64/oe:/system/hobot/lib/:/system/hobot/lib/sensor/:${LD_LIBRARY_PATH}; export LIBMM_FM_PATH=/lib/firmware/libmm/; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;echo "HAHAHAHAHA准备启动回灌模式AHAHAHAHAH" ;/usr/bin/sensor_center /system/etc/sensor_center/sensor_global_fillback.json & fi' C-m

# 核心修改：在执行任何操作前，先强制杀死相关进程
tmux send-keys -t my_ssh_session:0.1 $'echo "正在清理旧的回灌模式进程..."' C-m
# 查找包含 sensor_global_fillback.json 的进程并强制杀死（忽略 grep 自身进程）
tmux send-keys -t my_ssh_session:0.1 $'ps | grep "sensor_global_fillback.json" | grep -v grep | awk \'{print $1}\' | xargs -r kill -9' C-m
tmux send-keys -t my_ssh_session:0.1 $'echo "旧进程清理完成"' C-m
sleep 1  # 等待进程杀死完成
echo "终止结束 "
# 调整窗口布局
#tmux select-layout -t my_ssh_session:0 tiled
#
## 附加到会话
#tmux attach-session -t my_ssh_session
#
#sleep 1000
# 检查进程是否已被杀死（可选，用于验证）
tmux send-keys -t my_ssh_session:0.1 $'if ps | grep "sensor_global_fillback.json" | grep -v grep > /dev/null; then echo "警告：仍有残留进程"; else echo "所有相关进程已清除"; fi' C-m

sleep 1


# 第二个窗口，检查进程是否存在并启动回灌模式（带重试机制）
tmux send-keys -t my_ssh_session:0.1 "sshpass -p '$PASSWORD' ssh $SSH_SERVER" C-m
sleep 1  # 等待 SSH 连接建立
# 1. 检查进程状态并决定是否继续
tmux send-keys -t my_ssh_session:0.1 $'PROCESS_CHECK=$(ps | grep "sensor_global_fillback.json" | grep -v grep); if [ -z "$PROCESS_CHECK" ]; then echo "准备启动回灌模式..."; else echo "回灌模式运行中...."; echo 'continue'; fi' C-m

# 2. 执行初始化操作
tmux send-keys -t my_ssh_session:0.1 $'mount -o remount,rw / || { echo "挂载失败,退出"; exit 1; }' C-m
tmux send-keys -t my_ssh_session:0.1 $'stop sensor_center || echo "停止服务失败,继续尝试启动....."' C-m

# 3. 设置环境变量
tmux send-keys -t my_ssh_session:0.1 $'export PLUGIN_PATH=/app/lib64' C-m
tmux send-keys -t my_ssh_session:0.1 $'export AA_CONFIG_FILES_PATH=/app/etc' C-m
tmux send-keys -t my_ssh_session:0.1 $'export LOG_APP_CONFIG_PATH=/system/etc/sensor_center/logging.json' C-m
tmux send-keys -t my_ssh_session:0.1 $'export LD_LIBRARY_PATH=/app/lib64/oe:/app/lib64/oe:/system/hobot/lib/:/system/hobot/lib/sensor/:${LD_LIBRARY_PATH}' C-m
tmux send-keys -t my_ssh_session:0.1 $'export LIBMM_FM_PATH=/lib/firmware/libmm/' C-m
tmux send-keys -t my_ssh_session:0.1 $'export CYCLONEDDS_URI=/etc/cyclonedds_config.xml' C-m

# 4. 执行重试逻辑
tmux send-keys -t my_ssh_session:0.1 $'MAX_RETRIES=3; RETRY_COUNT=0; SUCCESS=false; while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do ((RETRY_COUNT++)); echo "尝试启动回灌模式 (尝试次数: $RETRY_COUNT/$MAX_RETRIES)"; echo "启动回灌模式命令: /usr/bin/sensor_center /system/etc/sensor_center/sensor_global_fillback.json &"; /usr/bin/sensor_center /system/etc/sensor_center/sensor_global_fillback.json & SENSOR_PID=$!; sleep 2; NEW_PROCESS_CHECK=$(ps | grep "sensor_global_fillback.json" | grep -v grep); if [ -n "$NEW_PROCESS_CHECK" ]; then echo "++++++回灌模式已启动+++++++"; SUCCESS=true; break; else echo "启动失败，等待 5 秒后重试..."; if ps -p $SENSOR_PID > /dev/null; then kill -9 $SENSOR_PID; fi; sleep 5; fi; done; if [ "$SUCCESS" = "false" ]; then echo "错误：尝试 $MAX_RETRIES 次后仍无法启动回灌模式"; fi' C-m



# 由于目标命令已在后台运行，这里不再重复执行
# 但是我们可以检查是否成功启动了后台进程（这里简单通过再次检查进程实现）
sleep 1
tmux send-keys -t my_ssh_session:0.1 'PROCESS_CHECK=$(ps -ef | grep "sensor_global_fillback.json" | grep -v grep); if [ -n "$PROCESS_CHECK" ]; then echo "++++++回灌模式已启动+++++++"; else echo "启动回灌模式命令执行失败"; fi' C-m

# 水平分割第二个窗口，形成第三个窗口
tmux split-window -h -t my_ssh_session:0.1

# 再次垂直分割第三个窗口，形成第四个窗口
tmux split-window -v -t my_ssh_session:0.2


# 在第四个窗口运行自定义函数
tmux send-keys -t my_ssh_session:0.3 "sshpass -p '$PASSWORD' ssh $SSH_SERVER" C-m
sleep 1  # 等待 SSH 连接建立
# tmux send-keys -t my_ssh_session:0.3 "cd /log/app/sensor_center" C-m
tmux send-keys -t my_ssh_session:0.3 "echo '开始监控播放最新日志文件...'" C-m

# 定义函数来查询远程最大编号的日志文件名
# 定义获取远程最大日志文件的函数
get_max_log_filename() {
    SSH_SERVER="root@172.31.1.40"
    # echo $SSH_SERVER
    # echo "aaaabbbccc"
    local remote_command="ls /log/app/sensor_center/log_[0-9][0-9][0-9][0-9].log 2>/dev/null | sort -n | tail -n 1"
    local max_log_filename=$(sshpass -p "$PASSWORD" ssh "$SSH_SERVER" "$remote_command")
    
    if [ -z "$max_log_filename" ]; then
        echo "未找到符合条件的日志文件" >&2
        return 1
    fi
    
    # 返回绝对路径
    echo "$max_log_filename"
}

# 定义 tail_latest_log_file 函数
tail_latest_log_file() {
    local current_log_file=""
    echo "开始监控远程最新日志文件..."
    local $cout_find = 0
    while true; do
        # 直接调用本地定义的 get_max_log_filename 函数
        local latest_log_file=$(get_max_log_filename)
        
        if [ $? -ne 0 ]; then
            echo "$latest_log_file" >&2
        elif [ "$latest_log_file" == "$current_log_file" ]; then
            echo "未发现新的日志文件---$cout_find"
        else
            echo "找到最新的日志文件: $latest_log_file"
            current_log_file="$latest_log_file"
            
            # 使用 ssh 直接 tail 远程文件
            echo "正在其他窗口 tail 最新日志: $latest_log_file"

                        # 发送两次 C-c 停止当前的 tail 命令
            tmux send-keys -t my_ssh_session:0.3 C-c
            tmux send-keys -t my_ssh_session:0.3 C-c
            sleep 1
            
            current_log_file="$latest_log_file"
            
            # 在第三个窗口执行 tail -f 命令（使用绝对路径）
            tmux send-keys -t my_ssh_session:0.3 "tail -f $current_log_file" C-m
        fi
        
        sleep 21  # 每10秒检查一次
        ((cout_find++))
    done
}

# # 第三个窗口，连接到指定服务器并运行 tail_latest_log_file 函数
# tmux send-keys -t my_ssh_session:0.2 "sshpass -p '$PASSWORD' ssh $SSH_SERVER" C-m
sleep 2  # 等待 SSH 连接建立
tmux send-keys -t my_ssh_session:0.2 "$(cat << EOF
$(typeset -f tail_latest_log_file)
$(typeset -f get_max_log_filename)
tail_latest_log_file
EOF
)" C-m



# 调整窗口布局
tmux select-layout -t my_ssh_session:0 tiled

# 附加到会话
tmux attach-session -t my_ssh_session
