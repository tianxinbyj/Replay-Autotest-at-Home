#!/bin/bash

# 检查是否传入了 LOCAL_DIR 参数
if [ -z "$1" ]; then
    echo "错误：未传入本地目录路径！"
    exit 1
fi

# 远程服务器信息
REMOTE_IP="172.31.1.40"
REMOTE_USER="root"
REMOTE_PASSWORD=""
REMOTE_DIR="/log/app/sensor_center"

# 本地目录
LOCAL_DIR="$1"

# 创建本地目录（如果不存在）
mkdir -p "$LOCAL_DIR"

# 获取所有匹配的日志文件列表
echo "正在获取所有日志文件..."
ALL_LOG_FILES=$(sshpass -p "$REMOTE_PASSWORD" ssh "$REMOTE_USER@$REMOTE_IP" "ls $REMOTE_DIR/log_*.log 2>/dev/null")

if [ -z "$ALL_LOG_FILES" ]; then
    echo "错误：在远程服务器上未找到日志文件！"
    exit 1
fi

# 提取文件名中的数字部分并排序
SORTED_FILES=$(echo "$ALL_LOG_FILES" | while read -r file; do
    # 提取文件名中的数字部分
    num=$(basename "$file" .log | cut -d_ -f2)
    # 输出数字和文件名，用空格分隔
    echo "$num $file"
done | sort -n -k1 | tail -n 5 | cut -d' ' -f2)

echo "找到以下最新的3个文件："
echo "$SORTED_FILES"

# 复制文件
echo "开始复制文件..."
for file in $SORTED_FILES; do
    filename=$(basename "$file")
    echo "正在复制 $filename..."
    sshpass -p "$REMOTE_PASSWORD" scp "$REMOTE_USER@$REMOTE_IP:$file" "$LOCAL_DIR/"
    if [ $? -ne 0 ]; then
        echo "警告：复制 $filename 失败！"
    fi
done

echo "操作完成！文件已复制到 $LOCAL_DIR"