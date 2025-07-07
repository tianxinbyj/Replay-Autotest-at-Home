import argparse
import concurrent.futures
import json
import os
import subprocess
import threading
import time


def get_video_info(input_file):
    """获取视频的基本信息，包括帧率、宽度、高度、总帧数和每帧时间戳"""
    try:
        # 获取视频流基本信息
        cmd_basic = [
            'ffprobe', '-v', 'error', '-select_streams', 'v:0',
            '-show_entries', 'stream=width,height,r_frame_rate,duration',
            '-of', 'json', input_file
        ]
        basic_output = subprocess.check_output(cmd_basic, stderr=subprocess.STDOUT).decode('utf-8')
        basic_info = json.loads(basic_output)['streams'][0]

        width = int(basic_info['width'])
        height = int(basic_info['height'])
        frame_rate = eval(basic_info['r_frame_rate'])  # 将"30/1"这样的字符串转换为浮点数
        duration = float(basic_info.get('duration', 0))

        # 获取每帧的时间戳
        cmd_frames = [
            'ffprobe', '-v', 'error', '-select_streams', 'v:0',
            '-show_frames', '-show_entries', 'frame=pkt_pts_time',
            '-of', 'json', input_file
        ]
        frames_output = subprocess.check_output(cmd_frames, stderr=subprocess.STDOUT).decode('utf-8')
        frames_info = json.loads(frames_output)

        timestamps = []
        for frame in frames_info.get('frames', []):
            if 'pkt_pts_time' in frame:
                timestamps.append(float(frame['pkt_pts_time']))

        total_frames = len(timestamps)

        return {
            'width': width,
            'height': height,
            'frame_rate': frame_rate,
            'total_frames': total_frames,
            'duration': duration,
            'timestamps': timestamps
        }
    except subprocess.CalledProcessError as e:
        print(f"获取视频信息失败: {e.output.decode('utf-8')}")
        raise
    except Exception as e:
        print(f"发生错误: {str(e)}")
        raise


def select_frames_by_ratio(timestamps, select_ratio=3):
    """按固定比例选择帧（例如每3帧选1帧）"""
    if not timestamps or select_ratio <= 0:
        return []

    selected_indices = list(range(0, len(timestamps), select_ratio))
    selected_timestamps = [timestamps[i] for i in selected_indices]

    print(f"已从{len(timestamps)}帧中选择{len(selected_timestamps)}帧，选择比例为{select_ratio}:1")
    return selected_timestamps, selected_indices


def process_frame(input_file, output_file, timestamp, crf_value=32):
    """处理单帧的函数，可被多线程调用"""
    try:
        # 使用ffmpeg提取单帧并编码为H.265
        cmd = [
            'ffmpeg', '-y', '-ss', f'{timestamp}', '-i', input_file,
            '-t', '0.04',  # 提取约40ms的内容
            '-c:v', 'libx265', '-preset', 'ultrafast',  # 使用ultrafast预设提高速度
            '-crf', f'{crf_value}',  # 质量控制参数
            '-x265-params', 'keyint=1:scenecut=0:bframes=0:no-sao=1:rd=1',  # 禁用更多编码功能
            '-pix_fmt', 'yuv420p', output_file
        ]

        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return True, output_file, timestamp, None
    except subprocess.CalledProcessError as e:
        return False, output_file, timestamp, e.stderr
    except Exception as e:
        return False, output_file, timestamp, str(e)


def split_video_to_frames(input_file, output_dir, timestamps=None, select_ratio=3, crf_value=32,
                          workers=5):
    """将视频分割为单帧的H.265文件，每帧都是I帧，按固定比例抽帧，使用多线程加速"""
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    if timestamps is None or len(timestamps) == 0:
        print("警告: 没有可用的时间戳信息，无法进行抽帧处理")
        return

    # 按比例选择帧
    selected_timestamps, selected_indices = select_frames_by_ratio(timestamps, select_ratio)

    # 统计信息
    total_frames = len(selected_timestamps)
    success_count = 0
    failed_count = 0

    # 创建进度显示的锁
    progress_lock = threading.Lock()
    start_time = time.time()

    # 多线程处理帧
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        # 提交所有帧处理任务
        future_to_frame = {
            executor.submit(
                process_frame,
                input_file,
                os.path.join(output_dir, f"frame_{i:05d}.h265"),
                timestamp,
                crf_value
            ): (i, timestamp, original_idx)
            for i, (timestamp, original_idx) in enumerate(zip(selected_timestamps, selected_indices))
        }

        # 处理完成的任务
        for future in concurrent.futures.as_completed(future_to_frame):
            i, timestamp, original_idx = future_to_frame[future]
            success, output_file, ts, error = future.result()

            with progress_lock:
                if success:
                    success_count += 1
                    elapsed = time.time() - start_time
                    fps = success_count / elapsed if elapsed > 0 else 0
                    remaining = (total_frames - success_count - failed_count) / fps if fps > 0 else 0
                    print(
                        f"\r进度: {success_count + failed_count}/{total_frames} 成功: {success_count} 失败: {failed_count} 估计剩余时间: {remaining:.1f}秒",
                        end="")
                else:
                    failed_count += 1
                    print(f"\n处理帧 {i} (时间戳: {ts:.3f}s) 失败: {error}")

    print(f"\n处理完成! 成功: {success_count}, 失败: {failed_count}")

    # 返回选择的帧信息
    return selected_timestamps, selected_indices


def main():
    parser = argparse.ArgumentParser(description='将H.264编码的MKV文件转换为H.265单帧文件')
    parser.add_argument('-i', '--input', required=True, help='输入的MKV文件路径')
    parser.add_argument('-o', '--output', required=True, help='输出目录路径')
    parser.add_argument('--crf', type=int, default=32, help='设置H.265编码的CRF值，越高质量越低(默认32)')
    parser.add_argument('--ratio', type=int, default=3, help='抽帧比例，每N帧选择1帧(默认3)')
    parser.add_argument('--workers', type=int, default=4, help='并行处理的线程数(默认4)')

    args = parser.parse_args()
    input_file = args.input
    output_dir = args.output
    crf_value = args.crf
    select_ratio = args.ratio
    workers = args.workers

    # 验证输入文件是否存在
    if not os.path.exists(input_file):
        print(f"错误: 输入文件 '{input_file}' 不存在")
        return

    try:
        # 获取视频信息
        video_info = get_video_info(input_file)
        print(f"视频信息: {video_info}")

        # 分割视频为单帧
        selected_timestamps, selected_indices = split_video_to_frames(
            input_file,
            output_dir,
            video_info['timestamps'],
            select_ratio,
            crf_value,
            workers
        )

        # 保存时间戳信息到文件
        timestamp_file = os.path.join(output_dir, "frame_timestamps.txt")
        with open(timestamp_file, 'w') as f:
            for i, (ts, original_idx) in enumerate(zip(selected_timestamps, selected_indices)):
                f.write(f"{i:05d} {original_idx:05d} {ts:.6f}\n")

        print(f"处理完成! 单帧H.265文件已保存到: {output_dir}")
        print(f"时间戳信息已保存到: {timestamp_file}")
        print(f"使用CRF值: {crf_value} (质量设置)")
        print(f"抽帧比例: 每{select_ratio}帧选择1帧")
        print(f"使用线程数: {workers}")

    except Exception as e:
        print(f"程序执行失败: {str(e)}")
        return


if __name__ == "__main__":
    main()