# 预处理模块（PreProcess）
# 简介
本模块包含了一系列用于数据处理和预处理的类和方法，主要用于处理与自动驾驶或物体检测相关的数据。这些功能包括但不限于计算时间间隔、角度、矩形点的坐标、探测有效性、物体观测角度范围、物体遮挡率、物体运动状态等。

# 功能概述
calculate_time_gap: 计算两个时间序列之间的最佳时间间隔，以最小化速度误差。
calculate_angle: 计算从原点到给定点的方位角和仰角。
RectPoints: 计算给定位置、偏航角、长度和宽度的矩形的四个角点坐标。
VisionAngleRange: 计算物体在参考点的观察角度范围。
IsCoverageValid: 计算物体的被遮挡率，并判断其是否有效。
DruDirection: 计算物体的运动和朝向状态。
IsKeyObj: 判断物体是否为关键目标。
IsObstaclesDetectedValid: 计算动态障碍物是否在标定的探测范围内。
ObstaclesPreprocess: 一个综合类，用于按顺序应用多个预处理步骤。
使用方法

# 依赖关系
确保你的Python环境中已安装以下库：
numpy
pandas
scipy


# 安装
```markdown  
pip install PerceptMetricsTool-0.1.0-py3-none-any.whl
```

# 导入
```markdown  
from PerceptMetrics import PreProcess
```

# 初始化预处理对象
```markdown  
preprocess = PreProcess.ObstaclesPreprocess()  
```

# 你有一个包含物体检测数据的DataFrame
```markdown  
# data 来源于原始数据，不限于gt或者pred数据
data = pd.read_csv(pred_data_path, index_col=False)
```

# 运行预处理  
部分类允许通过input_parameter_container字典传入自定义参数，如：
```markdown  
input_parameter_container = {
    'coverage_reference_point': [2, 0, 1],
    'coverage_threshold': 0.6,
    'lane_width': 3.6,
    'moving_threshold': 2,
    'key_coverage_threshold': 0.1,
    'ROI': {'x': [-80, 120], 'y': [-30, 30]},
}
processed_data = preprocess.run(data, input_parameter_container)  # 如果不需要修改默认参数，可以传入 None  
```

# processed_data 现在包含了预处理后的新列
```markdown  
print(processed_data.head())
```

许可证
本代码遵循MIT许可证。