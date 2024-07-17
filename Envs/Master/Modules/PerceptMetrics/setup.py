from setuptools import setup, find_packages

setup(
    name='PerceptMetricsTool',  # 包名
    version='0.1.0',  # 版本号
    packages=find_packages(),  # 自动查找当前目录及子目录下所有Python包
    install_requires=[  # 安装此包时需要先安装的依赖列表
        'numpy',
        'pandas',
        'scipy',
    ],
    # 其他可选参数
    author='Bu Yujun',  # 作者名
    author_email='18516225486@163.com',  # 作者邮箱
    description='A brief description of your package',  # 包的简短描述
    long_description=open('README.md').read() if (open('README.md').read(1)) else '',  # 包的详细描述，通常读取README文件
    long_description_content_type='text/markdown',  # 详细描述的文件类型，如Markdown
    url='http://github.com/yourusername/your_package_name',  # 包的URL
    classifiers=[  # 包的分类信息
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',  # 指定Python版本要求
)