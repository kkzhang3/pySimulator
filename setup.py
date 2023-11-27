# coding=utf8
import platform
import os
import glob
from setuptools import setup
from setuptools import find_packages
from Cython.Build import cythonize


from modules.__init__ import __version__


py_ver = f"py{platform.python_version()}_{platform.architecture()[0]}"
ver = f"{__version__}_{py_ver}"

source_files = [
    'modules/gui/QtGUI.py',
    'modules/MIMO_2x2/process/MIMOmodel.py',
    'modules\SISOSim1\process\sim.py',
]


# # 定义一个函数来在指定目录创建__init__.py文件（如果不存在）
# def create_init_if_not_exists(directory):
#     init_file = os.path.join(directory, '__init__.py')
#     if not os.path.exists(init_file):
#         open(init_file, 'a').close()  # 创建一个空的__init__.py文件
#         # print(f"Created {init_file}")
#     else:
#         # print(f"{init_file} already exists")
#         pass

# # 遍历文件列表
# for file_path in source_files:
#     directory = os.path.abspath(os.path.dirname(file_path))
#     while len(directory) > len(os.getcwd()):
#         create_init_if_not_exists(directory)
#         directory = os.path.dirname(directory)

source_files_pyx = []

# 遍历文件列表并更改扩展名
for file in source_files:
    if os.path.exists(file):
        # 分离文件的目录和文件名
        file_dir, file_name = os.path.split(file)
        # 分离文件名和扩展名，并更改扩展名为.pyx
        name, _ = os.path.splitext(file_name)
        new_file = os.path.join(file_dir, name + '.pyx')
        os.rename(file, new_file)
        source_files_pyx.append(new_file)

print(f"source_files_pyx: {source_files_pyx}")

setup(
    ext_modules=cythonize(source_files_pyx, language_level="3"),  # 指定编译的文件
    script_args=['build_ext', '--inplace']  # 指定输出目录
)


setup(
    name=f"pySimulation",
    version=ver,
    author="ZHANG KANGKANG",
    author_email="",
    description="",
    keywords="",
    packages=find_packages(),
    # setup_requires="" 指定安装的依赖，同样可以使用pip freeze > requirements.txt 生成依赖
    dependency_links=[],  # 添加依赖链接
    include_package_data = True,
    package_data={
        '': ['*.txt', '*.yaml', '*.yml', '*.csv', '*.docx', '*.pdf'],
    },
    install_requires=[
        "numpy"
    ]
)