# coding=utf8
import os
from setuptools import setup
from Cython.Build import cythonize


from modules.__init__ import __version__


source_files = [
    'modules/gui/QtGUI.py',
]

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