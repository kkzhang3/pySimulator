# coding=utf8
from setuptools import setup
from setuptools import find_packages
from modules.__init__ import __version__

setup(
    name=f"pySimulation",
    version=__version__,
    author="ZHANG KANGKANG",
    author_email="",
    description="",
    keywords="",
    packages=find_packages(),
    # setup_requires="" 指定安装的依赖，同样可以使用pip freeze > requirements.txt 生成依赖
    dependency_links=[],  # 添加依赖链接
    include_package_data = True,
    install_requires=[
        "numpy"
    ]
)