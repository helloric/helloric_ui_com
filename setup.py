#!/usr/bin/env python
from setuptools import find_packages, setup


package_name = 'helloric_ui_com'


with open('README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()

setup(
    name=package_name,
    description="HelloRIC UI FastAPI ROS 2 Communication server",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/brean/python-pathfinding",
    version="0.0.1",
    license="BSD-3",
    author="Andreas Bresser",
    packages=find_packages(),
    tests_require=[],
    include_package_data=False,
    install_requires=["fastapi"],
    entry_points={
        'console_scripts': [
            'helloric_ui_com = helloric_ui_com.cli:main',
        ],
    },
)