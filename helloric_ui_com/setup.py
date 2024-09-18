from setuptools import find_packages, setup

package_name = 'helloric_ui_com'

with open('../README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'fastapi'],
    zip_safe=True,
    author="Andreas Bresser",
    description="HelloRIC UI FastAPI ROS 2 Communication server",
    long_description=long_description,
    long_description_content_type="markdown",
    license="BSD-3",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloric_ui_com = helloric_ui_com.cli:main',
        ],
    },
)
