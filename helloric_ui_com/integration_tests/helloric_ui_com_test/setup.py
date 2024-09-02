from setuptools import setup

package_name = 'helloric_ui_com_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Andreas Bresser',
    author_email='andreas.bresser@dfki.de',
    maintainer='Andreas Bresser',
    maintainer_email='andreas.bresser@dfki.de',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD-3',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal publishers using rclpy.',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotion_publisher ='
            ' helloric_ui_test.emotion_publisher:main',
        ],
    },
)