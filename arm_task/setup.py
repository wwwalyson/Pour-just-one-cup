from setuptools import setup
from glob import glob
import os

package_name = 'arm_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='silly',
    maintainer_email='silly@todo.todo',
    description='Tea task scheduler: orchestrates arm movements for tea/water pouring',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tea_task_node = arm_task.tea_task_node:main',
        ],
    },
)
