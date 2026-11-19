from setuptools import setup
import os
from glob import glob

package_name = 'bot_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CuoiKiHRI',
    maintainer_email='cuoikihri@example.com',
    description='This package collects the data of the bot simulations and computes different metrics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_results = bot_evaluation.process_results:main',
            'bot_evaluation_node = bot_evaluation.bot_evaluation_node:main',
            'bot_action_monitor_node = bot_evaluation.action_monitor_node:main',
            'bot_teleop_monitor_node = bot_evaluation.teleop_monitor_node:main'
        ],
    },
)
