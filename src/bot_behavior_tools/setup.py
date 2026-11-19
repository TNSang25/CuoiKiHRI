from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bot_behavior_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/prompts', glob(f'{package_name}/prompts/*.txt')),
        ('share/' + package_name + '/templates', glob(f'{package_name}/templates/*.xml'))
    ],
    install_requires=[
        'setuptools',
        'PyYAML',
    ],
    extras_require={'llm': ['openai']},
    zip_safe=True,
    maintainer='CuoiKiHRI',
    maintainer_email='cuoikihri@example.com',
    description='LLM-based behavior tree and scenario generator for Bot simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_trees = bot_behavior_tools.generate_trees:main',
            'generator_cli = bot_behavior_tools.generator_cli:main',
        ],
    },
)
