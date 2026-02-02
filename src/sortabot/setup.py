from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sortabot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Install launch files
        ('share/' + package_name + '/launch', [
            'launch/sortabot.launch.py'
        ]),


        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='benjamin',
    maintainer_email='benjamin@todo.todo',
    description='Sortabot procedural world generator and controllers',
    license='MIT',
    entry_points={
        'console_scripts': [
            'world_manager = sortabot.world_manager:main',
            'robot_controller = sortabot.robot_controller:main',
            'childabot = sortabot.childabot:main',
            'keyboard_reset = sortabot.keyboard_reset:main',
        ],
    },
)