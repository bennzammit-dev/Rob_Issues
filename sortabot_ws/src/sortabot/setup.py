from setuptools import setup

package_name = 'sortabot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/sortabot.world']),
        ('share/' + package_name + '/models', [
            'models/sortabot.sdf',
            'models/childabot.sdf',
            'models/sortabot.sdf'
        ]),
        ('share/' + package_name + '/launch', ['launch/sortabot.launch.py', 'launch/spawn_robots.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@example.com',
    description='Sortabot simulation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'childabot = sortabot.childabot:main',
            'robot_controller = sortabot.robot_controller:main',
            'world_manager = sortabot.world_manager:main',
        ],
    },
)