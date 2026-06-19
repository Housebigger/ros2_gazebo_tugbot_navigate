from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tugbot_maze'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Housebigger',
    maintainer_email='housebigger@example.com',
    description='Maze world metadata, exit monitoring, and autonomous maze-solving nodes for the Tugbot (reactive wall-following solver, plus Trémaux/frontier/DFS explorers and guided-corridor navigation).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_goal_monitor = tugbot_maze.maze_goal_monitor:main',
            'maze_image_to_world = tugbot_maze.maze_image_to_world:main',
            'maze_explorer = tugbot_maze.maze_explorer:main',
            'maze_solver = tugbot_maze.maze_solver:main',
            'wall_follow_solver = tugbot_maze.wall_follow_solver:main',
            'flood_fill_solver = tugbot_maze.flood_fill_solver:main',
            'phase64_5_first_dispatch_visual_overlay = tugbot_maze.phase64_5_first_dispatch_visual_overlay:main',
            'phase67_goal1_timeout_visual_overlay = tugbot_maze.phase67_goal1_timeout_visual_overlay:main',
        ],
    },
)
