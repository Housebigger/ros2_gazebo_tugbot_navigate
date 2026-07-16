from setuptools import find_packages, setup

package_name = 'tugbot_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Housebigger',
    maintainer_email='housebigger@example.com',
    description='Frontier exploration node for Tugbot SLAM + Nav2 mapping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = tugbot_exploration.frontier_explorer:main',
        ],
    },
)
