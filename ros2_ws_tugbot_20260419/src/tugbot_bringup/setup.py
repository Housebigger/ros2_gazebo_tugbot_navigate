from glob import glob
from setuptools import setup

package_name = 'tugbot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.*')),
        ('share/' + package_name + '/config', glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyh',
    maintainer_email='hyh@example.com',
    description='Layered launch entry points for minimal tugbot visual lane simulation.',
    license='MIT',
    tests_require=['pytest'],
)
