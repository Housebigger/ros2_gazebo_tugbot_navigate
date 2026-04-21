from pathlib import Path
from setuptools import setup

from tugbot_description.package_layout import collect_data_files

package_name = 'tugbot_description'
package_root = Path(__file__).resolve().parent

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=collect_data_files(package_root, package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyh',
    maintainer_email='hyh@example.com',
    description='Minimal packaged tugbot description for visual lane following.',
    license='MIT',
    tests_require=['pytest'],
)
