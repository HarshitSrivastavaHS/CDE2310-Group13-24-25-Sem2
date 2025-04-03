from setuptools import setup
import os
from glob import glob

package_name = 'cde2310'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Custom bringup package for robot autonomy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'thermal = cde2310.thermal:main',
        ],
    },
    data_files=[
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages', ['resource/cde2310']),
        ('share/' + package_name, ['package.xml']),
    ],
)

