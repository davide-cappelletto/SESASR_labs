from setuptools import find_packages, setup

import os # This change was made to hopefully fix issues with the launch file.
from glob import glob # This change was made to hopefully fix issues with the launch file.

package_name = 'localization_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # This change was made to hopefully fix issues with the launch file.
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')), # This change was made to hopefully fix issues with the launch file.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giuseppe',
    maintainer_email='giuseppedeninarivera@gmail.com',
    description='localization_project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'EKF_node = localization_project.run_ekf_2:main',
            'recorder = localization_project.recorder:main',
            'recorder_alt = localization_project.recorder_alt:main'
        ],
    },
)
