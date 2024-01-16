from setuptools import find_packages, setup

package_name = 'localization_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
