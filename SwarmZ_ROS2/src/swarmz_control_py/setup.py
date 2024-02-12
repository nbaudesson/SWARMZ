import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarmz_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nicolasbaudesson_adm',
    maintainer_email='nicolas.baudesson@alten.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simu_processes = '+package_name+'.simu_processes:main',
            'simu_circle = '+package_name+'.simu_circle:main',
            'offboard_control_py = '+package_name+'.offboard_control:main',
            'mission_control = '+package_name+'.mission_control:main',
        ],
    },
)
