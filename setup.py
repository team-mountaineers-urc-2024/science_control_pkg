import os 
from glob import glob 
from setuptools import find_packages, setup

package_name = 'science_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))) # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'dynamixel_control_node = science_control_pkg.dynamixel_control:main',

            'ocean_optics_spectrometer_node = science_control_pkg.ocean_optics_spectrometer:main',

            'science_motor_control_node = science_control_pkg.science_motor_control:main',

            'subsurface_motor_node = science_control_pkg.subsurface_motor:main',
        ],
    },
)
