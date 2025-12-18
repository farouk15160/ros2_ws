import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'visiona_firmware'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        # Exclude .pio directory
        if '.pio' in directories:
            directories.remove('.pio')
        for filename in filenames:
            paths.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))
    return paths

extra_files = package_files('firmware')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        ('share/' + package_name, ['package.xml']),
    ] + extra_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farouk',
    maintainer_email='farouk15160@gmail.com',
    description='Firmware package for Visiona robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flash_firmware = visiona_firmware.firmware_tools:flash',
            'monitor_firmware = visiona_firmware.firmware_tools:monitor',
        ],
    },
)
