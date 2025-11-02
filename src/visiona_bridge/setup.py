import os
from glob import glob
from setuptools import setup

package_name = 'visiona_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static/models'), 
         glob(os.path.join(package_name, 'static', 'models', '*.stl'))),
         
        
        (os.path.join('share', package_name, 'templates'), 
         glob(os.path.join(package_name, 'templates', '*.html'))),
        
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.xacro'))),
            
        (os.path.join('share', package_name, 'meshes'), 
            glob(os.path.join('meshes', '*.stl'))),
            
        (os.path.join('share', package_name, 'rviz'), 
            glob(os.path.join('rviz', '*.rviz'))),
            
        (os.path.join('share', package_name, 'resource'), 
            ['resource/visiona_bridge']),
],
    install_requires=['setuptools', 'flask', 'flask_socketio', 'pyserial', 'ament_index_python'], # Added dependencies
    zip_safe=True,
    maintainer='farouk',
    maintainer_email='farouk15160@gmail.com',
    description='ROS2 bridge for the Robot arm.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Entry point for the new Web GUI node
            'web_gui = visiona_bridge.web_gui_node:main',
            
        ],
    },
)

