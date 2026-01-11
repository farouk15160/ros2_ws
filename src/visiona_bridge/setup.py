import os
from glob import glob
from setuptools import setup

package_name = 'visiona_bridge'

setup(
    name=package_name,
    version='5.0.0',  # Updated version for restructure
    packages=[
        package_name,
        package_name + '.hardware',
        package_name + '.ros2_interface',
        package_name + '.gui',
        package_name + '.state',
        package_name + '.llm_control',
        package_name + '.visual_servoing',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
       
         
        
        (os.path.join('share', package_name, 'templates'), 
         glob(os.path.join(package_name, 'templates', '*.html'))),
        
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.xacro'))),

        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
            
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
    scripts=['scripts/moveit_homer.py', 'scripts/command_forwarder.py', 'scripts/cartesian_controller.py'],
    entry_points={
        'console_scripts': [
            'web_gui = visiona_bridge.web_gui_node:main',
            # LLM Control
            'llm_task_planner = visiona_bridge.llm_control.llm_task_planner:main',
            'vla_action_generator = visiona_bridge.llm_control.vla_action_generator:main',
            'task_executor = visiona_bridge.llm_control.task_executor:main',
            # Visual Servoing
            'visual_servo_node = visiona_bridge.visual_servoing.visual_servo_node:main',
            'simple_ik_solver = visiona_bridge.simple_ik_solver:main',
            # Mapping
            'colored_map = visiona_bridge.colored_map_node:main',
        ],
    },
)
