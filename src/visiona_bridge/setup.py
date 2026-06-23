import os
from glob import glob
from setuptools import setup

package_name = 'visiona_bridge'

setup(
    name=package_name,
    version='6.0.0',
    packages=[
        package_name,
        package_name + '.hardware',
        package_name + '.ros2_interface',
        package_name + '.gui',
        package_name + '.state',
        package_name + '.visual_servoing',
        # Jarvis AI pipeline
        package_name + '.perception',
        package_name + '.world_model',
        package_name + '.llm',
        package_name + '.uraf',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'templates'),
            glob(os.path.join(package_name, 'templates', '*.html'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'robots'),
            glob(os.path.join('config', 'robots', '*.yaml'))),
        (os.path.join('share', package_name, 'static', 'css'),
            glob(os.path.join('visiona_bridge', 'static', 'css', '*.css'))),
        (os.path.join('share', package_name, 'static', 'js'),
            glob(os.path.join('visiona_bridge', 'static', 'js', '*.js'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'world'),
            glob(os.path.join('world', '*.world'))),
        (os.path.join('share', package_name, 'community'),
            glob(os.path.join('community', '*.yaml'))),
        (os.path.join('share', package_name, 'plugins', 'visiona_fal'),
            ['plugins/visiona_fal/plugin.yaml']),
        (os.path.join('share', package_name, 'resource'),
            ['resource/visiona_bridge']),
        # Jarvis LLM prompt templates
        (os.path.join('share', package_name, 'llm', 'prompt_templates'),
            glob(os.path.join(package_name, 'llm', 'prompt_templates', '*.txt'))),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'flask_socketio',
        'flask_cors',
        'pyserial',
        'ament_index_python',
        'requests',
        'pyyaml',
        'numpy',
    ],
    zip_safe=True,
    maintainer='farouk',
    maintainer_email='farouk15160@gmail.com',
    description='Visiona Robotics Studio — ROS2 bridge, URAF foundation, JARVIS AI, web GUI.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=[
        'scripts/moveit_homer.py',
        'scripts/command_forwarder.py',
        'scripts/cartesian_controller.py',
        'scripts/uraf_plugin.py',
    ],
    entry_points={
        'console_scripts': [
            # Core
            'web_gui = visiona_bridge.web_gui_node:main',
            # Motion
            'visual_servo_node = visiona_bridge.visual_servoing.visual_servo_node:main',
            'simple_ik_solver = visiona_bridge.simple_ik_solver:main',
            # Mapping
            'colored_map = visiona_bridge.colored_map_node:main',
            # ─── Jarvis AI pipeline ───────────────────────────────────────
            # Perception
            'jarvis_object_detector = visiona_bridge.perception.object_detector_node:main',
            'jarvis_segmentation    = visiona_bridge.perception.segmentation_node:main',
            'jarvis_pose_estimator  = visiona_bridge.perception.pose_estimator_node:main',
            # World Model
            'jarvis_world_model     = visiona_bridge.world_model.world_model_node:main',
            # LLM Planner + Executor
            'jarvis_llm_planner     = visiona_bridge.llm.llm_planner_node:main',
            'jarvis_action_executor = visiona_bridge.llm.action_executor_node:main',
            # URAF foundation
            'uraf_hardware_discovery = visiona_bridge.uraf.hardware_discovery_node:main',
            'uraf_health_monitor = visiona_bridge.uraf.health_monitor_node:main',
            'uraf_self_healing = visiona_bridge.uraf.self_healing_node:main',
            'uraf_learning_agent = visiona_bridge.uraf.learning_agent_node:main',
            'uraf_multi_robot_coordinator = visiona_bridge.uraf.multi_robot_coordinator_node:main',
            'uraf_urdf_generator = visiona_bridge.uraf.urdf_generator_node:main',
            'uraf_digital_twin = visiona_bridge.uraf.digital_twin_node:main',
            'uraf_plugin_manager = visiona_bridge.uraf.plugin_manager_node:main',
            'uraf_community_library = visiona_bridge.uraf.community_library_node:main',
            'uraf_safety_monitor = visiona_bridge.uraf.safety_monitor_node:main',
        ],
    },
)
