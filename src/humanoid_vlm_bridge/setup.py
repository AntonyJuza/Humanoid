from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_vlm_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Bridge between ROS2 and Qwen2.5-Omni VLM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_node = humanoid_vlm_bridge.vlm_node:main',
            'image_processor = humanoid_vlm_bridge.image_processor:main',
            'audio_processor = humanoid_vlm_bridge.audio_processor:main',
            'intent_executor = humanoid_vlm_bridge.intent_executor:main',
        ],
    },
)