from setuptools import find_packages, setup

package_name = 'lsy_robot_dev'

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
    maintainer='Orrin Dahanaggamaarachchi',
    maintainer_email='orrin.dahanaggamaarachchi@mail.utoronto.ca',
    description='Learning Systems Lab Stretch Development',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = lsy_robot_dev.test_node:main',
            'vision_language_service = lsy_robot_dev.vision_language_client:main',
            'vlm_teleop = lsy_robot_dev.vlm_teleop:main',
            'vlm_teleop_tmp = lsy_robot_dev.vlm_teleop_tmp:main',
            'test_service = lsy_robot_dev.test_service:main'
        ],
    },
)
