from setuptools import find_packages, setup

package_name = 'lsy_laptop_dev'

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
    maintainer='orrin',
    maintainer_email='orrin.dahanaggamaarachchi@mail.utoronto.com',
    description='Learning systems Lab Stretch Laptop Development',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_language_server = lsy_laptop_dev.vision_language_server:main',
            'preprompt_server = lsy_laptop_dev.preprompt_server:main'
        ],
    },
)
