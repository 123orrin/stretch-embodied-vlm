from setuptools import find_packages, setup

package_name = 'mic'

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
    maintainer='Gordon Tan',
    maintainer_email='gordon.tan111@gmail.com',
    description='Package for use with RODE mics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_AB = mic.get_A_B_from_prompt:main',
            'local_STT = mic.local_STT:main'
        ],
    },
)
