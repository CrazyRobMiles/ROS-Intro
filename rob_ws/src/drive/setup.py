from setuptools import find_packages, setup

package_name = 'drive'

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
    maintainer='rob',
    maintainer_email='rob@robmiles.com',
    description='Provides a motor controller, motion sensor and a keyboard tester',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard = drive.keyboard_member_function:main',
            'motors = drive.motors_member_function:main',
            'motion = drive.motion_sensor_member_function:main'
        ],
    },
)
