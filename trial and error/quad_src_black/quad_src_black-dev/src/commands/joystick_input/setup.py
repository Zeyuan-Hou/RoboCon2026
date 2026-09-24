from setuptools import find_packages, setup

package_name = 'joystick_input'

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
    maintainer='guo',
    maintainer_email='3256646795@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_input_node = joystick_input.joystick_pub:main', 
            'joystick_handler_node = joystick_input.joystick_handler:main'
        ],
    },
)
