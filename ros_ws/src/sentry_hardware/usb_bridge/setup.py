from setuptools import setup

package_name = 'usb_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/usb_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/usb_bridge.yaml']),
        ('share/' + package_name + '/udev', ['udev/99-sentry-stm32.rules']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='argen',
    maintainer_email='argen@todo.todo',
    description='Jetson <-> STM32 USB CDC bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'usb_bridge = usb_bridge.usb_bridge_node:main',
            'cmd_vel_to_unified = usb_bridge.cmd_vel_to_unified:main',
            'keyboard_teleop = usb_bridge.keyboard_teleop:main',
        ],
    },
)
