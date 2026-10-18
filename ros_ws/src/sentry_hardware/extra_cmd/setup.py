from setuptools import find_packages, setup

package_name = 'extra_cmd'

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
    maintainer='gzu-prink',
    maintainer_email='agr.thu23@gzu.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_to_udp = extra_cmd.cmd_vel_to_udp:main',
            'cmd_vel_udp_to_udp = extra_cmd.cmd_vel_udp_to_udp:main',
        ],
    },
)
