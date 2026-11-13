from setuptools import setup

package_name = 'sentry_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sentry',
    maintainer_email='lthzs@qq.com',
    description='Localization-driven safety supervisor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_supervisor = sentry_safety.localization_supervisor:main',
        ],
    },
)
