from setuptools import setup

package_name = 'overtake_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='F1Tenth Overtaking Controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imm_filter = overtake_controller.imm_filterpy:main',
            'interceptor = overtake_controller.interceptor:main',
            'overtake = overtake_controller.overtake:main',
            'ego_ftg = overtake_controller.ego_ftg:main',
        ],
    },
)