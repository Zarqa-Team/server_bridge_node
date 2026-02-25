from setuptools import setup

package_name = 'server_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='ROS2 bridge node for competition simulation server',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bridge = server_node.bridge:main',
        ],
    },
)
