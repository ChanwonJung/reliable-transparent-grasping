from setuptools import find_packages, setup

package_name = 'depth_restoration_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/depth_restoration_params.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Jaewon Heo',
    maintainer_email='jaewonheo1101@gmail.com',
    description='Scan-matched HTTP depth restoration for an EE RGB-D camera.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'depth_restoration_node = depth_restoration_ros.node:main',
    ]},
)
