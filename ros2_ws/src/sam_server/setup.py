from setuptools import find_packages, setup

package_name = 'sam_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sam_server.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/sam_server_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaewon Heo',
    maintainer_email='jaewonheo1101@gmail.com',
    description='Slow Brain segmentation — SAM 2.1 box prompts on the Seraph GPUs.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sam_mask_node = sam_server.sam_mask_node:main',
        ],
    },
)
