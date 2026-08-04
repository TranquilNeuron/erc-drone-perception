from setuptools import find_packages, setup
from glob import glob

package_name = 'erc_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),   # <- added
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niranjan',
    maintainer_email='niranjan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_node = erc_perception.aruco_node:main',
            'probe_node = erc_perception.probe_node:main',
            'depth_check = erc_perception.depth_check:main',
        ],
    },
)
