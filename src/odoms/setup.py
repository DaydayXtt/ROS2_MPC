from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'odoms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 可选：添加rviz文件
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
        # 可选：添加URDF和mesh文件
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'meshes'),
         glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes'),
         glob('meshes/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dayday22',
    maintainer_email='1229824413@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'odom_differential_node = odom_differential.odom_differential_node:main',
        'odom_integrator = odoms.odom_integrator:main',
        ],
    },
)
