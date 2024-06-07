import os
from glob import glob
from setuptools import setup

package_name = 'line_sen'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='movehoon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'line_sen_node = line_sen.line_sen_node:main',
            'video_pub_node = line_sen.video_pub_node:main'
=======
            'detect_line = line_sen.detect_line:main',
            'pcan_manager = line_sen.pcan_manager:main'
>>>>>>> bd343e170dd9eae011e6e68852ea22636295ef55
        ],
    },
)
