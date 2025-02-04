import os
from glob import glob
from setuptools import setup

package_name = 'xbox_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='A00825234@itesm.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_node = xbox_controller.xbox_node:main',
            'throttle_module = xbox_controller.throttle_module:main',
            'sdv_main = xbox_controller.sdv_main:main',
            'sdv_vision = xbox_controller.sdv_vision:main',
            'publish_images = xbox_controller.publish_images:main',
            'lane_detection_publisher = LaneNet.lane_detection_publisher:main'
        ],
    },
)
