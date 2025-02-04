import os
from glob import glob
from setuptools import setup

package_name = 'sdv_code'

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
            'sdv_node = sdv_code.sdv_node:main',
            'throttle_module = sdv_code.throttle_module:main',
            'sdv_main = sdv_code.sdv_main:main',
            'sdv_vision = sdv_code.sdv_vision:main',
            'publish_images = sdv_code.publish_images:main',
            'lane_detection_publisher = LaneNet.lane_detection_publisher:main'
        ],
    },
)
