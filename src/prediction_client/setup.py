from setuptools import setup

package_name = 'prediction_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hkclr',
    maintainer_email='ycli@hkclr.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_franka_motion_read_states = prediction_client.test_franka_motion_read_states:main',
        'test_franka_motion_server_vision = prediction_client.test_franka_motion_server_vision:main',
        'test_franka_motion_server_vision_predict = prediction_client.test_franka_motion_server_vision_predict:main',
        ],
    },
)
