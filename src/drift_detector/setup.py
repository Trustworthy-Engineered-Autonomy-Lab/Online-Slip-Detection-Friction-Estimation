from setuptools import setup

package_name = 'drift_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f1tenth',
    maintainer_email='f1tenth@todo.todo',
    description='Lightweight algorithm for detecting the loss of traction and estimating the friction coefficient from LIDAR and IMU during autonomous racing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'drift_detector = drift_detector.revised_detector:main'
            ],
    },
)
