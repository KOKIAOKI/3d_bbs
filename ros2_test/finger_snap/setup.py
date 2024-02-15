from setuptools import setup

package_name = 'finger_snap'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koki',
    maintainer_email='180447001@ccalumni.meijo-u.ac.jp',
    description='finger_snap',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finger_snap = finger_snap.finger_snap:main',
        ],
    },
)