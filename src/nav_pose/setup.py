from setuptools import setup

package_name = 'nav_pose'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imxq',
    maintainer_email='2645199416@qq.com',
    description='Through-poses slalom navigation mission package.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slalom_through_poses = nav_pose.slalom_through_poses:main',
        ],
    },
)
