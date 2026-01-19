from setuptools import setup, find_packages

package_name = 'robot2_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chandan',
    maintainer_email='chandan@example.com',
    description='Robot 2 control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_pub = robot2_control.test_pub:main',
        ],
    },
)