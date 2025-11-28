from setuptools import setup

package_name = 'command_server_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='user@example.com',
    description='Command server node providing /compute_command service.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_server = command_server_pkg.command_server:main',
        ],
    },
)


