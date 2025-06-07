from setuptools import find_packages, setup

package_name = 'vazquez_ejer2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/republisher.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Define the console scripts for the action server and client
            'republisher_server = vazquez_ejer2_pkg.republisher_server:main',
            'republisher_client = vazquez_ejer2_pkg.republisher_client:main',
        ],
    },
)