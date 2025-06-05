from setuptools import find_packages, setup

package_name = 'vazquez_ejer1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/counter_demo.launch.py'])
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
            'counter_publisher = vazquez_ejer1_pkg.counter_publisher:main',
            'counter_subscriber = vazquez_ejer1_pkg.counter_subscriber:main',
        ],
    },
)
