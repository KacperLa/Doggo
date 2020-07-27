from setuptools import setup

package_name = 'py_pubsub'

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
    maintainer='autodollyv2',
    maintainer_email='autodollyv2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_base_driver = py_pubsub.drive_base_driver_node:main',
	    'doggo_base = py_pubsub.doggo_base:main',
            'wviz = py_pubsub.wviz:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'fsm = py_pubsub.FSM:main',
            'client = py_pubsub.client:main',
        ],
    },
)
