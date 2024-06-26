from setuptools import find_packages, setup

package_name = 'templates_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alvaro',
    maintainer_email='alvarobatrez27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transmitter = templates_py.transmitter:main',
            'receiver = templates_py.receiver:main',
            'server = templates_py.server:main',
            'client = templates_py.client:main',
            'client_async = templates_py.client_async:main',
            'action_server = templates_py.action_server:main',
            'action_server_queue_goals = templates_py.action_server_queue_goals:main',
            'action_client = templates_py.action_client:main'
        ],
    },
)