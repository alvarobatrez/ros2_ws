from setuptools import setup

package_name = 'templates_py'

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
    maintainer='alvaro',
    maintainer_email='alvarobatrez27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_template = templates_py.publisher_template:main',
            'subscriber_template = templates_py.subscriber_template:main',
            'server_template = templates_py.server_template:main',
            'client_template = templates_py.client_template:main',
            'simple_action_server_template = templates_py.simple_action_server_template:main',
            'action_server_template = templates_py.action_server_template:main',
            'action_client_template = templates_py.action_client_template:main'
        ],
    },
)
