from setuptools import find_packages, setup

package_name = 'can_messenger'
submodules = 'can_messenger/module'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kohki',
    maintainer_email='kou-pen@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_messenger = can_messenger.can_messenger:main'
        ],
    },
)
