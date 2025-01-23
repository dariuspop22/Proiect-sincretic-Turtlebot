from setuptools import find_packages, setup

package_name = 'tb3_tele'

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
    maintainer='emil',
    maintainer_email='emil@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = tb3_tele.pub:main',
            'tele = tb3_tele.remote:main',
            'capture = tb3_tele.capture:main',
            'rotate = tb3_tele.rotate:main',
            'rotate2 = tb3_tele.rotate2:main'
        ],
    },
)
