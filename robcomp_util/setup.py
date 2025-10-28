from setuptools import find_packages, setup

package_name = 'robcomp_util'

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
    maintainer='borg',
    maintainer_email='dpsoler09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_odom = robcomp_util.test_odom:main",
            "test_laser = robcomp_util.test_laser:main",
            "andar = robcomp_util.andar:main",
            "goto = robcomp_util.goto:main",
        ],
    },
)
