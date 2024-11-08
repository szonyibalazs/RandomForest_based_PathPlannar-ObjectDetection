from setuptools import setup

package_name = 'object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BalazsSzonyi',
    maintainer_email='szonyibali@gmail.com',
    description='object_detector_node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node = object_detector.object_detector_node:main'
        ],
    },
)