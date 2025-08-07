from setuptools import find_packages, setup

package_name = 'pascal_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'scipy'
    ],
    zip_safe=True,
    maintainer='pascal',
    maintainer_email='pascal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_z_estimate = pascal_odom.visual_z_estimate:main',
        ],
    },
)
