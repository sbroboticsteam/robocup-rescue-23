from setuptools import find_packages, setup

package_name = 'web_publishers'

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
    maintainer='gteref',
    maintainer_email='giovanni.terefi@stonybrook.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = web_publishers.imgpublisher:main',
            'random_string_publisher = web_publishers.stringpublisher:main',
            'video_publisher = web_publishers.videopublisher:main',
            'pointcloud2_publisher = web_publishers.pointcloud2publisher:main'
        ],
    }
)
