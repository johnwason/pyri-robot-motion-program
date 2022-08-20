from setuptools import setup, find_packages, find_namespace_packages

setup(
    name='pyri-robotics-motion-program',
    version='0.1.0',
    description='PyRI Teach Pendant Robotics Motion Program Package',
    author='John Wason',
    author_email='wason@wasontech.com',
    url='http://pyri.tech',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    zip_safe=False,
    install_requires=[
        'pyri-common',
        'pyri-robotics'
    ],
    entry_points = {
        'pyri.plugins.sandbox_functions': ['pyri-robotics-sandbox-functions=pyri.robotics_motion_program.sandbox_functions:get_sandbox_functions_factory'],
        'pyri.plugins.blockly': ['pyri-robotics-plugin-blockly=pyri.robotics_motion_program.blockly:get_blockly_factory'],
        'pyri.plugins.webui_server': ['pyri-robotics-motion-program-webui-server=pyri.robotics_motion_program.webui_server:get_webui_factory']
    }
)