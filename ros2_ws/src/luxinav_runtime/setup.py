from setuptools import find_packages, setup


package_name = "luxinav_runtime"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/luxinav_runtime"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LuxiNav",
    maintainer_email="maintainer@example.com",
    description="Plugin catalog and run specification resolution for LuxiNav.",
    license="Apache-2.0",
)
