import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="curiouschip_breadboard",
    version="0.0.1",
    author="Jason Frame",
    description="Client library for Curious Chip Breaboard",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8',
    py_modules=["curiouschip_breadboard"],
    package_dir={'':'curiouschip_breadboard/src'},
    install_requires=[]
)
