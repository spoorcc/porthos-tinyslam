[![Build Status](https://travis-ci.org/spoorcc/porthos-tinyslam.svg?branch=master)](https://travis-ci.org/spoorcc/porthos-tinyslam)

# Porthos Tiny-slam

This repository is a clone from [the SVN repo of tinyslam provided by OpenSLAM](https://svn.openslam.org/data/svn/tinyslam/).
I will adapt this to fit in my Porthos project

tinySLAM is Laser-SLAM algorithm which has been programmed in less than 200 lines of C-language code.

## Compilation
```
mkdir -p bld
cd bld 
cmake ..
make
```

## Run tests
```
cd bld/test
./test_loop_closing
```

# Further information

## Original Authors
Bruno Steux; Oussama El Hamzaoui;

## Papers Describing the Approach
[Bruno Steux and Oussama El Hamzaoui: tinySLAM : a SLAM Algorithm in less than 200 lines of C code, Accepted for the International Conference on Control, Automation, Robotics and Vision (ICARCV), 2010](http://ieeexplore.ieee.org/abstract/document/5707402/) ([PDF](https://pdfs.semanticscholar.org/1c7e/7af4388b17b137badc6ec19e9724e6bd91e4.pdf))

## License Information
This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
The authors allow the users of OpenSLAM.org to use and modify the source code for their own research. Any commercial application, redistribution, etc has to be arranged between users and authors individually and is not covered by OpenSLAM.org.

tinySLAM is licenced under the MIT License.

