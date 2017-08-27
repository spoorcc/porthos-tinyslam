[![Build Status](https://travis-ci.org/spoorcc/porthos-tinyslam.svg?branch=master)](https://travis-ci.org/spoorcc/porthos-tinyslam)
[![license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

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

