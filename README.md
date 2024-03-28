# KUKA app
This repository contains personal C++ applications using KUKA LBR iiwa14. 

The application uses [Exp[licit]-cpp](https://github.com/explicit-robotics/Explicit-cpp) and [Exp[licit]-FRI](https://github.com/explicit-robotics/Explicit-FRI). Details can be checked in the [**explicit-robotics** Github repository](https://github.com/explicit-robotics).

The code includes applications for:
1. [Drawing and erasing task](./draw_and_erase/). [[Youtube Video]](https://youtu.be/zc1vlX_XmR8)
2. [Modular Imitation Learning](./draw_and_erase/). [[Youtube Video]](https://youtu.be/sARKQHFkkdA)

The data used for these applications are from [DMP-MATLAB Github repository](https://github.com/mosesnah-shared/DMP-MATLAB). Feel free to reach out to me for questions and details to generate the data.

## Overview of Software

### *Explicit-cpp*
This software contains a submodule of [Exp[licit]-cpp](https://github.com/explicit-robotics/Explicit-cpp). Exp[licit]-cpp in turn contains a submodule of [Eigen version 3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0). To update the submodules, type:
```
    git submodule update --init --recursive
```
You always need to check for Explicit-cpp updates. For that, type:
```
    git submodule update --remote
```
To run Exp[licit]-FRI, a .so-file of Exp[licit]-cpp has to be compiled. This can be done by running:
```
    make -f Makefile-lib
```

 Note that the intial source files are created for a "Media Flange Touch." If needed, please adapt the Flange Position in *exp_robots.cpp*.  

### *myFRIClient*
This library is provided by KUKA and establishes a state-machine for communication between the Client-PC and the Sunrise controller. The software uploaded here was extracted from a Sunrise 1.17-version. For compilation, please refer to the README-file inside the folder.


All C++ source codes  include an iir-filter, with coefficients determined [here](http://www.winfilter.20m.com/). The filter is needed to activate the robot's build-in friction compensation. Before sending the torques, we add a simple mean filter to smooth out the torque signals. The basic application calculates the Forward Kinematics, Jacobian Matrix, and Mass matrix of the robot and prints the calculational effort. 


# Explicit-MATLAB
If you are interested in simulating the KUKA robot first, check out [Explicit-MATLAB](https://github.com/explicit-robotics/Explicit-MATLAB).