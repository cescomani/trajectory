# Python implementation
This folder presents a python implementation of the trajectory planing concept.     
# Recommended tools
The follow tools are needed to run the implemented example:
- [x] CASADI-package: 
python version of the package can be downloaded or cloned from [GitHub Pages](https://github.com/casadi/casadi/tree/master/docs). No need to install it(one can just use it in the same folder with others scripts). The recommended installation method is to build from the source. One schould therfore edit the CMakeCache.txt file before running the installer by adapting python to the correct version(only if you have many Python versions) and setting the parameters WITH_IPOPT=ON and WITH_PYTHON3=ON so that CASADI can integrate the IPOPT-solver. 
- [x] Python 3:
- [x] IpOpt:
non-lineare and non convexe problems solver. It needs to be installed as explained [here](https://www.coin-or.org/Ipopt/documentation/node10.html)(a library for large-scale nonlinear optimization)
## ###################main.py################
### With this script one can perform an example of lane keeping situation using the implemented MPC.

## ################mpc.py#############################
### This script designs the model predictive control

## ################road.py############################
### Implements the used coordinate transformation, rotation matrix, manages the reference path and implements the dynamic 
### optimization.

## ###############obstacle.py#######################
### It designs obstacles as rectangle

## ##############fahrstrecke##############################
### Design of a test track consisting of several sections in which different tracking movements can be performed(Lane keeping). 

## ######################plotrecorder.py####################
### A simple Python module for recording matplotlib animation and convert to mp4 video(this module just works for UNIX-Users)

## ##############run the main script##################################
### >> python3 main.py
