# Python implementation
This folder presents a python implementation of the trajectory planing concept.     
# Recommended tools
The follow tools are needed to run the implemented example:
- [x] CASADI-package: 
python version of the package can be downloaded or cloned from [GitHub Pages](https://github.com/casadi/casadi/tree/master/docs). No need to install it(one can just use it in the same folder with oder skripts). The recommended installation way is to build from the source. One schould therfore edite the CMakeCache.txt file before performing the installion, by adapting python to the right version(just needed if one has many python versions) and seting the parameter WITH_IPOPT=ON to allow CASADI to integrate that solve 
- [x] Python 3:
- [x] IpOpt:
non-lineare and non convexe problems solver. It needs to be installed as explained [here](https://www.coin-or.org/Ipopt/documentation/node10.html)
## ###################Main.py################
## With this script one can perform an example of lane keeping stuation using the the implemented MPC.

## ################mpc.py#############################
## This script designs the model predictive control

## ################road.py############################
## Implements the used coordinate transformation, rotation matrix, manages the reference path and implements the dynamic 
## optimization.

## ###############obstacle.py#######################
## It designs obstacles as rectangle and implements the methode of dynamic programming 

## ##############Fahrstrecke##############################
## Design of a test track consisting of several sections in which different tracking movements can be performed(Lane keeping). 

## ##############run the main script##################################
## >> python3 main.py
