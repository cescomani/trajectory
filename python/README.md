# Python implementation
This folder presents a python implementation of the trajectory planing concept.     
# Recommended tools
The follow tools are needed to run the implemented example:
- [x] CASADI-package 
python version of the package can be downloaded or cloned form [GitHub Pages](https://github.com/casadi/casadi/tree/master/docs). No need to inatll it.
- [x] Python 3
- [x] IpOpt
non-lineare and non convexe problems solver. It needs to be installed as explained [here](https://www.coin-or.org/Ipopt/documentation/node10.html)




# ###################Voraussetzung###############
# CASADI-paket

# ###################Main.py################
# Implementiert die Simulation und dient als Hauptprogramm

##################mpc.py#############################
# Implementiert die Optimierung + Sim(.) + In(.)
# ----Achtung----- 
# Die Parameter sehen etwa anders als in der Arbeit 

# ################road.py############################
# Implementiert die verwendete Koordinatentransformation, Rotationsmatrix, verwaltet den Referenzweg 
# und implementiert die dynamische Optimierung

# ###############obstacle.py#######################
# Implements obstacles and the methode of dynamic programming 

# ##############Fahrstrecke##############################
# Implementiert die Teststrecke 

# ##############Programm starten##################################
# >> python3 main.py
