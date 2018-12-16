####################Voraussetzung###############
 CASADI-paket

#################### Main.py ################
Implementiert die Simulation und dient als Hauptprogramm

################## mpc.py #############################
Implementiert die Optimierung + Sim(.) + In(.)
----Achtung----- 
Die Parameter sehen etwa anders als in der Arbeit 

################# road.py ############################
Implementiert die verwendete Koordinatentransformation, Rotationsmatrix, verwaltet den Referenzweg 
und implementiert die dynamische Optimierung

################ hinderniss.py #######################
Modeliert die Hindernisse und implementiert die bei der dynamischen Programmierung benötigten funktionen 

############### Fahrstrecke ##############################
Implementiert die Teststrecke 

############### Programm starten ##################################
>> python3 main.py
