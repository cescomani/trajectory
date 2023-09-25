from pylab import *
from source.road.fahrstrecke import *
import source.obstacle.obstacle as hinder
from source.road.road import erzeugHindernis 

########### Definition of the vehicles' parameters  ##################################
Nx = 5                              #Anzahl Zustände 
Nu = 2                              #Anzahl Stellgröße
ord_mpc = 3                         # Ordnung des Polynomes(Referenz)  
T_mpc = 3                           # jede Planung dauert 3s(Zeithorizont)
N_mpc = 15                          # Stellhorizont bzw. Prädiktionshorizont(Anzahl von Knotenvariablen)
Q_mpc = np.diag([10,10, 1, 50, 10, 0]) # Nicht mit der Q-Matrix in der Ausarbeitung vertauchen
R_mpc = np.diag( [1, 1] )              # 
q = 5                                  # 
vmax = 5                            #Maximale Geschwindigkeit

########## Definition of the reference trajectory ####################################
temp1, temp2, temp3 = testStrecke1()
Xref = np.array([temp1[0],temp1[1]])

X0 = np.array([Xref[0,0], Xref[1,0], 0, 0, 0])
 
 
######### Definition of the car/obstacles in triangle shapes ##################################
Hdim, Cdim = np.array([2.5, 1.8]), np.array([1.5, 0.5])# Größe eines Hindernisses und des Fahrzeugs
car = hinder.RechteckHindernis([0,0],Cdim[0], Cdim[1],0) #Das Ego-Fahrzeug
car1, car2, car3 = erzeugHindernis(Xref,100, Hdim,0.9,1), erzeugHindernis(Xref,200, Hdim,0.9), erzeugHindernis(Xref,300, Hdim,0.9,1)
car4, car5, car6 = erzeugHindernis(Xref,400, Hdim,0.9),  erzeugHindernis(Xref,500, Hdim,0.9), erzeugHindernis(Xref,600, Hdim,0.9) 
car7, car8, car9 = erzeugHindernis(Xref,700, Hdim,0.9,1), erzeugHindernis(Xref,800, Hdim,0.9), erzeugHindernis(Xref,900, Hdim,0.9)
car10, car11, car12 = erzeugHindernis(Xref,1000, Hdim,0.9,1), erzeugHindernis(Xref,1100, Hdim,0.9), erzeugHindernis(Xref,1200, Hdim,0.9,1)
car13, car14, car15 = erzeugHindernis(Xref,1300, Hdim,0.9), erzeugHindernis(Xref,1400, Hdim,0.9,1), erzeugHindernis(Xref,1500, Hdim,0.9)
car16, car17, car18 = erzeugHindernis(Xref,1600, Hdim,0.9,1), erzeugHindernis(Xref,1700, Hdim,0.9), erzeugHindernis(Xref,1800, Hdim,0.9, 1)
car19, car20, car21 = erzeugHindernis(Xref,1900, Hdim,0.9), erzeugHindernis(Xref,2100, Hdim,0.9,1), erzeugHindernis(Xref,2300, Hdim,0.9)
car22, car23, car24 = erzeugHindernis(Xref,2500, Hdim,0.9,1), erzeugHindernis(Xref,2700, Hdim,0.9), erzeugHindernis(Xref,2900, Hdim,0.9,1)
car25, car26, car27 = erzeugHindernis(Xref,3100, Hdim,0.9), erzeugHindernis(Xref,3300, Hdim,0.9,1), erzeugHindernis(Xref,3500, Hdim,0.9)
car28, car29, car30 = erzeugHindernis(Xref,3700, Hdim,0.9,1), erzeugHindernis(Xref,3900, Hdim,0.9), erzeugHindernis(Xref,4000, Hdim,0.9,1)
obstList_wk = [car1, car2, car3, car4, car5, car6, car7, car8, car9, car10, car11, car12, car13, car14, car15, \
               car16, car17, car18, car19, car20, car21, car22, car23, car24, car25, car26, car27, car28, car29, car30]
    
 
 # start conditions
def anfangsbedingungen(X0, Nx, N_mpc, nHin):
    Nsim = 890     
    tsoll = 0.1  # 
    tSolver  = 0.1
    eT = 0
    eT_1 = 0 
    X = np.zeros(( 20000 , Nx ))    #Alle globalen Zustände werden in X gespeichert
    x = np.zeros( (N_mpc+1, Nx) )   # Alle 5x15 Knotenvariablen auf einem Horizont im SK
    xwk = np.zeros( (N_mpc+1,Nx) )  # Alle 5x15 Knotenvariablen auf einem Horizont im WK
    w = 0.01*np.ones((8*nHin,))     # wird nur benötigt, wenn Hindernisse vorhanden sind
    X[0 ,:] = X0   
    return Nsim, tsoll, tSolver, eT, eT_1, X, x, xwk, w
    
####### Store recorders ########################
saveFramesIn = 'example'
