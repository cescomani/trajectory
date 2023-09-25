import time
from pylab import *
from sys import path
path.append("source/")

#import matplotlib.pyplot as plt
from source.road.road import *
import source.obstacle.obstacle as hinder
from source.mpc.mpc import *
from source.road.fahrstrecke import *
from source.recorder.plotrecorder import *
from konfigParameters import *
   
######################### Durchführung der MPC ###############################################
# Solver erzeugen
hin = [] # Am Anfang wird kein Hindernis berücksichtigt
nHin = len(hin) #Anzahl von relevanten Hindernisse
solver, varlb, varub, conlb, conub, varguess, var, integrierer=optimierer(Q_mpc, R_mpc, T_mpc, q, N_mpc, ord_mpc, Cdim, 0, vmax, M=1)

Nsim, tsoll, tSolver, eT, eT_1, X, x, xwk, w =   anfangsbedingungen(X0, Nx, N_mpc, nHin)                
   
##Achsen Vorbereiten
fig = plt.figure()
ax = fig.add_subplot(111)
wegA = ax.plot(temp2[0], temp2[1], temp3[0], temp3[1], color='0.75', linewidth=0.9)  #Gesamtstrecke
pathAx = ax.plot([],[],'.y', color='0.90', linewidth=0.1)                            #Aktueller Referenzweg
carAx = ax.plot([],[])                                                               #Achse zum Zeichen des Fahrzeugs 
richtungA = ax.plot([],[])                                                           #nicht so wichtig 

#Alle Hindernisse Darstellen
color ='0.4'   
a = 1
for elt in obstList_wk:
    Y = elt.getXY()      
    c = elt.getCenter()
    ax.fill(Y[0], Y[1], color)
    ax.text(c[0]-0.5,c[1]-0.2,'H'+str(a), fontsize=15)
    a +=1 
   
obsAx  = ax.text(0, 0, '', fontsize=15, color='blue')        #Anzahl aktueller Hindernisse
gridAx = ax.plot([], [], '>b')                               #Gitter
zoptAx = ax.plot([], [], '<k')                               #Mögliche Zielpunkte                       
xoptAx = ax.plot([], [], '.g')                               #Geplante Trajektorie
   

zielAx = ax.plot([], [], 'or', linewidth=15)                 #Zielpunkt
speedCarAx = ax.text(0, 0, '', fontsize=15, color='blue')    #Aktuelle Geschwindigkeit
plt.axis("equal")  # Gleiche Auflösung auf beiden Achsen                                          
   
plt.grid()

ursp_alt = 0 #Alter Ursprung von SK 

#makeRecord = False
for t in range( Nsim ):   
    # Straßenkoordinate festlegen. SK liegt in ursp und wird um theta bezüglich WK gederht
    # mapX ist der aktuelle Streckenabschnitt und obstList eine Liste aller Hindernisse auf diesem Abschnitt
    mapX, ursp, ursp_ind, theta, obstList = akuellRouteReferenz(Xref,X[t,0:2], obstList_wk, ursp_alt)
    ursp_alt = ursp_ind  
    K = polynomInFahrzeug( mapX, ord_mpc, ursp, theta)  #Streckenabschnitt als Polynom definieren
    
    #Zeitabweichung
    eT_2 = eT_1
    eT_1 = eT
    eT = tsoll-tSolver 
    
    ##############################Zielpunkt definieren################################################# 
    # x_ziel und X_ziel sind die Zielpunkte jeweils im SK und WK
    # vd ist die Wüschgeschwindigkeit 
    # z und grid sind optimale Gitterpunkte und Gitter   
    x_ziel, vd, z, Grid = geneZielpunkt(Xref, ursp_ind, ursp, theta, 30, vmax, obstList, X[t,0:2], eT, eT_1, eT_2)
    X_ziel = fahrzeugZuWelt(x_ziel, ursp, theta)
    Z = fahrzeugZuWelt( np.array([ [m[0] for m in z],  [v[1] for v in z] ] ), ursp, theta)
    
    ################### Relevante Hindernisse aussuchen ####################
    hin, temp = akkuelleHin( mapX, X_ziel,obstList_wk , ursp_ind)#
    
    #Solver zurücksetzen, wenn sich die Anzahl von Hindernissen ändert  
    if len(hin) is not nHin:
       nHin = len(hin)
       w = 0.01*np.ones((8*nHin,))
       solver, varlb, varub, conlb, conub, varguess, var, integrierer=optimierer(Q_mpc, R_mpc, q, \
       T_mpc, N_mpc, ord_mpc,Cdim, nHin, vmax, M=1)             
    hin = hindImFK( hin, ursp, theta)
       
    # Aktuelle Position des Fahrzeugs als Anfangszustände im FK(Initialisierung)
    # SEHR WICHTIG !!!!!!!!!!!!!!!!!
    x_aktuell = np.concatenate( (weltZuFahrzeug(X[t,0:2], ursp, theta), [X[t,2]-theta], X[t,3:5] ) ) 
    varlb[ "x",0,:] = x_aktuell
    varub[ "x",0,:] = x_aktuell
    varguess["x",0,:] = x_aktuell
    if nHin>0: # Diese Variablen existieren, nur wenn es Hindernis gibt
       varguess['w',0,:]=w
    
    # Veränderbare Parameter im Solver  
    if nHin>0:
       args = dict(x0 = varguess, p=vertcat(x_ziel,K,vd, hin), lbx = varlb, ubx = varub, lbg = conlb, ubg = conub)    
    else:
       args = dict(x0 = varguess, p=vertcat(x_ziel,K,vd), lbx = varlb, ubx = varub, lbg = conlb, ubg = conub)
    
    # NLP im FK lösen 
    anfangszeit = time.time()
    sol = solver(**args)
    tSolver = time.time()-anfangszeit
    status = solver.stats()["return_status"]
    optvar = var( sol["x"])      
       
    # Status des Solvers zeigen 
    #print(t,status, 'Optzeit:',tSolver, 'DPzeit:',zeit)
    u = optvar["u",0,:].full().flatten()
    if nHin>0: 
       w = optvar["w",0,:].full().flatten()  
    x = optvar['x']  #Geplante nächste Zustände des Fahrzeugs im FK
    xwk = naechstePosition(x)
    xwk = fahrzeugZuWelt( xwk[0:2,:] ,ursp, theta)
      
    #Integrieren um das Fahrzeug zu bewegen( Integration der Dgl.) 
    intargs = dict( x0 = x_aktuell, p=u)
    out = integrierer(**intargs )
    next_x = np.array(out["xf" ]).flatten()
       
    #Nächste Fahrzeugsposition wieder in WK umwandeln
    X[t+1,:] = np.concatenate( (fahrzeugZuWelt(next_x[0:2] ,ursp, theta), [next_x[2]+theta], next_x[3:5]) )
       
       
    #pathAx[0].set_data(temp[0],temp[1])
    # Fahrzeug in die neue Position bewegen und darstellen
    car.bewegeHin(X[t,0:2],X[t,2])
    Y = car.getXY()
    carAx[0].set_data( Y ) 
    
    #Gitter darstellen
    #if Grid is not []: 
    #   gridAx[0].set_data(Grid[:,:,0], Grid[:, :, 1])
    #zoptAx[0].set_data(Z[0,:], Z[1,:])                  # mögliche Zielpunkte   
    xoptAx[0].set_data( xwk[0,:], xwk[1,:] )           # Geplanter Weg
    #richtungA[0].set_data( [X[t,0],X_ziel[0] ], [X[t,1],X_ziel[1] ])
    
    
    zielAx[0].set_data( X_ziel[0], X_ziel[1] )
    if -3/4*np.pi <= theta <= -np.pi/4:
       xData_max = X[t,0]+20
       xData_min = X[t,0]-20
    if theta <= -3*np.pi/4:
       xData_min = X[t,0]-40
       xData_max = X[t,0]+5
    else:
       xData_max = 40 + X[t,0]
       xData_min = -5 + X[t,0]    
    ax.set_xlim(xData_min, xData_max)
    ax.set_ylim( -3+ X[t,1], 3+ X[t,1])
       
    speedCarAx.set_position((X[t,0]-2, X[t,1]+1))      
    speedCarAx.set_text('v:'+'{:.3}'.format(str(X[t,3]) ))
    obsAx.set_position( (X[t,0]-2, X[t,1]+2) )
    obsAx.set_text('obst:'+'{:.3}'.format(str(nHin) ))
    	
    plt.pause(0.002);
    #assert makeRecord==True, "makeRecord is set to False"   
    save_frame(saveFramesIn)      
plt.show(block=False)
save_movie_as_mp4(saveFramesIn)










