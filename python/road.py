import numpy as np
import scipy.integrate as integrator 
from obstacle import *

      
def abstand(p1,p2):
    """
    Berechnet den euklidischen Abstand zwischen zwei vorgegebenen Punkten
    """ 
    return np.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])) 
    

def naechstePosition(listX):
    """
    Berechnet die nächsten geplanten Positionen des Fahrzeugs im Fk
    listX --> Eine Liste von Casadi-DM-Elementen (Solver)
    """
    result = np.zeros((5,1))
    for i in range(len(listX)):
        result = np.concatenate( (result, listX[i].full()), 1)
    return result[:,1:(i+1)]
                         

def polynomInFahrzeug(mapXY, ordnung, ursprung, theta):
    """
    Straßenstück als Polynom darstellen
    Dies wird in Fahrzeugkoordinate stattfinden 
    mapXY is ein (2,n)-numpy.array 
    """
    temp = weltZuFahrzeug(mapXY, ursprung, theta)     
    return np.polyfit(temp[0,:],temp[1,:], ordnung)

             
             
def rotMatrix(theta):
    """
    Rotationsmatrix von FK bezüglich WK
    """
    return np.array([ [np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)] ])


def krummlinigeKoord(Xref, start_ind):
    """
    Berechnet den Weg (Krummlinige Koordinaten) auf Referenzverlaufs Xref
    beginnend ab index start_ind 
    Xref--> Satz(Menge) von Punkten (x,y) bzw. Referenzverlauf oder Miitelinie der Strasse
    """ 
    n = max( Xref.shape) 
    if n==0:
       return 0
    temp = Xref[:,start_ind:n+1]   
    S1 = [abstand(temp[:,k],temp[:,(k+1)]) for k in range( np.size(temp,1)-1)]
    S2 = [sum(S1[0:k]) for k in range(len(S1)+1)]
    return np.array(S2)
    


def gridImFK(grid, ursprung, theta):
    """
    grid--> Eine Liste von Layern. Jede Layer ist wiederum eine Liste von punkten
    ursprung --> Ursprung von FK
    theta--> Drehwinkel von FK bezüglich WK  
    """
    result = np.zeros(grid.shape)
    for i in range( np.size(grid,0)):
        for j in range(np.size(grid,1)):
            result[i,j] = weltZuFahrzeug( grid[i,j],ursprung , theta)       
    return result 


def gridImFK2(grid, ursprung, theta):
    """
    grid--> Eine Liste von Layern. Jede Layer ist wiederum eine Liste von punkten
    ursprung --> Ursprung von FK
    theta--> Drehwinkel von FK bezüglich WK  
    """
    result = []
    temp = [elt for elt in grid]
    for i in range(len(grid[0])):
            result += [ [weltZuFahrzeug( X[i], ursprung, theta) for X in temp ] ]       
    return np.array(result)
 

def hindImFK(hinList, ursprung, theta):
    result = np.array([])
    for elt in hinList:
        temp1 = elt.getAttribut()
        temp2 = np.append( weltZuFahrzeug(temp1[0], ursprung, theta), temp1[1]-theta)
        temp2 = np.append(temp2, temp1[2:4])
        result = np.append(result,temp2)    
    return result                    


def hindImFK2(hinList, ursprung, theta):
    result1= [] #Hindernisse plus Sicherheitsabstand
    result2= [] #
    for elt in hinList:    
        temp = elt.getAttribut()
        result1 += [RechteckHindernis( weltZuFahrzeug( temp[0], ursprung, theta ), temp[2]+10, temp[3]+0.1, temp[1]-theta, temp[4] ) ]
        result2 += [RechteckHindernis( weltZuFahrzeug( temp[0], ursprung, theta ), temp[2], temp[3], temp[1]-theta, temp[4] ) ]       
    return result1, result2  
        

def weltZuFahrzeug(X, Xf, theta):
    """
    Transformiert Welt- zu Fahezeugsystem
    X --> Koordinaten eines Punkt bezüglich Welt
    Xf --> Koordinaten des Fahrzeug bezüglich Welt
    theta --> Drehwinkel des Fahrzeugs
    """
    if max(X.shape)>2:
       temp = [np.dot(np.linalg.inv(rotMatrix(theta)), X[:,i]-Xf ) for i in range(max(X.shape))]              
       temp1 = [elt[0] for elt in temp]
       temp2 = [elt[1] for elt in temp]
       return np.array( [temp1,temp2] )  
    return np.dot( np.linalg.inv(rotMatrix(theta)), X-Xf )

    
    
def fahrzeugZuWelt(X, Xf, theta):
    """
    Transformation FK-->WK 
    """
    if max(X.shape)>2:
       temp = [np.dot( rotMatrix(theta), X[:,i]) + Xf for i in range(max(X.shape))]
       temp1 = [elt[0] for elt in temp]
       temp2 = [elt[1] for elt in temp] 
       return np.array([temp1,temp2])
    return np.dot(rotMatrix(theta), X) + Xf



def punktZuDist(Xref,d, start_ind):
    """
    Sucht der Punkt des Referenzverlaufs, der im Abstand d liegt
    beginnend ab index start_ind 
    start_ind --> Index vom Startpunkt
    d --> Entfernung 
    """   
    n = max(Xref.shape)
    temp=0
    if start_ind>n-1:
       return None
    delta = abstand(Xref[:,0],Xref[:,1])   
    for k in range(start_ind+1, n):    
        temp+=delta 
        if temp-d>0.01:
           return k-start_ind 
    return None 


def nahPunktIndex(Xref, X, ind_alt):
    """
    Xref--> Menge(Satz) von (x,y)-Punkten(Referenz)
    X--> ein Punkt (x,y) 
    Gibt das Index des Punkts von Xref der am nähesten zu X liegt
    """
    
    n = np.size(Xref,1)
    if ind_alt >= n-1:
       return n-1
    if ind_alt > 5:
       ind_alt -=5
    else:
       ind_alt=0      
    if ind_alt+80>n-1:
       ende = n
    else:
       ende = ind_alt+80                  
    temp= [abstand(X,Xref[:,i]) for i in range(ind_alt,ende )]
    return temp.index(min(temp))+ind_alt



def akuellRouteReferenz(Xref, X, hinList, ursp_alt):
    """
    Gesamte Referenzstraße im WK
    X --> Aktuelle Position des Fahrzeugs im WK
    #smax --> maximaler Abstand(Es wird immer 50 Letter der Straße berücksichtigt )#
    Rückwerte sind:
    1) die zu betrackter Abschnitte der Straße als (2,n)-Shape Matrix
    2) Ursprung des lokalen Koordinatensystems
    3) Index des Ursprungs des lokalen Koordinatensystems im Xref
    """
    nahPunkt_ind = nahPunktIndex(Xref, X, ursp_alt)
    start = nahPunkt_ind-5
    end = nahPunkt_ind+200
    n = max(Xref.shape)
    while n<end:
          start-=1
          end-=1
    if start<0:
       end += start
       start = 0
    if nahPunkt_ind > 1:
       nahPunkt_ind-=1  
    
    if nahPunkt_ind<n-1:
       X1 = Xref[:,nahPunkt_ind]
       X2 = Xref[:,nahPunkt_ind+1]
       theta = np.arctan2(X2[1]-X1[1], X2[0]-X1[0] )
    else:
       X1 = Xref[:,nahPunkt_ind]
       X2 = Xref[:,nahPunkt_ind-1]
       theta = np.arctan2(X1[1]-X2[1], X1[0]-X2[0] )
    if theta == np.pi:
       theta += -2*np.pi   
    hin = []
    for elt in hinList:       
        if start-5<= elt.getIndex() <= end+5:
           hin+=[elt]            
    return Xref[:,start:end], Xref[:,nahPunkt_ind], nahPunkt_ind, theta, hin    


def erzeugHindernis(Xref,ind, Hdim,delta=0, up=0):
    """
    Ein Hindernis auf der Referenzweg Xref erzeugen
    Hdim --> Vektor aus Länge und Breite des Hindernisses
    ind --> Index im Xref der Lage des Hindernisses
    delta--> Entfernung von Xref(Absatz)
    up--> Hinweis darauf ob das hinderniss über oder unter Xref gestellt wird
    """
    if ind> max(Xref.shape):
       Print('Achtung: Der Index ist zu groß')
       return 0  
    X1 = Xref[:,ind]
    X2 = Xref[:,ind+1]
    theta = np.arctan2(X2[1]-X1[1], X2[0]-X1[0] )
    R = rotMatrix(theta)
    if up is 1:
       X = X1+np.dot(R,[0, delta])
    else:
       X = X1+np.dot(R,[0, -delta])  
    return RechteckHindernis( X,Hdim[0], Hdim[1], theta,ind)

def akkuelleHin(mapX, ziel, hinList, ursp_ind):
    """
    Gibt alle Hindernisse aus, die für die aktuelle Planung relevant sind
    mapX--> Aktuelle Referenz 
    Xcar--> Aktuelle Position des Fahrzeugs
    ziel--> Zielpunkt
    hinList--> Eine Liste aller Hindernisse im WK 
    hinListA--> Aktuelle Liste der Hindernisse
    """
    result = []
    temp= [abstand(ziel,mapX[:,i]) for i in range(np.size(mapX,1) )]
    indz = temp.index(min(temp))
  
    for elt in hinList:         
        if ursp_ind-5 <= elt.getIndex() <= ursp_ind+indz+5:
           result.append(elt)
    return result, mapX[:,0:indz+5] 

def geneZielpunkt(Xref, ursp_ind, ursp, ursp_theta, dmax, vmax, hinList, Xcar, eT, eT_1, eT_2):# ursp, ursp_theta
    """
    Adaptive Zielpunktgenerierung 
    Xref-->Markierung der Mittelinie der Straße als Referenzweg
    ursp_ind--> Index(in Xref) des Ursprungs der SK  
    dmax--> Maximaler Abstand der Planung
    vmax--> Maximal erläubte Geschwindigkeit
    hinList--> Liste aller Hindernisse mit Parametern im SK
    Xcar--> Akktuelle Position der Fahrzeugs im SK
    """
    #Reglerparameter
    kp=7
    ki=0
    kd=0
    a =1

    Xcar = weltZuFahrzeug(Xcar,ursp,ursp_theta)
    hinList1, hinList2 = hindImFK2(hinList, ursp, ursp_theta)       
    grid, N, hin = addGrid(Xref, ursp_ind, ursp, ursp_theta, dmax, hinList1)
    Grid = gridImFK2( grid, ursp, ursp_theta)
     
    zielpunkte = dynamikOpt(Grid, hinList1, Xcar)
    delta = kp*eT + ki*eT_1 + kd*eT_2  #PID-Regler
    f = 1/(1+np.exp(-delta/a))
    result=zielpunkte.copy() # (10,2)-ndarray
    
    #Sichtbereich 
    temp = []
    for i in range(max(result.shape)):
        if not beruehtHin(hinList2, result[i], Xcar):
           temp +=[result[i] ]
    temp = np.array(temp)       
    if np.size(temp,0)<1:
       temp = result
            
    n2 = np.size(temp,0)        
    n = round( np.asscalar(f*N) )  #Hier kann interpoliert werden
    if n>n2:
       n=n2
    if n>0:
       return temp[n-1], n*vmax/N, zielpunkte, grid
    return temp[n], vmax/N, zielpunkte, grid
    


def addGrid(Xref, ursp_ind, ursp, ursp_theta, dmax, hinList, N_sample=3, delta_l=0.9, delta_s=3):
    grid = []
    n = max(Xref.shape)
    k = punktZuDist(Xref,delta_s, ursp_ind)
    if k is None: # es kann kein Gitter erzeugt werden
       k = n          
    N_layer = round(dmax/delta_s)
    ind=ursp_ind
    x_init = (N_sample-1)/2*delta_l
    for i in range(N_layer):
        ind +=k 
        if ind<=n-1: 
           X1 = Xref[:,ind]
           if ind<=n-2:
              X2 = Xref[:,ind+1]
              theta = np.arctan2(X2[1]-X1[1], X2[0]-X1[0] ) 
           else:
              X2 = Xref[:,ind-1]         
              theta = np.arctan2(X1[1]-X2[1], X1[0]-X2[0] ) 
           temp1 = [x_init-j*delta_l for j in range(N_sample)]
           temp = [np.dot( rotMatrix(theta), np.array([0,X]) ) + X1 for X in temp1]
           grid +=[temp]
        else:
           break
    #relevante Hindernisse aussuchen
    hin = []
    for elt in hinList:       
        if ursp_ind-5<= elt.getIndex() <= ind+5:
           hin+=[elt]        
    return np.array(grid), N_layer, elt
 
 
def dynamikOpt(grid, hinList, Xcar, w1=0.5, w2=2.5):
    """
    Dynamische Optimierung eines Gitters
    grid --> erzeugter Gritter
    w1 und w2 --> Gewichtungen
    hinList --> List von hindernissen
    Xcar--> Aktuelle Position des Fahrzeugs
    Rückgabe ist ein Vektor aus optimalen Samplen(Stichpunkte)
    """
    if grid is []:
       return []     
    temp = grid.shape
    temp1 = np.zeros((temp[0],temp[1]-1))
    result = np.zeros((temp[1], temp[2]))
    gridR= np.zeros( temp )
    vecR1 = np.zeros((temp[0]))
    vecR2 = np.zeros((temp[0]))
    aux = np.zeros( (temp[0]))
    ind_ref = (temp[0]-1)//2
    if np.remainder(temp[0],2)==0:
       print('Die Anzahl von Samplen: ', temp[0],' muss ungerade sein')
       return 
        
    for i in range( temp[1]-1 ): 
          for j in range( temp[0] ):
              aux[:] = np.inf 
              for t in range(temp[0]):                 
                  if not beruehtHin(hinList, grid[j, -i-2], grid[t,-i-1]):                
                     aux[t] = w1*( abstand(grid[ind_ref, -i-1], grid[t, -i-1]) )**2 + w2*(abstand(grid[j, -i-2],grid[t, -i-1]) )**2         
              temp1[j, -i-1] = np.argmin(vecR1+aux) 
              vecR2[j] += np.min(vecR1+aux) 
          vecR1 = vecR2.copy()
          vecR2[:] = 0
                     
    aux[:] = np.inf 
    for t in range(temp[0]):
        if not liegtInHindernis(grid[t,0], hinList):
           aux[t] = w1*( abstand(grid[ind_ref,0], grid[t,0]) )**2 + w2*(abstand(Xcar, grid[t,0]) )**2 
    temp1 = temp1.astype(int)       
    aux2 = np.argmin(vecR1+aux)
    aux2 = aux2.astype(int)       
    result[0] = grid[aux2, 0]
    for k in range(temp[1]-1):
        aux2 = temp1[aux2,k] 
        result[k+1] = grid[aux2, k+1]                
    return result            # (n,2)-shape 
                    


