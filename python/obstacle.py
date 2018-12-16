import numpy as np
class KreisHindernis:
      """
      Modellierung eines Hindernisses als Kreis
      """
      def __init__(self, center, radius):
            self._center = center
            self._radius = radius
    
      def getRadius(self):
          """
          Radius des Kreises
          """
          return self.radius

      def getCenter(self):
          """
          Mittelpunkt des Kreises
          """
          return self.center
 
      def liegtImKreis(self,X):
          """
          Überprüft ob der Punkt X im Kreis liegt
          """
          return radius**2 >= (X[0]-center[0])**2 + (X[1]-center[1])**2
    
    
      def abstandZuHin(self,X):
          """
          Berechnet den Abstand zu einem Hindernis
          """
          if self.liegtImKreis(X):
             return np.inf
          return np.sqrt((X[0]-center[0])**2 + (X[1]-center[1])**2)-self.radius
        
          
                    

class RechteckHindernis:
      """
      Modellierung eines Hindernisses als Rechteck
      Das Fahrzeug wird auch als Rechteck dargestellt
      """
      def __init__(self, cen, laeng, breit, thet=0, index=None):
          self.center = cen          
          self.laenge = laeng
          self.breite = breit
          self.theta = thet
          self.ind = index
          

      def getBreite(self):
          """
          Breite eines Rechtecks
          """
          return self.breite

      def getLaenge(self):
          """
          Länge eines Rechtecks
          """
          return self.laenge
          
      def getCenter(self):
          """
          Mittetpunkt eines Rechtecks
          """
          return self.center 
          
      def getAngle(self):
          """
          Drehwinkel eines Rechtecks bezüglich WK
          """
          return self.theta 
          
      def getIndex(self):
          """
          Index zur Erzeugung eines Hindernis auf der Straße
          """             
          return self.ind
            
      def rotMatrix(self):
          """
          Rotationsmatrix von FK bezüglich WK
          """
          return np.array([ [np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)] ])

      def invRotMatrix(self):
          """
          Rotationsmatrix von WK bezüglich FK
          """
          return np.array([ [np.cos(self.theta), np.sin(self.theta)], [-np.sin(self.theta), np.cos(self.theta)] ])

          
      def getAttribut(self):
          """
          Alle Parameter eines Rechtecks zurückgeben 
          """
          return self.center, self.theta, self.laenge, self.breite, self.ind  
          
      def setCenter(self, newCenter):
          """
          Mittelpunkt eines Rechtecks vorgeben
          """
          self.center = newCenter
          
      def setAngle(self, newAngle):
          """
          Drehwinkel eines Rechtecks vorgeben
          """
          self.theta = newAngle 
          
      def setLaenge(self, l):
          self.laenge = l


      def setBreite(self, b):
          self.laenge = b
                     
          
      def bewegeHin(self, newCenter, newTheta):
          """
          Ein Hindernis bewegen, indem neue Mittelpunkt und Drehwinkel vorgegeben werden
          """
          self.setCenter(newCenter)
          self.setAngle(newTheta) 
      
          
      def getXY(self):
          """
          Ein Rechteck zum Darstellen vorbereiten
          """
          p1, p2, p3, p4 = self.eckpunkte() 
          X = [p1[0], p2[0], p3[0], p4[0], p1[0]]
          Y = [p1[1], p2[1], p3[1], p4[1], p1[1]]
          return np.array(X), np.array(Y)          


      def liegtImRechteck(self, X):
          """
          Überprüft ob ein vorgegebener Punkt in einem Hindernis liegt
          
          E1             E2
            |------------|
            |            |
            |    X       | 
          E4|------------| E3
                    
          """
          E1, E2, E3, E4 = self.eckpunkte()  
          K1, K2, K3, K4 = self.seitenGerade()
          return (distZuGerade(X,K1)+distZuGerade(X,K3)==distanz(E1,E4) ) and \
                 ( distZuGerade(X, K2)+distZuGerade(X, K4) == distanz(E3, E4) )


      def liegtImRechteck2(self, X):
          """
          Überprüft ob ein vorgegebener Punkt in einem Hindernis liegt
          zweite Variante
          
          E1             E2
            |------------|
            |            |
            |    X       | 
          E4|------------| E3
                    
          """
          temp = self.getAttribut()
          A0= np.array([[1, 0],[-1, 0],[0, 1],[0, -1]])
          #inv_Rot = self.invRotMatrix()
          inv_Rot = np.array([ [np.cos(temp[1]), np.sin(temp[1])], [-np.sin(temp[1]), np.cos(temp[1])] ])
          A = np.dot(A0,inv_Rot)
          b0 = 0.5*np.array([temp[2],temp[2], temp[3],temp[3]])
          b = b0+np.dot(A,temp[0])
          B = np.dot(A,X)-b
          return B[0]<=0 and B[1]<=0 and B[2]<=0 and B[3]<=0


      def eckpunkte(self):
          """
          Die Koordinaten aller vier Eckpunkte eines Rechtecks berechnen
          Rückgabe im WK
          
          E1              E2
            |------------|
            |            |
            |            | 
          E4|------------| E3
                           
          """
          temp = self.rotMatrix()
          l = self.laenge/2
          b = self.breite/2
          c = self.center
          E1 = np.dot(temp, np.array([-l,b]) ) + c
          E2 = np.dot(temp, np.array([l, b]) ) + c
          E3 = np.dot(temp, np.array([l,-b]) ) + c
          E4 = np.dot(temp, np.array([-l,-b])) + c
          return E1, E2, E3, E4 
      
      
      def seitenGerade(self):
          """
          Die vier Seiten eines Rechteckts als Gerade darstellen
          
                 K1
            |------------|
            |            |
         K4 |            | K2
            |------------| 
                  K3  
          """
          E = self.eckpunkte()
          K1 = geradenGleichung(E[0], E[1])
          K2 = geradenGleichung(E[1], E[2])
          K3 = geradenGleichung(E[2], E[3])
          K4 = geradenGleichung(E[3], E[0])
          return K1, K2, K3, K4
 

      def abstandZuHin(self,X):
          """
          Berechnet den Abstant zu einem Hindernis(Rechteckt)
          vorausgesetzt, dass der Punkt nicht im Hindernis liegt
          E1     K1       E2 
           |--------------|
           |              | 
        K4 |              | K2 
           |              | 
           |--------------|
          E4      K3       E3
                 
          """
          E1, E2, E3, E4 = self.eckpunkte()  
          K1, K2, K3, K4 = self.seitenGerade() 
          temp1, temp2, temp3, temp4 = distZuGerade(X,K1), distZuGerade(X,K3), distZuGerade(X,K2), distZuGerade(X,K4) 
          if temp1 + temp2 == distanz(E1,E4): #Der Punkt liegt zwwischen Gerade K1 und K3    
             return min( temp3, temp4)
          elif temp3 + temp4 == distanz(E1,E2): #Der Punkt liegt zwwischen Gerade K2 und K4    
             return min(temp1, temp2)
          else:
             return min( distanz(X, E4), distanz(X, E3), distanz(X, E1), distanz(X, E2))  


      def kontaktPunkt(self, p1,p2):
          """
          Überprüft ob der Segment p1p2 und das Hindernis sich überschneiden
          vorausgesetzt, dass die Punkte nicht im Hindernis liegen
                  p1 
          E1       |       E2
            |------|-----|
            |      |     |
            |      |     | 
          E4|------|-----| E3
                   |p2 
          """
          E1, E2, E3, E4 = self.eckpunkte()    
          aux1, K1 = schnittSegment(p1,p2,E1,E2)
          aux2, K2 = schnittSegment(p1,p2,E2,E3)
          aux3, K3 = schnittSegment(p1,p2,E3,E4)
          aux4, K4 = schnittSegment(p1,p2,E4,E1)
          return aux1 or aux2 or aux3 or aux4 or self.liegtImRechteck2(p2) 
 
 
      def istImKontakt(self, R):
          """
          TO DO
          Überprüft ob zwei Rechtecke im kontakt sind
          Müss noch gefixt werden
          """
          E1, E2, E3, E4 = R.eckpunkte()
          return (self.liegtImRechteck(E1) or self.liegtImRechteck(E2) or self.liegtImRechteck(E3) or self.liegtImRechteck(E4) )
     
 
def liegtInHindernis(X, hinList):
    """
    Überprüft ob der Punkt X in einem Hinderniss aus der Liste liegt
    hinList --> Liste von Hindernissen
    """ 
    for elt in hinList:
        if elt.liegtImRechteck2(X):
           return True
    return False
                      

def geradenGleichung(X1, X2):
    """
    Gibt die Koeffizienten einer Gerade,
    die durch X1 und X2 verlauft (y = a*x+b) als [a, b] zurück
    Für Gerade der Form x=a wird [inf, a] zurückgegeben
    """
    if X1[0]==X2[0]:
       return [np.inf, X1[0]]
    return [(X1[1]-X2[1])/(X1[0]-X2[0]), -(X1[1]-X2[1])/(X1[0]-X2[0])*X1[0]+X1[1]]         
     
     
def distZuGerade(X, K):
    """
    Berechnet den Abstand von dem Punkt X zu der Gerade
    mit Kooeffizienten in K (y = a*x+b und K= [a, b])
    """     
    if K[0] is np.inf: #Gerade der Form x = a
       return np.abs( X[0]-K[1]) 
    return np.abs(K[0]*X[0]-X[1]+K[1])/np.sqrt(K[0]**2+1)  
         
          
 
def distanz(X1, X2):
    """
    Euklidischer Abstand zwischen zwei Punkte
    """
    return  np.sqrt((X1[0]-X2[0])**2 + (X1[1]-X2[1])**2)   
      
      
           
def abstandZuHinList(hinList, X):
    """
    Berechnet den kleinsten Abstand zu allen Hindernissen in der Liste hinList
    Vorausgesetzt, dass der Punkt X nicht in einem Hindernis liegt 
    """
    result = np.inf
    for elt in hinList:
        temp = elt.abstandZuHin(X)
        if temp<result:
           result=temp
    return result       
      
             

def beruehtHin(hindList, p1, p2):
    """
    Überprüft ob der Segment p1p2 mindestens ein Hindernis aus der Liste überschneidet
    Vorausgesetzt, dass p1 und p2 in keinem Hindernis liegen
    hindList--> eine Liste von Hindernissen
    """
    for elt in hindList:
        if elt.kontaktPunkt(p1,p2):
           return True
    return False       
            

      
def schnittSegment(A,B,C,D):
    """
    Überprüft ob die Segmente AB und CD gemeinsamer Schnittpunkt haben
    """
    K1 = geradenGleichung(A, B)
    K2 = geradenGleichung(C, D)
    if K1[0] is np.inf: #Gerade der Form x = a
       aux1 = (C[0]-K1[1])*(D[0]-K1[1]) 
    else:
       aux1 = (C[1]-K1[0]*C[0]-K1[1])*(D[1]-K1[0]*D[0]-K1[1])    
    if K2[0] is np.inf:
       aux2 = (A[0]-K2[1])*(B[0]-K2[1])
    else:
       aux2 = (A[1]-K2[0]*A[0]-K2[1])*(B[1]-K2[0]*B[0]-K2[1]) 
    return (aux1<= 0) and (aux2<=0), K2  
          
