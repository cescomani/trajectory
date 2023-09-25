from casadi import *
from casadi.tools import *
import numpy as np


def optimierer(Q, R, q, T, N, ordnung, Cdim, nHin, vmax, M=4):
    """ 
    Hdim --> Vector aus Länge und Breite eines Hindernisses
    nHin --> Anzahl Von möglichen Hindernissen
    """
    
    # Stell-, Zustandgröße und andere Parameter als symbolische Variable
    Nx=5
    Nu=2 
    Nw=8 
    r = 0.15
    dim = SX.sym('d',2)                 #Größe eines Hindernisses 
    x = SX.sym('x',Nx)                  #(5,1)-Matrix
    u1 = SX.sym('u',Nu)                 #(2,1)-Matrix
    u2 = SX.sym('u2',Nu)
    X_z = SX.sym('x_z',2)               #Zwischenzielpunkt als (x,y)
    K = SX.sym('k',ordnung+1)           #Koefficienten eines Polynoms der Ordnung 'ordnung'
    vd = SX.sym('k')                    #Soll-Gescwindigkeit 
    o = SX.sym('o', 5)                  #Darstellung eines hindernisses oder Fahrzeugs
    h = SX.sym('h',5*nHin)              #Position, Drehwinkel, Länge und Breite [x,y,l,b,theta] für alle Aktuelle Hindernisse

    # Diefferentialgleichung des Systems
    xdot = vertcat( x[3]*cos(x[2]), x[3]*sin(x[2]), x[4], u1[0], u1[1])  
    f = Function('f', [x, u1], [xdot] )

    #Mittellinie der Straße als Polynom approximieren und Y-Abweischung berechnen
    ey = 0 # y = f(x) = a_nx^n+...+a_1x+ a_0 mit K=[a_n,..., a_0 ]
    e_theta = 0 # tan(theta) = f'(x)
    for i in range(ordnung+1):
        ey += K[i]*x[0]**(ordnung-i)
    for j in range(ordnung):
        e_theta+=(ordnung-j)*K[j]*x[0]**(ordnung-j-1)      
    e_theta = np.arctan(e_theta)
    # Gütefunktion Aus Zuständen
    L= mtimes(mtimes((x[0:2]-X_z).T,Q[0:2,0:2] ),x[0:2]-X_z) + Q[2,2]*(x[1]-ey)**2 + \
       Q[3,3]*(x[2]-e_theta)**2 + Q[4,4]*(x[3]-vd)**2 + Q[5,5]*x[4]**2
    fL = Function('fl',[x, X_z], [L])
    LN = q*(x[2]-atan2(X_z[1], X_z[0]))**2
    fN = Function('fn',[x, X_z],[LN])    #Endbedingung


    # Gütefunktion aus Stellgrößen
    Lu = mtimes( mtimes(u1.T,R), u1 ) 
    fu = Function('fu', [u1], [Lu] )
    
    # Sprungänderung der Stellgröße berücksichtigen
    Lua = mtimes(mtimes((u1-u2).T,R),(u1-u2))
    fua = Function('fua',[u1,u2], [Lua])
    
    # Bedingung zur Hindernisvermeidung
    invRot = vertcat( horzcat(cos(o[2]), sin(o[2]) ), horzcat(-sin(o[2]), cos(o[2]) ) ) 
    A0 = vertcat( horzcat(1,0), horzcat(-1,0), horzcat(0,1), horzcat(0,-1) )
    fA = Function('fa',[o],[ mtimes(A0,invRot)])
    
    b = mtimes(fA(o), o[0:2] ) + 1/2*vertcat( o[3]+2, o[3]+2, o[4]+0.3, o[4]+0.3)
    fb = Function('fb',[o], [b] )
    
    
    # Integrator-Objekt erzeugen 
    # CVODES from the SUNDIALS suite
    dae=dict(x=x, p=u1, ode=xdot)
    intopts = {"abstol":1e-8, "reltol":1e-8, "tf":T/N}
    integrierer = casadi.integrator('itg', 'cvodes', dae, intopts)

    # M -RK4- Schritte per interval
    DT = T/N/M
    X = x
    for j in range(M):
        k1 = f(X, u1)
        k2 = f(X + DT/2*k1, u1)
        k3 = f(X + DT/2*k2, u1)
        k4 = f(X + DT*k3, u1)
        X = X + DT/6*(k1 +2*k2 +2*k3 +k4)
    F = Function('F', [x, u1], [X] )
          
    # N*Nu + (N+1)*Nx Varaiablen werden für die Durchführung der Optimierung erzeugt    
    if nHin>0: 
       var=struct_symSX( [( entry("x",shape=(Nx,), repeat=N+1), entry("w",shape=(Nw*nHin,), repeat=N+1), entry("u",shape=(Nu,), repeat=N), )] )
    else:
       var = struct_symSX( [( entry("x",shape=(Nx,), repeat=N+1), entry("u",shape=(Nu,), repeat=N ), )] )
       
    varlb = var(-np.inf)   #Obere Grenze 
    varub = var(np.inf)    #Untere Grenze 
    varguess = var(0)      #Anfangslösung
    
    # Stellgrößen- und Zustandbeschränckung 
    for t in range( N ):
        varlb["u" ,t ,:] = -2
        varub["u" ,t ,:] = 2
        varub["x" ,t ,3] = vmax
        varlb["x" ,t ,3] = 0
        varlb['x' ,t ,1] =-2
        varub['x' ,t ,1] =2
                
    varub["x" ,-1 ,3] = vmax
    varlb["x" ,-1 ,3] = 0   
    
    # Gütemaße und Nebenbedingungen(bzw. Integration der DG)
    obj = SX(0)
    con = []
    conlb = []
    conub = []

    for t in range( N ):
        con.append( F( var["x" ,t], var["u" ,t ] )-var["x" ,t +1] ) # Dynamik des Modells
        conlb+=[0,0,0,0,0]
        conub+=[0,0,0,0,0]
        obj += fL( var["x",t], X_z) + fu(var["u",t]) # 
        if t>=1:
           obj+=fua(var["u",t-1], var["u",t])
    obj += fL( var['x',-1], X_z) + fN( var['x',-1], X_z)
       
    # Bedingung zur Hindernisvermeidung       
    if nHin>0:  
        varguess['w']=0.1  
        for t in range(N+1):  
            for j in range( nHin):
                elt = h[5*j:5*(j+1)]
                varlb["w", t, 8*j:8*(j+1)] = 0.01    # alle variable w müsen größer null sein
                gI = vertcat( fb( vertcat(var["x",t,0:3], Cdim) ), fb(elt) )
                GI = vertcat(fA( var["x",t, 0:5] ), fA(elt) ) # oder var['x',t,0:3]
                con.append( mtimes(GI.T,var["w",t,8*j:8*(j+1)]) )
                conlb+=[0,0]
                conub+=[0,0] 
                con.append( mtimes(gI.T, var["w",t, 8*j:8*(j+1)]) )
                conlb+=[-np.inf]
                conub+=[-0.01]

    # Solver bauen, Zwischenzielpunkt, K, Wünschgeschwindigkeit und atuelle Hindernisse werden als Parameter vorgegeben
    # K enthält die Koeffizienten des Polynoms R (Referenzweg) 
    if nHin>0:
       nlp = dict(x=var, p=vertcat(X_z, K, vd, h), f= obj, g=vertcat( *con) )
    else:   
       nlp = dict(x=var, p=vertcat(X_z, K, vd), f= obj, g=vertcat( *con) )
    nlpoptions = {"ipopt":{"print_level":0 ,"max_cpu_time":60, 'max_iter':200}, "print_time":False}

    solver = casadi.nlpsol("solver",'ipopt' , nlp , nlpoptions ) 
    
    return solver, varlb, varub, conlb, conub, varguess, var, integrierer 
