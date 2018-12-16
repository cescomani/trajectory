import numpy as np
import matplotlib.pyplot as plt


def courbe3D(points, nb):
   p0, p1, p2, p3 = points[0],  points[1],  points[2],  points[3]
   t = np.linspace(0,1,nb) 
   aux1 = [p0[0]*(1-i)**3+3*i*p1[0]*(1-i)**2 + 3*p2[0]*(1-i)*i**2+p3[0]*i**3 for i in t ]
   aux2 = [p0[1]*(1-i)**3+3*i*p1[1]*(1-i)**2 + 3*p2[1]*(1-i)*i**2+p3[1]*i**3 for i in t ]

   return  np.array(aux1), np.array(aux2)


def courbe2D(points, nb):
   t = np.linspace(0,1,nb) 
   temp1 = np.zeros(nb)
   temp2 = np.zeros(nb)
   for i in range(nb):
       q0 = points[0]+t[i]*(points[1]-points[0])
       q1 = points[1]+t[i]*(points[2]-points[1])
       aux = q0+t[i]*(q1-q0)
       temp1[i] = aux[0]
       temp2[i] = aux[1]
   return  temp1, temp2    


def courbe1D(points, nb):
   t = np.linspace(0,1,nb) 
   temp1 = np.zeros(nb)
   temp2 = np.zeros(nb)
   for i in range(nb):
       aux = points[0]+t[i]*(points[1]-points[0])
       temp1[i] = aux[0]
       temp2[i] = aux[1]
   return  temp1, temp2  

def testStrecke1():
    P0, P01, P02 = np.array([0,60]), np.array([0,62]), np.array([0,58])
    P1, P1_1, P1_2 = np.array([25,66]), np.array([27,64]), np.array([23,68])
    P2, P21, P22 = np.array([60,80]), np.array([62,78]), np.array([58,82])
    P3, P31, P32 = np.array([100,80]), np.array([100,82]), np.array([100,78])
    P311, P312, P313 = np.array([150,80]), np.array([152,78]), np.array([148,82])
    P4, P41, P42 = np.array([150,40]), np.array([152,40]), np.array([148,40])
    P5, P51, P52 = np.array([150,35]), np.array([152,35]), np.array([148,35])
    P6, P61, P62 = np.array([150,0]), np.array([152,2]), np.array([148,-2])
    P7, P71, P72 = np.array([220,0]), np.array([220,2]), np.array([220,-2])
    P8, P81, P82 = np.array([360,0]), np.array([360,2]), np.array([360,-2])
    P9, P91, P92 = np.array([400,0]), np.array([398,2]), np.array([402,-2])
    P10, P101, P102 = np.array([400,45]), np.array([398,45]), np.array([402,45])
    P11, P111, P112 = np.array([400,150]), np.array([398,150]), np.array([402,150])
    P12, P121, P122 = np.array([400,200]), np.array([398,198]), np.array([402,202])
    P13, P131, P132 = np.array([250,200]), np.array([250,198]), np.array([250,202])
    P14, P141, P142 = np.array([100,200]), np.array([100,198]), np.array([100,202])
    P15, P151, P152 = np.array([60,200]), np.array([62,198]), np.array([58,202])
    P16, P161, P162 = np.array([60,160]), np.array([62,160]), np.array([58,160])
    P17, P171, P172 = np.array([60,130]), np.array([62,130]), np.array([58,130])
    P18 = np.array([70,82])
    P19, P191, P192 = np.array([80,170]), np.array([78,172]), np.array([82,168]) 
    P20, P201, P202 = np.array([150,170]), np.array([150,172]), np.array([150,168])
    P24, P241, P242 = np.array([300,170]), np.array([300,172]), np.array([300,168])
    P25, P251, P252 = np.array([400,170]), np.array([402,172]), np.array([398,168])
    P26, P261, P262 = np.array([400,80]), np.array([402,78]), np.array([398,82])
    P27, P271, P272 = np.array([300,80]), np.array([300,78]), np.array([300,82])
    P28, P281, P282 = np.array([220,80]), np.array([220,78]), np.array([220,82])
    P29, P291, P292 = np.array([100,60]), np.array([98,62]), np.array([102,58])

    C1, C1_1, C1_2 = courbe3D([P0,P29, P19, P20], 800), courbe3D([P01,P291, P191, P201], 800), courbe3D([P02,P292,P192, P202], 800)
    C2, C21, C22 = courbe1D([P20, P24], 550), courbe1D([P201,P241], 550), courbe1D([P202, P242], 550)
    C3, C31, C32 = courbe3D([P24,P25, P26, P27], 950), courbe3D([P241,P251, P261, P271], 950), courbe3D([P242,P252, P262, P272], 950)
    C4, C41, C42 = courbe3D([P28,P311,P6, P7], 840), courbe3D([P281,P312, P61, P71], 840), courbe3D([P282,P313, P62, P72], 840)
    C5, C51, C52 = courbe1D([P7,P8], 520), courbe1D([P71,P81], 520), courbe1D([P72,P82], 520)
    C6, C61, C62 = courbe2D([P8,P9,P10], 400), courbe2D([P81,P91,P101], 400), courbe2D([P82,P92,P102], 400)
    C7, C71, C72 = courbe1D([P10,P11], 400), courbe1D([P101,P111], 400), courbe1D([P102,P112], 400)
    C8, C81, C82 = courbe2D([P11,P12,P13], 400), courbe2D([P111,P121,P131], 400), courbe2D([P112,P122,P132], 400)
    C9, C91, C92 = courbe1D([P13,P14], 600), courbe1D([P131,P141], 600), courbe1D([P132,P142], 600)
    C10, C101, C102 = courbe2D([P14,P15,P16], 400), courbe2D([P141,P151,P161], 400), courbe2D([P142,P152,P162], 400)
    C11, C111, C112 = courbe1D([P16,P17], 300), courbe1D([P161,P171], 300), courbe1D([P162,P172], 300)
    C12, C121, C122 = courbe3D([P17,P2,P1, P0], 500), courbe3D([P171,P21,P1_1, P02], 500), courbe3D([P172,P22,P1_2, P01], 500)
    C13, C131, C132 = courbe1D([P27,P28], 400), courbe1D([P271,P281], 400),  courbe1D([P272,P282], 400)
    X = np.concatenate((C1[0], C2[0], C3[0], C13[0], C4[0], C5[0], C6[0], C7[0], C8[0], C9[0], C10[0], C11[0], C12[0]))
    Y = np.concatenate((C1[1], C2[1], C3[1], C13[1], C4[1], C5[1], C6[1], C7[1], C8[1], C9[1], C10[1], C11[1], C12[1]))

    X1 = np.concatenate((C1_1[0], C21[0], C31[0], C131[0], C41[0], C51[0], C61[0], C71[0], C81[0], C91[0], C101[0], C111[0], C121[0]))
    Y1 = np.concatenate((C1_1[1], C21[1], C31[1], C131[1], C41[1], C51[1], C61[1], C71[1], C81[1], C91[1], C101[1], C111[1], C121[1]))

    X2 = np.concatenate((C1_2[0], C22[0], C32[0], C132[0], C42[0], C52[0], C62[0], C72[0], C82[0], C92[0], C102[0], C112[0], C122[0]))
    Y2 = np.concatenate((C1_2[1], C22[1], C32[1], C132[1], C42[1], C52[1], C62[1], C72[1], C82[1], C92[1], C102[1], C112[1], C122[1]))
    return (X[0:4060], Y[0:4060]), (X1[0:4060], Y1[0:4060]), (X2[0:4060],Y2[0:4060])



if __name__ == '__main__':
   (X, Y), (X1, Y1), (X2,Y2) = testStrecke1()
   #print(X.shape)
   plt.plot(X, Y, '.-')
   #plt.plot(X[1500:-1], Y[1500:-1],'>')
   #plt.axis("equal")
   plt.show()





