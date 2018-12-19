#include<casadi/casadi.hpp>                                                    
#include<iostream>                                                              
#include <QGenericMatrix>
#include <QPointF>
#include "Geometrie.h"
#include <QtMath>
#include <QDebug>
#include <string>

using namespace std;                                                            
using namespace casadi;                                                        
                                                                               
int main(int argc, char *argv[]){
        cout << "Test Cadadi:" << endl;
        MX x = MX::sym("x", 1);                                                
        MX y = MX::sym("y", 1);                                                
        MX z = MX::sym("z", 1);                                                
        MX f = pow(x,2) + 100 * pow(z,2);                                      
        MX g = z + pow((1-x), 2) - y;                                          
        MXDict nlp = {{"f", f}, {"g", g}, {"x", MX::vertcat({x,y,z})}};        
        Function solver = nlpsol("S","ipopt",nlp);                              
        DMDict arg ={{"ubg", 0}, {"lbg", 0}, {"x0", DM::vertcat({0,0,0})}};    
        DMDict res = solver(arg);

        cout<< "Test QGenericMatrix:" <<endl;
        double tab[]={1,2,2,4};
        QGenericMatrix<2,2,double> ma(tab);
        cout<<"m11="<<ma.data()[0]<<" m12="<<ma.data()[1]<<" m21="<<ma.data()[2]<<" m22="<<ma.data()[3]<<endl;
        cout<< "Test code:" <<endl;
        QVector<QPointF> vecTest{};
        vecTest.push_back(QPointF());
        for(int i(0); i<=5; ++i){
           vecTest.push_back(QPointF(i*1.5, i*4.0-3));
        }
        Repere rep(QPoint(1.0,1.0), M_PI);
        QVector<QPointF> vecTestR( vecTransfo(vecTest,rep) );
        for(QVector<QPointF>::iterator it(vecTest.begin()); it!=vecTest.end(); ++it)
            cout << "(" <<it->x()<<","<<it->y()<<");" ;
        cout<< endl;
        for(QVector<QPointF>::iterator it(vecTestR.begin()); it!=vecTestR.end(); ++it)
            cout << "(" <<it->x()<<","<<it->y()<<");";
        cout<< endl;
        QPointF* tab1=vecTest.data();
        cout <<"("<<(++tab1)->x()<<","<<(++tab1)->y()<<");"<<"("<<tab1->x()<<","<<tab1->y()<<");" ;
        cout<<endl;
        struct foo{
            double x,y;
        };
        foo a{1,2}, b(a);
        foo c=a;
        string text;
        text="tous dehors";
        cout << text <<endl;
        //int vale(4), &refVal(vale), *ptrval(&refVal);
        //cout<< ptrval <<endl;

        qDebug() << "bien recu";
        cout << "x="<<c.x <<" und y="<<c.y << "\n";
        return 0;                                                              
}
