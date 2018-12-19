#include "Gerade.h"
#include <QtMath>

/**
 * @brief Gerade::Gerade
 */
Gerade::Gerade():a_0(0), a_1(0){}

/**
 * @brief Gerade::Gerade
 * @param pt1
 * @param pt2
 */
Gerade::Gerade(QPointF const& pt1, QPointF const& pt2){
  if(pt1.x()==pt2.x()){
      a_0=pt1.x();
      a_1=qInf();
  }else{
      a_1=(pt2.y()-pt1.y())/(pt2.x()-pt1.x());
      a_0=(pt1.y()*pt2.x()-pt2.x()*pt1.y())/(pt2.x()-pt1.x());
  }
}

/**
 * @brief Gerade::Gerade
 * @param a1
 * @param a0
 */
Gerade::Gerade(double a1, double a0):a_1(a1), a_0(a0) {}

/**
 * @brief Gerade::Gerade
 * @param ge
 */
Gerade::Gerade(Gerade const& ge): a_0(ge.a_0), a_1(ge.a_1){}

/**
 * @brief Gerade::distanzZuG
 * @param pt
 * @return
 */
double Gerade::distanzZuG(QPointF const& pt ) const{
  if( qIsInf(a_0) ) return qAbs(pt.x()-a_0);
  return qAbs(a_1*pt.x()-pt.y()+a_0)/qSqrt(qPow(a_1,2)+1);
}

/**
 * @brief Gerade::getA1
 * @return
 */
double Gerade::getA1() const{
    return a_1;
}

/**
 * @brief Gerade::getA0
 * @return
 */
double Gerade::getA0() const{
    return a_0;
}

/**
 * @brief distanzP1P2
 * @param pt1
 * @param pt2
 * @return
 */
double distanzP1P2(QPointF const& pt1, QPointF const& pt2){
   return qSqrt( qPow(pt1.x()-pt2.x(),2) + qPow(pt1.y()-pt2.y(),2) ) ;
}


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
