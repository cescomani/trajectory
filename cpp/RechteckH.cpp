#include "RechteckH.h"
#include "Gerade.h"
#include "Geometrie.h"
//#include <QtMath>

/**
 * @brief RechteckH::RechteckH
 * @param center
 * @param laenge
 * @param breite
 * @param theta
 */
RechteckH::RechteckH(QPointF ce, double laenge, double breite, double theta): m_center(ce.x(),ce.y()), m_laenge(laenge), m_breite(breite), m_theta(theta){}
RechteckH::RechteckH():m_center(0,0), m_laenge(1.0), m_breite(1.0), m_theta(0.0){}
RechteckH::RechteckH(RechteckH const& re):m_center(re.m_center), m_laenge(re.m_laenge), m_breite(re.m_laenge), m_theta(re.m_theta){}

/**
 * @brief getCenter
 * @return
 */
QPointF RechteckH::getCenter() const{
    return m_center;
}

/**
 * @brief RechteckH::getLaenge
 * @return
 */
double RechteckH::getLaenge() const{
   return m_laenge;
}

/**
 * @brief RechteckH::getBreite
 * @return
 */
double RechteckH::getBreite() const{
   return m_breite;
}

/**
 * @brief RechteckH::getTheta
 * @return
 */
double RechteckH::getTheta() const{
   return m_theta;
}

/**
 * @brief setCenter
 * @param newCe
 */
void RechteckH::setCenter(QPointF const& newCe){
   m_center=newCe;
}

/**
 * @brief RechteckH::setTheta
 * @param newTheta
 */
void RechteckH::setTheta(double const& newTheta){
   m_theta=newTheta;
}

/**
 * @brief RechteckH::imRechtEck
 * @param pt
 * @return
 */
bool RechteckH::imRechtEck(QPointF const& pt) const{
   QVector<QPointF> eckP=eckPunkte();
   QVector<Gerade> G=seitenGerade2(eckP);
   bool temp1( G.at(0).distanzZuG(pt)+G.at(2).distanzZuG(pt)==distanzP1P2(eckP.at(0), eckP.at(3)) );
   bool temp2( G.at(1).distanzZuG(pt)+G.at(3).distanzZuG(pt)==distanzP1P2(eckP.at(0), eckP.at(1)) );
   return temp1 && temp2;
}

/**
 * @brief RechteckH::eckPunkte
 *
 * @return
 */
QVector<QPointF> RechteckH::eckPunkte() const{
  QVector<QPointF> result;
  QGenericMatrix<2,2,double> R(rotMatrix1(m_theta));
  result.push_back( Ma2Punkt( R*( punkt2Ma(QPointF(-0.5*m_laenge, 0.5*m_breite)))) + getCenter() );
  result.push_back( Ma2Punkt( R*( punkt2Ma(QPointF(0.5*m_laenge, 0.5*m_breite) ))) + getCenter() );
  result.push_back( Ma2Punkt( R*( punkt2Ma(QPointF(0.5*m_laenge, -0.5*m_breite) ))) + getCenter() );
  result.push_back( Ma2Punkt( R*( punkt2Ma(QPointF(-0.5*m_laenge, -0.5*m_breite) ))) + getCenter() );
  return result;
}

/**
 * @brief RechteckH::seitenGerade
 * @return
 */
QVector<QPointF> RechteckH::eckPunkte1(QGenericMatrix<2,2,double> const& R) const{
    QVector<QPointF> result;
    result.push_back(Ma2Punkt( R*( punkt2Ma(QPointF(-0.5*m_laenge, 0.5*m_breite) )))+getCenter() );
    result.push_back(Ma2Punkt( R*( punkt2Ma(QPointF(0.5*m_laenge, 0.5*m_breite) )))+getCenter() );
    result.push_back(Ma2Punkt( R*( punkt2Ma(QPointF(0.5*m_laenge, -0.5*m_breite) )))+getCenter() );
    result.push_back(Ma2Punkt( R*( punkt2Ma(QPointF(-0.5*m_laenge, -0.5*m_breite) )))+getCenter() );
    return result;
}

/**
 * @brief RechteckH::seitenGerade
 * @return
 */
QVector<Gerade> RechteckH::seitenGerade() const{
  QVector<QPointF> eckP=eckPunkte();
  QVector<Gerade> result;
  result.push_back( Gerade(eckP.at(0), eckP.at(1)) );
  result.push_back( Gerade(eckP.at(1), eckP.at(2)) );
  result.push_back( Gerade(eckP.at(2), eckP.at(3)) );
  result.push_back( Gerade(eckP.at(3), eckP.at(0)) );
  return result;
}

/**
 * @brief seitenGerade1
 * @param R
 * @return
 */
QVector<Gerade> RechteckH::seitenGerade1(QGenericMatrix<2,2,double> const& R) const{
    QVector<QPointF> eckP=eckPunkte1(R);
    QVector<Gerade> result;
    result.push_back( Gerade(eckP.at(0), eckP.at(1)) );
    result.push_back( Gerade(eckP.at(1), eckP.at(2)) );
    result.push_back( Gerade(eckP.at(2), eckP.at(3)) );
    result.push_back( Gerade(eckP.at(3), eckP.at(0)) );
    return result;
}

/**
 * @brief seitenGerade2
 * @param eckP
 * @return
 */
QVector<Gerade> RechteckH::seitenGerade2(QVector<QPointF> const& eckP) const{
    QVector<Gerade> result;
    result.push_back( Gerade(eckP.at(0), eckP.at(1)) );
    result.push_back( Gerade(eckP.at(1), eckP.at(2)) );
    result.push_back( Gerade(eckP.at(2), eckP.at(3)) );
    result.push_back( Gerade(eckP.at(3), eckP.at(0)) );
    return result;
}

/**
 * @brief RechteckH::distanzZuH
 * @param pt
 * @return
 */
double RechteckH::distanzZuH(QPointF const& pt) const{
   QVector<QPointF> eckP=eckPunkte();
   QVector<Gerade> G=seitenGerade2(eckP);
   if( G.at(0).distanzZuG(pt)+G.at(2).distanzZuG(pt)==distanzP1P2( eckP.at(0), eckP.at(3)) ){
     return std::min( G.at(1).distanzZuG(pt), G.at(3).distanzZuG(pt) );
   }else if( G.at(1).distanzZuG(pt)+G.at(3).distanzZuG(pt)==distanzP1P2(eckP.at(0), eckP.at(1) ) ){
     return std::min( G.at(0).distanzZuG(pt), G.at(2).distanzZuG(pt) );
   }
   double tab[]={distanzP1P2(eckP.at(0), pt), distanzP1P2(eckP.at(1), pt), distanzP1P2(eckP.at(2), pt), distanzP1P2(eckP.at(3), pt) };
    return *std::min_element(tab, tab+4 );
}

/**
 * @brief RechteckH::imKontakt
 * @param ge
 * @return
 */
bool RechteckH::imKontakt(QPointF const& pt1, QPointF const& pt2) const{
    QVector<QPointF> eckP=eckPunkte();
    bool temp1(schnittPunkt(pt1, pt2, eckP.at(0), eckP.at(1) )), temp2(schnittPunkt(pt1, pt2, eckP.at(1), eckP.at(2) ));
    bool temp3(schnittPunkt(pt1, pt2, eckP.at(2), eckP.at(3) )), temp4(schnittPunkt(pt1, pt2, eckP.at(3), eckP.at(0) ));
    return temp1 && temp2 && temp3 && temp4;
}
