#include "Geometrie.h"
#include "Gerade.h"
#include <QtMath>
#include <algorithm>
#include <iterator>

Repere::Repere():ursp(QPointF()), theta(0.0){}

Repere::Repere(QPointF const& urs, double const& thet):ursp(urs),theta(thet){}

//Repere(Repere const& newRep);

//~Repere();

void Repere::setUrsprung(QPointF const& newUrsp){
    ursp=newUrsp;
}

void Repere::setAngle(double const& newTheta){
    theta=newTheta;
}

QPointF Repere::getUrsprung() const{
    return ursp;
}

double Repere::getAngle() const{
    return theta;
}

FunktObjekt::FunktObjekt(Repere const& rep):m_rep(rep){}

QPointF FunktObjekt::operator()(QPointF const& pt){
    return punktTransfo(pt, m_rep);
}

/**
 * @brief punkt2Ma
 * @param pt
 * @return
 */
QGenericMatrix<1,2,double> punkt2Ma(QPointF const& pt){
    double tab[]={pt.x(),pt.y()};
    return QGenericMatrix<1,2,double>(tab);
}

/**
 * @brief Ma2Punkt
 * @param ma
 * @return
 */
QPointF Ma2Punkt(QGenericMatrix<1,2,double> const& ma){
    double const *tab=ma.data();
    return QPointF(  tab[0], tab[1]    );
}

/**
 * @brief schnittPunkt
 * @param pt1
 * @param pt2
 * @param pt3
 * @param pt4
 * @return
 */
bool schnittPunkt(QPointF const& pt1, QPointF const& pt2, QPointF const& pt3, QPointF const& pt4){
  Gerade G1(Gerade(pt1,pt2)), G2(Gerade(pt3,pt4));
  double temp1(0), temp2(0);
  if(qIsInf(G1.getA1())){
      temp1=(G1.getA0()-pt3.x())*(G1.getA0()-pt4.x());
  }else{
      temp1=( G1.getA1()*pt3.x()+G1.getA0() - pt3.y())*(G1.getA1()*pt4.x()+G1.getA0()-pt4.y());
  }
  if(qIsInf(G2.getA1())){
      temp2=(G2.getA0()-pt1.x())*(G2.getA0()-pt2.x());
  }else{
      temp2=(G1.getA1()*pt1.x()+G1.getA0()- pt1.y())*(G1.getA1()*pt2.x()+G2.getA0()- pt2.y());
  }
  return temp1<=0 && temp2<=0 ;
}

/**
 * @brief RechteckH::rotMatrix
 * @return
 */
QGenericMatrix<2,2,double> rotMatrix(Repere const& rep){
   double theta(rep.getAngle());
   double tab[]={qCos(theta), -qSin(theta), qSin(theta), qCos(theta)};
   return QGenericMatrix<2,2,double>(tab);
}

QGenericMatrix<2,2,double> rotMatrix1(double const& angle){
    double tab[]={qCos(angle), -qSin(angle), qSin(angle), qCos(angle)};
    return QGenericMatrix<2,2,double>(tab);
}

QPointF punktTransfo(QPointF pt, Repere const& rep){
   return Ma2Punkt( rotMatrix(rep).transposed()*punkt2Ma(pt-rep.getUrsprung()) );
}

//QPointF punktTransfo1(QPointF pt, QGenericMatrix<2,2,double> R){
//   return Ma2Punkt( R.transposed()*punkt2Ma(pt-rep.getUrsprung()) );
//}

QVector<QPointF> vecTransfo(QVector<QPointF> const& pts, Repere const& rep){
  QVector<QPointF> result;
  std::transform(pts.begin(), pts.end(), std::back_inserter(result), FunktObjekt(rep));
  return result;
}

//QVector<QPointF> vecTransfo1(QVector<QPointF> const& pts, QGenericMatrix<2,2,double> R);
//QVector<QPointF> result;
//std::transform(pts.begin(), pts.end(), std::back_inserter(result), [](QPointF const& pt){
//           punktTransfo1(pt,R);});

//Repere makeRepere(QVector<QPointF>::iterator it1, QVector<QPointF>::iterator it1, QPointF pos){
//    for(QVector<QPointF>::iterator it(it1);)
//}
