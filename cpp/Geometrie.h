#ifndef GEOMETRIE_H
#define GEOMETRIE_H

#include <QGenericMatrix>
#include <QPointF>

class Repere{
  public:
    Repere();
    Repere(QPointF const& urs, double const& thet);
    //Repere(Repere const& newRep);
    //~Repere();
    void setUrsprung(QPointF const& newUrsp);
    void setAngle(double const& newTheta);
    QPointF getUrsprung() const;
    double getAngle() const;
  private:
    QPointF ursp;
    double theta;
};

class FunktObjekt{
  public:
    FunktObjekt(Repere const& rep);
    QPointF operator()(QPointF const& pt);
  private:
    Repere m_rep;
};

double distanzP1P2(QPointF const& pt1, QPointF const& pt2);
QGenericMatrix<1,2,double> punkt2Ma(QPointF const& pt) ;
QPointF Ma2Punkt(QGenericMatrix<1,2,double> const& ma);
bool schnittPunkt(QPointF const& pt1, QPointF const& pt2, QPointF const& pt3, QPointF const& pt4);
QGenericMatrix<2,2,double> rotMatrix(Repere const& rep);
QGenericMatrix<2,2,double> rotMatrix1(double const& angle);
QPointF punktTransfo(QPointF pt, Repere const& rep);
//QPointF punktTransfo1(QPointF pt, QGenericMatrix<2,2,double> R);
QVector<QPointF> vecTransfo(QVector<QPointF> const& pts, Repere const& rep);
//Repere makeRepere(QVector<QPointF> pts, QPointF pos);

//QVector<QPointF> vecTransfo1(QVector<QPointF> const& pts, QGenericMatrix<2,2,double> R);
#endif // GEOMETRIE_H
