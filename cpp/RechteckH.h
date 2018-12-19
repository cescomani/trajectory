#ifndef RECHTECKH_H
#define RECHTECKH_H

#include "Gerade.h"
#include <QGenericMatrix>
#include <QVector>

class RechteckH {
public:
    RechteckH();
    RechteckH(QPointF center, double laenge, double breite, double theta);
    RechteckH(RechteckH const& re);
    QPointF getCenter() const;
    double getLaenge() const;
    double getBreite() const;
    double getTheta() const;
    void setTheta(double const& newTheta);
    void setCenter(QPointF const& newCe);
    bool imRechtEck(QPointF const& pt) const;
    QVector<QPointF> eckPunkte() const;
    QVector<QPointF> eckPunkte1(QGenericMatrix<2,2,double> const& R) const;
    QVector<Gerade> seitenGerade() const;
    QVector<Gerade> seitenGerade1(QGenericMatrix<2,2,double> const& R) const;
    QVector<Gerade> seitenGerade2(QVector<QPointF> const& eckP) const;
    double distanzZuH(QPointF const& pt) const;
    bool imKontakt(QPointF const& pt1, QPointF const& pt2) const;

private:
    QPointF m_center;
    double m_laenge;
    double m_breite;
    double m_theta;
};

#endif // RECHTECKH_H
