#ifndef GERADE_H
#define GERADE_H

#include <QPointF>


class Gerade{
  public:
    Gerade();
    Gerade(QPointF const& pt1, QPointF const& pt2);
    Gerade(double a1, double a0);
    // Konstruktor zum Kopieren
    Gerade(Gerade const& ge);
    double distanzZuG(QPointF const& pt ) const;
    double getA1() const;
    double getA0() const;
  private:
    double a_1;
    double a_0;
};

#endif // GERADE_H
