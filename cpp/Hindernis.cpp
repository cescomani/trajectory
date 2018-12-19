#include "Hindernis.h"

Hindernis::Hindernis(Punkt pt): m_center(pt){}

Punkt Hindernis::getCenter() const{
  return m_center;
}

void Hindernis::setCenter(Punkt pt){
    m_center=pt;
}
