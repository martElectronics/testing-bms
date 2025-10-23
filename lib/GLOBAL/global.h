#ifndef __GLOBALBMS
#define __GLOBALBMS
#include <Arduino.h>
#include <set>


// Estructura para definir un punto de exclusión.
// Estructura y lista para excluir sensores de VOLTAJE
struct VoltExclusionPoint
{
  int idModule;
  int idVolt; // Índice del sensor de voltaje

  bool operator<(const VoltExclusionPoint &other) const
  {
    if (idModule != other.idModule)
      return idModule < other.idModule;
    return idVolt < other.idVolt;
  }
};
std::set<VoltExclusionPoint> voltExclusionList;

// Estructura y lista para excluir sensores de TEMPERATURA
struct TempExclusionPoint
{
  int idModule;
  int idNTC; // Índice del sensor de temperatura (NTC)

  bool operator<(const TempExclusionPoint &other) const
  {
    if (idModule != other.idModule)
      return idModule < other.idModule;
    return idNTC < other.idNTC;
  }
};
std::set<TempExclusionPoint> tempExclusionList;


#endif
