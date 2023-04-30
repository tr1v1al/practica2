#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

// Tipo de dato para la ubicación
struct location{
  int row, col;
  bool operator==(const location &loc) const {
    return row == loc.row && col == loc.col;
  }
  bool operator!=(const location &loc) const {
    return !(*this == loc);
  }
  bool operator<(const location &loc) const {
    return row < loc.row || row == loc.row && col < loc.col;
  }  
};

// Tipo de dato para el estado del nivel 0-1
struct stateL01{
  location player, sleep;
  Orientacion compass_pl, compass_sl;

  bool operator==(const stateL01 & st) const {
    return player == st.player && sleep == st.sleep && 
    compass_pl == st.compass_pl && compass_sl == st.compass_sl;
  }
  bool operator!=(const stateL01 & st) const {
    return !(*this == st);
  }
  bool operator<(const stateL01 & st) const {
    return player < st.player || player == st.player && sleep < st.sleep ||
    player == st.player && sleep == st.sleep && compass_pl < st.compass_pl ||
    player == st.player && sleep == st.sleep && 
    compass_pl == st.compass_pl && compass_sl < st.compass_sl;
  } 
  /*
  stateL01& operator=(const stateL01& st)
  {
      player = st.player;
      sleep = st.sleep;
      compass_pl = st.compass_pl;
      compass_sl = st.compass_sl;
      return *this;
  }
  */
};

// Tipo de dato para el nodo del nivel 0-1
struct nodeL01{
  stateL01 st, parent;
  Action act;
  bool operator==(const nodeL01 & node) const {
    return st == node.st;
  }
  bool operator<(const nodeL01 & node) const {
    return st < node.st;
  }  
};

// Tipo de dato para el nodo del nivel 2-3
struct stateL23 {
  stateL01 pos;
  int player_item, sleep_item;  // 0 si no tiene nada, 1 si zapatos, 2 si bikini

  bool operator==(const stateL23 & st) const {
    return pos == st.pos && player_item == st.player_item && sleep_item == st.sleep_item;
  }
  bool operator!=(const stateL23 & st) const {
    return !(*this == st);
  }
  bool operator<(const stateL23 & st) const {
    return pos < st.pos || pos == st.pos && player_item < st.player_item || 
    pos == st.pos && player_item == st.player_item && sleep_item < st.sleep_item;
  } 
};

// Tipo de dato para el nodo del nivel 2-3
struct nodeL23 {
  stateL23 st;
  list<Action> path;
  int g, h, f;
  Action act;
  /*
  bool operator==(const nodeL23 & node) const {
    return st == node.st;
  }
  */
  bool operator<(const nodeL23 & node) const {
    return f > node.f;
  } 
  
};

// Clase comparadora para el min heap necesario en niveles 2-3
class myComparator
{
public:
    bool operator() (const nodeL23& n1, const nodeL23& n2)
    {
        return n1.f > n2.f;
    }
};

class myComparator1
{
public:
    bool operator() (const stateL23& st1, const stateL23& st2)
    {
        return st1 < st2;
    }
};


class ComportamientoJugador : public Comportamiento {
  private:
  // Pintamos el plan en el simulador a partir del estado dado
  void visualizePlan(stateL01 st, const list<Action> &plan);

  public:
    // Nivel 4
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
    }
    // Niveles 0-3
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
      havePlan = false;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);


  private:
    // Declarar Variables de Estado
    list<Action> plan;  // Plan de ejecución
    bool havePlan;    // True si tenemos plan
    location player_loc, sleep_loc, goal_loc; // Ubicación del jugador, sonámbulo y objetivo
    Orientacion player_ori, sleep_ori;  // Orientación del jugador y el sonámbulo
};

#endif
