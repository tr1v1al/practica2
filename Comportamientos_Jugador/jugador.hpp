#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <set>

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
  mutable stateL23 parent;
  mutable int g, h, f;
  Action act;
  bool operator==(const nodeL23 & node) const {
    return st == node.st;
  }
  bool operator<(const nodeL23 & node) const {
    return st < node.st;
  } 
};

// Clase comparadora para el min heap necesario en niveles 2-3
class myComparator
{
public:
    int operator() (const nodeL23& n1, const nodeL23& n2)
    {
        return n1.f > n2.f;
    }
};

class distComp
{
public:
    int operator() (const pair<int, location>& p1, const pair<int, location>& p2)
    {
      return p1.first < p2.first;
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
      havePlan = false;
      position_known = false;
      seek_sleep = false;
      displaced = false;
      have_plan_battery = false;
      follow_priority = false;
      player_item = 0;
      sleep_item = 0;
      spent_battery = 0;
      elapsed_time = 0;
      turns_w_charging = 0;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < size; ++j) {
          mapaResultado[i][j] = 'P';
          mapaResultado[size-i-1][j] = 'P';
        }
        for (int j = 0; j < size; ++j) {
          mapaResultado[j][i] = 'P';
          mapaResultado[j][size-i-1] = 'P';
        }
      }
      aux_map.resize(size+2*margin);
      for (int i = 0; i < size+2*margin; ++i) {
        aux_map[i].resize(size+2*margin);
      }
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
    bool havePlan, position_known;    // True si tenemos plan
    Action last_action;
    location player_loc, sleep_loc, goal_loc; // Ubicación del jugador, sonámbulo y objetivo
    Orientacion player_ori, sleep_ori;  // Orientación del jugador y el sonámbulo
    int player_item, sleep_item;
    stateL23 curr_state;
    vector< vector<unsigned char>> aux_map;
    const int margin = 30;
    bool displaced, seek_sleep;
    int spent_battery, elapsed_time, turns_w_charging;
    bool have_plan_battery, follow_priority;
    location priority_target;
    set<location> superchargers;
};

#endif
