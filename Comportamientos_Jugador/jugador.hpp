#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

// Tipo de dato para la ubicación
struct location{
  int row;
  int col;
  bool operator==(const location &loc) const {
    return row == loc.row && col == loc.col;
  }
  bool operator!=(const location &loc) const {
    return !(row == loc.row && col == loc.col);
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
    return !(player == st.player && sleep == st.sleep && 
    compass_pl == st.compass_pl && compass_sl == st.compass_sl);
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
  stateL01 st;
  stateL01 parent;
  Action act;
  bool operator==(const nodeL01 & node) const {
    return st == node.st;
  }
  bool operator<(const nodeL01 & node) const {
    return st < node.st;
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
