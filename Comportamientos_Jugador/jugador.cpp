#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <queue>

// True si la casilla dada es un obstáculo
bool isObstacle(const location &l, const vector<vector<unsigned char>> &map);

// Devuelve la ubicación de la siguiente casilla a avanzar del agente jugador o sonámbulo
location nextCell(location l, Orientacion ori);

// Generamos hijo dada la accción, no tiene en cuenta que el sonámbulo
// puede no estar en el FOV del jugador!!!
stateL01 generateChild(const Action &a, stateL01 st, const vector<vector<unsigned char>> &map);

// Devuelve true si el sonámbulo está en el FOV del jugador
bool viscon(const stateL01 & st);

// Devuelve curr si en loc no hay objeto, o el código del objeto si hay
int currItem(int curr, const location &loc, const vector<vector<unsigned char>> &map);

// Devuelve el coste en batería de la acción
int actionCost(int item, const Action &a, const location &loc, const vector<vector<unsigned char>> &map);

// Función heurística
int heuristic(const stateL01 & st, const location & target);

// Función heurística del jugador
int heuristicPlayer(const stateL01 & st, const location & target);

// Función heurística del sonámbulo
int heuristicSleep(const stateL01 & st, const location & target);

// Búsqueda en anchura del nivel 0
list<Action> getPlanLvl0(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// Búsqueda en anchura del nivel 1
list<Action> getPlanLvl1(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// Dijkstra para nivel 2
list<Action> getPlanLvl2(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// A* para nivel 3
list<Action> getPlanLvl3(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// A* para nivel 4, devuelve coste del plan
int getPlanLvl4(stateL23 start, location target, const vector<vector<unsigned char>> &map, list<Action> &plan);

// A* para jugador
int getPlanPlayer(stateL23 start, location target, const vector<vector<unsigned char>> &map, list<Action> &plan);

// A* para sonámbulo
int getPlanSleep(stateL23 start, location target, const vector<vector<unsigned char>> &map, list<Action> &plan);

// Ponemos matriz a cero
void setMatrixToNull(vector<vector<unsigned char>> &matrix);

// Función auxiliar que devuelve las coordenadas en el mapa de la celda
// número n(0..15) en el campo de visión actual
pair<int,int> getCoordinates(const stateL01 & st, int n);

// Función auxiliar que devuelve el índice del sonámbulo en los vectores
// terreno/superficie
int getSleepN(const stateL01 &st);

// Función que recupera la posición del jugador en función de la posición del
// sonámbulo y los sensores
void recalibrate(stateL01 &st, int n);

// Función auxiliar que actualiza el mapa auxiliar o el mapa resultado
// (dependiendo de si conocemos nuestra posición) con información sobre
// el terreno que vemos en el momento dado y devuelve el número de casillas nuevas descubiertas
int updateMap(const stateL01 & st, Sensores sens, vector<vector<unsigned char>> & v);

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action action = actIDLE;
	// Niveles 0-3
	if (sensores.nivel != 4) {
		if (!havePlan) {
			// Inicializamos
			player_loc = {sensores.posF, sensores.posC};
			sleep_loc = {sensores.SONposF, sensores.SONposC};
			goal_loc = {sensores.destinoF, sensores.destinoC};
			player_ori = sensores.sentido;
			sleep_ori = sensores.SONsentido;
			stateL01 st01 = {player_loc, sleep_loc, player_ori, sleep_ori};
			// Calculamos plan
			switch(sensores.nivel) {
				case 0:
					plan = getPlanLvl0(st01, goal_loc, mapaResultado);
					break;
				case 1:
					plan = getPlanLvl1(st01, goal_loc, mapaResultado);
					break;
				case 2:
					plan = getPlanLvl2(st01, goal_loc, mapaResultado);
					break;
				case 3:
					plan = getPlanLvl3(st01, goal_loc, mapaResultado);
					break;
			}
			// Para visualizar el estado de nivel 0-1 es válido incluso para niveles 2-3
			visualizePlan(st01, plan);
			havePlan = true;
		}
		if (havePlan && plan.size() > 0) {
			action = plan.front();
			plan.pop_front();
		}
		if (plan.size() == 0) {
			havePlan = false;
		}
	} else {
		// Nivel 4

		// Actualizamos objetivo actual
		goal_loc = {sensores.destinoF, sensores.destinoC};	

		if (!position_known || sensores.reset) {
			position_known = true;
			last_action = actWHEREIS;
			havePlan = false;
			return actWHEREIS;
		}

		// Actualizamos posición de los agentes
		if (last_action == actWHEREIS) {
			curr_state.pos = {{sensores.posF, sensores.posC}, {sensores.SONposF, sensores.SONposC},
			sensores.sentido, sensores.SONsentido};
 		} else if (last_action == actFORWARD) {
			switch (curr_state.pos.compass_pl) {
				case norte: --curr_state.pos.player.row; break;
				case noreste: --curr_state.pos.player.row; ++curr_state.pos.player.col; break;
				case este: ++curr_state.pos.player.col; break;
				case sureste: ++curr_state.pos.player.row; ++curr_state.pos.player.col; break;
				case sur: ++curr_state.pos.player.row; break;
				case suroeste: ++curr_state.pos.player.row; --curr_state.pos.player.col; break;
				case oeste: --curr_state.pos.player.col; break;
				case noroeste: --curr_state.pos.player.row; --curr_state.pos.player.col; break;			
			}
		} else if (last_action == actSON_FORWARD) {
			switch (curr_state.pos.compass_sl) {
				case norte: --curr_state.pos.sleep.row; break;
				case noreste: --curr_state.pos.sleep.row; ++curr_state.pos.sleep.col; break;
				case este: ++curr_state.pos.sleep.col; break;
				case sureste: ++curr_state.pos.sleep.row; ++curr_state.pos.sleep.col; break;
				case sur: ++curr_state.pos.sleep.row; break;
				case suroeste: ++curr_state.pos.sleep.row; --curr_state.pos.sleep.col; break;
				case oeste: --curr_state.pos.sleep.col; break;
				case noroeste: --curr_state.pos.sleep.row; --curr_state.pos.sleep.col; break;			
			}
		} else if (last_action != actIDLE){
			switch (last_action) {
				case actTURN_R:
					curr_state.pos.compass_pl = (Orientacion)((curr_state.pos.compass_pl + 2) % 8);
					break;
				case actTURN_L:
					curr_state.pos.compass_pl = (Orientacion)((curr_state.pos.compass_pl + 6) % 8);
					break;
				case actSON_TURN_SR:
					curr_state.pos.compass_sl = (Orientacion)((curr_state.pos.compass_sl + 1) % 8);
					break;
				case actSON_TURN_SL:
					curr_state.pos.compass_sl = (Orientacion)((curr_state.pos.compass_sl + 7) % 8);
					break;			
			}
		}

		// Caso de haber recibido empujón de un lobo
		if (sensores.colision) {
			bool sleepinfov = false;
			int n = 0;
			for (int i = 0; i < 16; ++i) {
				if (sensores.superficie[i] == 's') {
					sleepinfov = true;
					n = i;
					break;
				}
			}
			if (sleepinfov) {
				recalibrate(curr_state.pos, n);
				havePlan = false;
			} else {
				position_known = true;
				last_action = actWHEREIS;
				havePlan = false;
				return actWHEREIS;
			}
		}

		// Actualizamos el mapa y lo que llevan los agentes
		int discovered = updateMap(curr_state.pos, sensores, mapaResultado);
		curr_state.player_item = currItem(curr_state.player_item, curr_state.pos.player, mapaResultado);
		curr_state.sleep_item = currItem(curr_state.sleep_item, curr_state.pos.sleep, mapaResultado);
		
		// Si vemos algo nuevo replanificamos
		if (discovered > 0) {
			havePlan = false;
		}
		// Si tenemos plan y el siguiente paso es hacia delante del sonámbulo,
		// comprobamos si se queda en el FOV del jugador, si no replanificamos
		if (havePlan && plan.front() == actSON_FORWARD) {
			location l = nextCell(curr_state.pos.sleep, curr_state.pos.compass_sl);
			stateL01 st = curr_state.pos;
			st.sleep = l;			
			bool allowed = viscon(st);
			// Si no se queda en el FOV
			if (!allowed) {	
				// Caso especial de que además se mete en un punto muerto (a los dos lados del jugador)
				// En este caso el jugador al girar lo pierde de vista, y volvería a girar para verlo,
				// entrando en ciclo. Lo resolvemos usando planificador A* conjunto
				int maxdist = max(abs(curr_state.pos.player.row - l.row), 
				abs(curr_state.pos.player.col - l.col));
				if (maxdist == 1) {
					plan.clear();
					getPlanLvl4(curr_state, l, mapaResultado, plan);
					visualizePlan(curr_state.pos, plan);
				} else {
					havePlan = false;
				}
			}
		}
		if (!havePlan) {
			// Calculamos plan
			plan.clear();
			location l = nextCell(curr_state.pos.player, curr_state.pos.compass_pl);
			if (l == curr_state.pos.sleep) {
				getPlanSleep(curr_state, goal_loc, mapaResultado, plan);
			} else {
				getPlanPlayer(curr_state, curr_state.pos.sleep, mapaResultado, plan);
			}
			
			// Para visualizar el estado de nivel 0-1 es válido incluso para niveles 2-3
			visualizePlan(curr_state.pos, plan);
			havePlan = true;
		}
		if (havePlan && plan.size() > 0) {
			action = plan.front();
			if (action == actFORWARD && sensores.superficie[2] != '_') {
				action = actIDLE;
			} else if (action == actSON_FORWARD) {
				location l = nextCell(curr_state.pos.sleep, curr_state.pos.compass_sl);
				stateL01 st = curr_state.pos;
				st.sleep = l;
				int n = getSleepN(st);
				if (sensores.superficie[n] != '_') {
					action = actIDLE;
				} else {
					plan.pop_front();
				}
			} else {
				plan.pop_front();
			}
		}
		if (plan.size() == 0) {
			havePlan = false;
		}

		last_action = action;		
	}

	return action;
}


int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}


list<Action> getPlanLvl0(stateL01 start, location target, const vector<vector<unsigned char>> &map) {
	list<nodeL01> frontier;
	set<nodeL01> explored;
	list<Action> plan;
	stateL01 root_state = {{-1,-1}, {-1,-1}, norte, norte};
	stateL01 aux_st;
	nodeL01 curr_node = {start, root_state, actIDLE};
	nodeL01 aux_nd;

	bool solutionFound = start.player == target;
	frontier.push_back(curr_node);
	
	while (!solutionFound && !frontier.empty()) {
		frontier.pop_front();
		explored.insert(curr_node);

		// Generamos hijo actFORWARD
		aux_st = generateChild(actFORWARD, curr_node.st, map);
		aux_nd = {aux_st, curr_node.st, actFORWARD};
		if (aux_st.player == target) {
			curr_node = aux_nd;
			solutionFound = true;
			break;
		} else if (explored.find(aux_nd) == explored.end()) {
			frontier.push_back(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_R
		aux_st = generateChild(actTURN_R, curr_node.st, map);
		aux_nd = {aux_st, curr_node.st, actTURN_R};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push_back(aux_nd);
		}
		aux_st = generateChild(actTURN_L, curr_node.st, map);
		aux_nd = {aux_st, curr_node.st, actTURN_L};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push_back(aux_nd);
		}

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.front();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop_front();
			curr_node = frontier.empty() ? curr_node : frontier.front();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return plan;
}

list<Action> getPlanLvl1(stateL01 start, location target, const vector<vector<unsigned char>> &map) {
	list<nodeL01> frontier;
	set<nodeL01> explored;
	list<Action> plan;
	stateL01 root_state = {{-1,-1}, {-1,-1}, norte, norte};
	stateL01 aux_st;
	nodeL01 curr_node = {start, root_state, actIDLE};
	nodeL01 aux_nd;

	bool solutionFound = start.sleep == target;
	frontier.push_back(curr_node);
	
	while (!solutionFound && !frontier.empty()) {
		frontier.pop_front();
		explored.insert(curr_node);

		// Generamos hijo actFORWARD
		aux_st = generateChild(actFORWARD, curr_node.st, map);
		aux_nd = {aux_st, curr_node.st, actFORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push_back(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_R
		aux_st = generateChild(actTURN_R, curr_node.st, map);
		aux_nd = {aux_st, curr_node.st, actTURN_R};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push_back(aux_nd);
		}
		aux_st = generateChild(actTURN_L, curr_node.st, map);
		aux_nd = {aux_st, curr_node.st, actTURN_L};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push_back(aux_nd);
		}

		bool sleepInFOV = viscon(curr_node.st);
		if (sleepInFOV) {
			// Generamos hijo actSON_FORWARD
			aux_st = generateChild(actSON_FORWARD, curr_node.st, map);
			aux_nd = {aux_st, curr_node.st, actSON_FORWARD};
			if (aux_st.sleep == target) {
				curr_node = aux_nd;
				solutionFound = true;
				break;
			} else if (explored.find(aux_nd) == explored.end()) {
				frontier.push_back(aux_nd);
			}
			// Generamos hijos actSON_TURN_SR y actSON_TURN_SL		
			aux_st = generateChild(actSON_TURN_SR, curr_node.st, map);
			aux_nd = {aux_st, curr_node.st, actSON_TURN_SR};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push_back(aux_nd);
			}
			aux_st = generateChild(actSON_TURN_SL, curr_node.st, map);
			aux_nd = {aux_st, curr_node.st, actSON_TURN_SL};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push_back(aux_nd);
			}	
		}

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.front();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop_front();
			curr_node = frontier.empty() ? curr_node : frontier.front();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return plan;
} 

list<Action> getPlanLvl2(stateL01 start, location target, const vector<vector<unsigned char>> &map) {
	priority_queue <nodeL23, vector<nodeL23>, myComparator > frontier;
	set<nodeL23> explored;
	list<Action> plan;
	stateL23 aux_st;	
	nodeL23 aux_nd;	

	stateL23 root_state = {{{-1,-1}, {-1,-1}, norte, norte}, 0, 0};
	int pl_it = currItem(0, start.player, map);
	int sl_it = currItem(0, start.sleep, map);
	aux_st = {start, pl_it, sl_it};
	nodeL23 curr_node = {aux_st, root_state, 0, 0 ,0 , actIDLE};
	int cost = 0;

	bool solutionFound = start.player == target;
	frontier.push(curr_node);
	
	while (!solutionFound && !frontier.empty()) {
		frontier.pop();
		explored.insert(curr_node);
		if (curr_node.st.pos.player == target) {
			solutionFound = true;
			break;	
		} 
		// Generamos hijo actFORWARD
		cost = actionCost(curr_node.st.player_item, actFORWARD, curr_node.st.pos.player, map);
		aux_st.pos = generateChild(actFORWARD, curr_node.st.pos, map);
		aux_st.player_item = currItem(curr_node.st.player_item, aux_st.pos.player, map);
		aux_st.sleep_item = curr_node.st.sleep_item;
		aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actFORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_L
		cost = actionCost(curr_node.st.player_item, actTURN_R, curr_node.st.pos.player, map);
		aux_st.pos = generateChild(actTURN_R, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actTURN_R};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		cost = actionCost(curr_node.st.player_item, actTURN_L, curr_node.st.pos.player, map);
		aux_st.pos = generateChild(actTURN_L, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actTURN_L};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.top();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop();
			curr_node = frontier.empty() ? curr_node : frontier.top();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return plan;
}

list<Action> getPlanLvl3(stateL01 start, location target, const vector<vector<unsigned char>> &map) {
	priority_queue <nodeL23, vector<nodeL23>, myComparator > frontier;
	set<nodeL23> explored;
	list<Action> plan;
	stateL23 aux_st;	
	nodeL23 aux_nd;	

	stateL23 root_state = {{{-1,-1}, {-1,-1}, norte, norte}, 0, 0};
	int pl_it = currItem(0, start.player, map);
	int sl_it = currItem(0, start.sleep, map);
	aux_st = {start, pl_it, sl_it};
	nodeL23 curr_node = {aux_st, root_state, 0, 0 ,0 , actIDLE};
	int g = 0, h = 0;

	bool solutionFound = start.sleep == target;
	frontier.push(curr_node);
	
	while (!solutionFound && !frontier.empty()) {
		frontier.pop();
		explored.insert(curr_node);
		if (curr_node.st.pos.sleep == target) {
			solutionFound = true;
			break;	
		}
		// Generamos hijo actFORWARD
		aux_st.pos = generateChild(actFORWARD, curr_node.st.pos, map);
		aux_st.player_item = currItem(curr_node.st.player_item, aux_st.pos.player, map);
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actFORWARD, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristic(aux_st.pos, target);	
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actFORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_L
		aux_st.pos = generateChild(actTURN_R, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actTURN_R, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristic(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actTURN_R};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		aux_st.pos = generateChild(actTURN_L, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actTURN_L, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristic(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actTURN_L};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}
		
		bool sleepInFOV = viscon(curr_node.st.pos);
		if (sleepInFOV) {
			// Generamos hijo actSON_FORWARD
			aux_st.pos = generateChild(actSON_FORWARD, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = currItem(curr_node.st.sleep_item, aux_st.pos.sleep, map);
			g = actionCost(curr_node.st.sleep_item, actSON_FORWARD, curr_node.st.pos.sleep, map);
			g += curr_node.g;	
			h = heuristic(aux_st.pos, target);
			aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_FORWARD};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			// Generamos hijos actSON_TURN_SR y actSON_TURN_SL		
			aux_st.pos = generateChild(actSON_TURN_SR, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			g = actionCost(curr_node.st.sleep_item, actSON_TURN_SR, curr_node.st.pos.sleep, map);
			g += curr_node.g;	
			h = heuristic(aux_st.pos, target);
			aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_TURN_SR};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			
			aux_st.pos = generateChild(actSON_TURN_SL, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			g = actionCost(curr_node.st.sleep_item, actSON_TURN_SL, curr_node.st.pos.sleep, map);
			g += curr_node.g;	
			h = heuristic(aux_st.pos, target);
			aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_TURN_SL};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
		}
		

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.top();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop();
			curr_node = frontier.empty() ? curr_node : frontier.top();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return plan;
}

int getPlanLvl4(stateL23 start, location target, const vector<vector<unsigned char>> &map, list<Action> &plan) {
	priority_queue <nodeL23, vector<nodeL23>, myComparator > frontier;
	set<nodeL23> explored;
	stateL23 aux_st;	
	nodeL23 aux_nd;	

	stateL23 root_state = {{{-1,-1}, {-1,-1}, norte, norte}, 0, 0};
	nodeL23 curr_node = {start, root_state, 0, 0 ,0 , actIDLE};
	int g = 0, h = 0, cost = 0;
	bool solutionFound = false, sleepInFOV = false;
	frontier.push(curr_node);
	
	while (!frontier.empty()) {
		frontier.pop();
		explored.insert(curr_node);
		if (curr_node.st.pos.sleep == target) {
			solutionFound = true;
			break;	
		} 
		// Generamos hijo actFORWARD
		aux_st.pos = generateChild(actFORWARD, curr_node.st.pos, map);
		aux_st.player_item = currItem(curr_node.st.player_item, aux_st.pos.player, map);
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actFORWARD, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristic(aux_st.pos, target);	
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actFORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_L
		aux_st.pos = generateChild(actTURN_R, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actTURN_R, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristic(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actTURN_R};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		aux_st.pos = generateChild(actTURN_L, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actTURN_L, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristic(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actTURN_L};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}
		
		sleepInFOV = viscon(curr_node.st.pos);
		if (sleepInFOV) {
			// Generamos hijo actSON_FORWARD
			aux_st.pos = generateChild(actSON_FORWARD, curr_node.st.pos, map);
			sleepInFOV = viscon(aux_st.pos);
			// Solo permitimos que el sonámbulo se mueva hacia adelante si se mantiene
			// en el FOV del jugador (en caso contrario podría caerse por un precipicio
			// inexplorado o chocar con un NPC)
			if (sleepInFOV) {
				aux_st.player_item = curr_node.st.player_item;
				aux_st.sleep_item = currItem(curr_node.st.sleep_item, aux_st.pos.sleep, map);
				g = actionCost(curr_node.st.sleep_item, actSON_FORWARD, curr_node.st.pos.sleep, map);
				g += curr_node.g;	
				h = heuristic(aux_st.pos, target);
				aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_FORWARD};
				if (explored.find(aux_nd) == explored.end()) {
					frontier.push(aux_nd);
				}
			}

			// Generamos hijos actSON_TURN_SR y actSON_TURN_SL		
			aux_st.pos = generateChild(actSON_TURN_SR, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			g = actionCost(curr_node.st.sleep_item, actSON_TURN_SR, curr_node.st.pos.sleep, map);
			g += curr_node.g;	
			h = heuristic(aux_st.pos, target);
			aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_TURN_SR};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			
			aux_st.pos = generateChild(actSON_TURN_SL, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			g = actionCost(curr_node.st.sleep_item, actSON_TURN_SL, curr_node.st.pos.sleep, map);
			g += curr_node.g;	
			h = heuristic(aux_st.pos, target);
			aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_TURN_SL};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
		}
		

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.top();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop();
			curr_node = frontier.empty() ? curr_node : frontier.top();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		cost = curr_node.f;
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return cost;	
}

int getPlanPlayer(stateL23 start, location target, const vector<vector<unsigned char>> &map, list<Action> &plan) {
	priority_queue <nodeL23, vector<nodeL23>, myComparator > frontier;
	set<nodeL23> explored;
	stateL23 aux_st;	
	nodeL23 aux_nd;	

	stateL23 root_state = {{{-1,-1}, {-1,-1}, norte, norte}, 0, 0};
	nodeL23 curr_node = {start, root_state, 0, 0 ,0 , actIDLE};
	int g = 0, h = 0, cost = 0;
	bool solutionFound = false;

	frontier.push(curr_node);
	
	while (!frontier.empty()) {
		frontier.pop();
		explored.insert(curr_node);
		
		// Condición de parada
		if (curr_node.st.pos.player == target) {
			solutionFound = true;
			break;	
		} else if (curr_node.st.pos.sleep == target){
			// Si el objetivo es el sonámbulo y con un actFORWARD chocamos con él
			// hemos llegado al objetivo
			location l = nextCell(curr_node.st.pos.player, curr_node.st.pos.compass_pl);
			if (l == target) {
				solutionFound = true;
				break;
			}
		}

		// Generamos hijo actFORWARD
		aux_st.pos = generateChild(actFORWARD, curr_node.st.pos, map);
		aux_st.player_item = currItem(curr_node.st.player_item, aux_st.pos.player, map);
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actFORWARD, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristicPlayer(aux_st.pos, target);	
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actFORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_L
		aux_st.pos = generateChild(actTURN_R, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actTURN_R, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristicPlayer(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actTURN_R};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		aux_st.pos = generateChild(actTURN_L, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.player_item, actTURN_L, curr_node.st.pos.player, map);
		g += curr_node.g;	
		h = heuristicPlayer(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actTURN_L};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.top();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop();
			curr_node = frontier.empty() ? curr_node : frontier.top();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		cost = curr_node.f;
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return cost;	
}

int getPlanSleep(stateL23 start, location target, const vector<vector<unsigned char>> &map, list<Action> &plan) {
	priority_queue <nodeL23, vector<nodeL23>, myComparator > frontier;
	set<nodeL23> explored;
	stateL23 aux_st;	
	nodeL23 aux_nd;	

	stateL23 root_state = {{{-1,-1}, {-1,-1}, norte, norte}, 0, 0};
	nodeL23 curr_node = {start, root_state, 0, 0 ,0 , actIDLE};
	int g = 0, h = 0, cost = 0;
	bool solutionFound = false, sleepInFOV = false;

	frontier.push(curr_node);
	
	while (!frontier.empty()) {
		frontier.pop();
		explored.insert(curr_node);

		if (curr_node.st.pos.sleep == target) {
			solutionFound = true;
			break;	
		} 
		
		// Generamos hijo actSON_FORWARD
		aux_st.pos = generateChild(actSON_FORWARD, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = currItem(curr_node.st.sleep_item, aux_st.pos.sleep, map);
		g = actionCost(curr_node.st.sleep_item, actSON_FORWARD, curr_node.st.pos.sleep, map);
		g += curr_node.g;	
		h = heuristicSleep(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_FORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actSON_TURN_SR y actSON_TURN_SL		
		aux_st.pos = generateChild(actSON_TURN_SR, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.sleep_item, actSON_TURN_SR, curr_node.st.pos.sleep, map);
		g += curr_node.g;	
		h = heuristicSleep(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_TURN_SR};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}
		
		aux_st.pos = generateChild(actSON_TURN_SL, curr_node.st.pos, map);
		aux_st.player_item = curr_node.st.player_item;
		aux_st.sleep_item = curr_node.st.sleep_item;
		g = actionCost(curr_node.st.sleep_item, actSON_TURN_SL, curr_node.st.pos.sleep, map);
		g += curr_node.g;	
		h = heuristicSleep(aux_st.pos, target);
		aux_nd = {aux_st, curr_node.st, g, h, g+h, actSON_TURN_SL};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Elegimos nodo actual. Tiene que ser uno de la frontera
		// que no haya sido explorado
		curr_node = frontier.empty() ? curr_node : frontier.top();
		while (!frontier.empty() && explored.find(curr_node) != explored.end()) {
			frontier.pop();
			curr_node = frontier.empty() ? curr_node : frontier.top();
		}
	}

	// Recuperamos la solución
	if (solutionFound) {
		auto it = explored.begin();
		cost = curr_node.f;
		while (curr_node.parent != root_state) {
			plan.push_front(curr_node.act);
			curr_node.st = curr_node.parent;
			it = explored.find(curr_node);
			curr_node = *it;
		}
	}

	return cost;	
}

int heuristic(const stateL01 & st, const location & target) {
	int sleep_dist = max(abs(st.sleep.row - target.row), abs(st.sleep.col - target.col));
	//bool diagonal = st.compass_pl == noreste || st.compass_pl == noroeste || 
	//				st.compass_pl == sureste || st.compass_pl == suroeste;
	int player_dist = max(max(abs(st.player.row - st.sleep.row), abs(st.player.col - st.sleep.col))-4, 0);
	int third_dist = max(max(abs(st.sleep.row - target.row), abs(st.sleep.col - target.col))-8, 0);
	return player_dist + sleep_dist + third_dist;
}

int heuristicPlayer(const stateL01 & st, const location & target) {
	return max(max(abs(st.player.row - target.row), abs(st.player.col - target.col))-1, 0);
}

int heuristicSleep(const stateL01 & st, const location & target) {
	return max(max(abs(st.sleep.row - target.row), abs(st.sleep.col - target.col))-1, 0);
}

bool isObstacle(const location &l, const vector<vector<unsigned char>> &map) {
	return map[l.row][l.col] == 'P' || map[l.row][l.col] == 'M';
}

location nextCell(location l, Orientacion ori) {
	l.row = ori == norte || ori == noreste || ori == noroeste ? l.row-1 : 
	(ori == sur || ori == sureste || ori == suroeste ? l.row+1 : l.row);
	l.col = ori == oeste || ori == noroeste || ori == suroeste ? l.col-1 : 
	(ori == este || ori == sureste || ori == noreste ? l.col+1 : l.col);
	return l;
}

stateL01 generateChild(const Action &a, stateL01 st, const vector<vector<unsigned char>> &map) {
	location loc;
	switch(a) {
		case actFORWARD:
			loc = nextCell(st.player, st.compass_pl);
			if (!isObstacle(loc, map) && loc != st.sleep) {
				st.player = loc;
			}
			break;
		case actTURN_R:
			st.compass_pl = (Orientacion)((st.compass_pl + 2) % 8);
			break;
		case actTURN_L:
			st.compass_pl = (Orientacion)((st.compass_pl + 6) % 8);
			break;
		case actSON_FORWARD:
			loc = nextCell(st.sleep, st.compass_sl);
			if (!isObstacle(loc, map) && loc != st.player) {
				st.sleep = loc;
			}
			break;
		case actSON_TURN_SR:
			st.compass_sl = (Orientacion)((st.compass_sl + 1) % 8);
			break;
		case actSON_TURN_SL:
			st.compass_sl = (Orientacion)((st.compass_sl + 7) % 8);
			break;
	}
	return st;
}

int currItem(int curr, const location &loc, const vector<vector<unsigned char>> &map) {
	char c = map[loc.row][loc.col];
	return c == 'D' ? 1 : (c == 'K' ? 2 : curr);
}

int actionCost(int item, const Action &a, const location &loc, const vector<vector<unsigned char>> &map) {
	char c = map[loc.row][loc.col];
	int cost = 1;
	switch(a) {
		case actFORWARD:
			switch(c) {
				case 'A':
					cost = item == 2 ? 10 : 100;
					break;
				case 'B':
					cost = item == 1 ? 15 : 50;
					break;
				case 'T':
					cost = 2;
					break;
			}
			break;
		case actTURN_R: case actTURN_L:
			switch(c) {
				case 'A':
					cost = item == 2 ? 5 : 25;
					break;
				case 'B':
					cost = item == 1 ? 1 : 5;
					break;
				case 'T':
					cost = 2;
					break;
			}
			break;
		case actSON_FORWARD:
			switch(c) {
				case 'A':
					cost = item == 2 ? 10 : 100;
					break;
				case 'B':
					cost = item == 1 ? 15 : 50;
					break;
				case 'T':
					cost = 2;
					break;
			}
			break;
		case actSON_TURN_SR: case actSON_TURN_SL:
			switch(c) {
				case 'A':
					cost = item == 2 ? 2 : 7;
					break;
				case 'B':
					cost = item == 1 ? 1 : 3;
					break;
				case 'T':
					cost = 1;
					break;
			}
			break;
	}
	return cost;
}

bool viscon(const stateL01 & st){
	Orientacion ori = st.compass_pl;
	int x = st.sleep.row - st.player.row, y = st.sleep.col - st.player.col;
	switch (ori) {
		case norte: case noreste:
			break;
		case este: case sureste:
			y = -y;
			swap(x, y);		
			break;
		case sur: case suroeste:
			x = -x;
			y = -y;			
			break;
		case oeste: case noroeste:
			x = -x;
			swap(x,y);			
			break;
	}
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j <= 2*i; ++j) {
			if (ori == norte || ori == sur || ori == este || ori == oeste) {
				if (x == -i && y == j-i) {
					return true;
				}
			} else {
				if (j <= i && x == -i && y == j || j > i && x == j-2*i && y == i) {
					return true;
				}
			}
		}
	}
	return false;
}

void setMatrixToNull(vector<vector<unsigned char>> &matrix){
	for (int i = 0; i < matrix.size(); ++i) {
		for (int j = 0; j < matrix[0].size(); ++j) {
			matrix[i][j] = 0;
		}
	}
}

void ComportamientoJugador::visualizePlan(stateL01 st, const list<Action> &plan) {
	setMatrixToNull(mapaConPlan);
	auto it = plan.begin();

	while (it != plan.end()) {
		st = generateChild(*it, st, mapaResultado);
		switch(*it) {
			case actFORWARD:
				mapaConPlan[st.player.row][st.player.col] = 1;
				break;
			case actSON_FORWARD:
				mapaConPlan[st.sleep.row][st.sleep.col] = 2;
				break;
		}
		it++;
	}
}

pair<int,int> getCoordinates(const stateL01 & st, int n) {
	// Posición y orientación actual
	Orientacion ori = st.compass_pl;
	int x = st.player.row, y = st.player.col;
	int r = 0, c = 0, rot_r = 0, rot_c = 0;
	// Contador de celda de terreno
	int count = 0;

	// Calculo coordenadas de celda que interesa como si estuviesemos 
	// situados en el origen y mirando hacia el norte/noreste
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j <= 2*i; ++j) {
			if (count == n) {
				if (ori == norte || ori == sur || ori == este || ori == oeste) {
					r = -i;
					c = j-i;
				} else {
					r = j <= i ? -i : j-2*i;
					c = j <= i ? j : i;
				}
			}
			++count;
		}
	}

	// Rotamos
	switch (ori) {
	case norte: case noreste:
		rot_r = r;
		rot_c = c;
		break;
	case este: case sureste:
		rot_r = c;
		rot_c = -r;			
		break;
	case sur: case suroeste:
		rot_r = -r;
		rot_c = -c;			
		break;
	case oeste: case noroeste:
		rot_r = -c;
		rot_c = r;			
		break;
	}

	// Trasladamos
	rot_r += x;
	rot_c += y;
	
	return {rot_r, rot_c};
}

int getSleepN(const stateL01 &st) {
	pair<int,int> p = {st.sleep.row, st.sleep.col};
	for (int i = 0; i < 16; ++i) {
		if (getCoordinates(st, i) == p) {
			return i;
		}
	}
	return 0;
}

void recalibrate(stateL01 &st, int n) {
	auto coord = getCoordinates(st, n);
	st.player.row += coord.first - st.sleep.row;
	st.player.col += coord.second - st.sleep.col;
}

int updateMap(const stateL01 & st, Sensores sens, vector<vector<unsigned char>> & v) {
	int n = 0;
	pair<int,int> point;
	for (int i = 0; i < 16; ++i) {
		point = getCoordinates(st, i);
		if (v[point.first][point.second] == '?') {
			++n;
		}
		v[point.first][point.second] = sens.terreno[i];
	}
	return n;
}