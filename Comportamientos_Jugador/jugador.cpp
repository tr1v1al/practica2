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

// Búsqueda en anchura del nivel 0
list<Action> getPlanLvl0(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// Búsqueda en anchura del nivel 1
list<Action> getPlanLvl1(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// Dijkstra para nivel 2
list<Action> getPlanLvl2(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// A* para nivel 3
list<Action> getPlanLvl3(stateL01 start, location target, const vector<vector<unsigned char>> &map);

// Ponemos matriz a cero
void setMatrixToNull(vector<vector<unsigned char>> &matrix);

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;
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
					break;
			}
			// Para visualizar el estado de nivel 0-1 es válido incluso para niveles 2-3
			visualizePlan(st01, plan);
			havePlan = true;
		}
		if (havePlan && plan.size() > 0) {
			accion = plan.front();
			plan.pop_front();
		}
		if (plan.size() == 0) {
			havePlan = false;
		}
	} else {
		// Nivel 4
	}

	return accion;
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

		// Generamos hijo actFORWARD
		cost = actionCost(curr_node.st.player_item, actFORWARD, curr_node.st.pos.player, map);
		aux_st.pos = generateChild(actFORWARD, curr_node.st.pos, map);
		aux_st.player_item = currItem(curr_node.st.player_item, aux_st.pos.player, map);
		aux_st.sleep_item = curr_node.st.sleep_item;
		aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actFORWARD};
		if (aux_st.pos.player == target) {
			curr_node = aux_nd;
			solutionFound = true;
			break;	
		} else if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_R
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
		/*
		bool sleepInFOV = viscon(curr_node.st.pos);
		if (sleepInFOV) {
			// Generamos hijo actSON_FORWARD
			cost = actionCost(curr_node.st.sleep_item, actSON_FORWARD, curr_node.st.pos.sleep, map);
			aux_st.pos = generateChild(actSON_FORWARD, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = currItem(curr_node.st.sleep_item, aux_st.pos.sleep, map);
			aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actSON_FORWARD};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			// Generamos hijos actSON_TURN_SR y actSON_TURN_SL		
			cost = actionCost(curr_node.st.sleep_item, actSON_TURN_SR, curr_node.st.pos.sleep, map);
			aux_st.pos = generateChild(actSON_TURN_SR, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actSON_TURN_SR};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			cost = actionCost(curr_node.st.sleep_item, actSON_TURN_SL, curr_node.st.pos.sleep, map);
			aux_st.pos = generateChild(actSON_TURN_SL, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actSON_TURN_SL};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
		}
		*/

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
	int cost = 0;

	bool solutionFound = start.sleep == target;
	frontier.push(curr_node);
	
	while (!solutionFound && !frontier.empty()) {
		frontier.pop();
		explored.insert(curr_node);

		// Generamos hijo actFORWARD
		cost = actionCost(curr_node.st.player_item, actFORWARD, curr_node.st.pos.player, map);
		aux_st.pos = generateChild(actFORWARD, curr_node.st.pos, map);
		aux_st.player_item = currItem(curr_node.st.player_item, aux_st.pos.player, map);
		aux_st.sleep_item = curr_node.st.sleep_item;
		aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actFORWARD};
		if (explored.find(aux_nd) == explored.end()) {
			frontier.push(aux_nd);
		}

		// Generamos hijos actTURN_R y actTURN_R
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
		
		bool sleepInFOV = viscon(curr_node.st.pos);
		if (sleepInFOV) {
			// Generamos hijo actSON_FORWARD
			cost = actionCost(curr_node.st.sleep_item, actSON_FORWARD, curr_node.st.pos.sleep, map);
			aux_st.pos = generateChild(actSON_FORWARD, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = currItem(curr_node.st.sleep_item, aux_st.pos.sleep, map);
			aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actSON_FORWARD};
			if (aux_st.pos.sleep == target) {
				curr_node = aux_nd;
				solutionFound = true;
				break;	
			} else if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			// Generamos hijos actSON_TURN_SR y actSON_TURN_SL		
			cost = actionCost(curr_node.st.sleep_item, actSON_TURN_SR, curr_node.st.pos.sleep, map);
			aux_st.pos = generateChild(actSON_TURN_SR, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actSON_TURN_SR};
			if (explored.find(aux_nd) == explored.end()) {
				frontier.push(aux_nd);
			}
			cost = actionCost(curr_node.st.sleep_item, actSON_TURN_SL, curr_node.st.pos.sleep, map);
			aux_st.pos = generateChild(actSON_TURN_SL, curr_node.st.pos, map);
			aux_st.player_item = curr_node.st.player_item;
			aux_st.sleep_item = curr_node.st.sleep_item;
			aux_nd = {aux_st, curr_node.st, 0, 0, curr_node.f + cost, actSON_TURN_SL};
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

