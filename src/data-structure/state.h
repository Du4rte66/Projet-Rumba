#ifndef __STATE_H__
#define __STATE_H__
#include "utils.h"
#include "stdbool.h"

/**
 * @brief Définition de la structure état représentant l'état du jeux à un instant (les positions de 
 * pions)
 *      nbPion| pions (de bas en haut)
 * pic0 [  3  , 1 , 2 , 3 ]     3 6
 * pic1 [  3  , 4 , 5 , 6 ] --> 2 5   9
 * pic2 [  1  , 7 ,   ,   ]     1 4 7 8
 * pic3 [  2  , 8 , 9 ,   ]     =======
 * 
 */
typedef struct s_state {
    int matrix[4][4];
}State;

/**
 * @brief contient les informations relatives au déplacement d'un anneau
 *
 */
typedef struct s_move {
    int id;         /* numéro(id) de l'anneau. Pre : 1 <= id <= 9*/
    int weight;     /* poids du coup  */
    int g_value;    /* poids du chemin jusqu'à l'état dest */
    int stem_src;   /* tige source. Pre : 0 <= p_src <= 3 */
    int stem_dst;   /* tige destination. Pre : 0 <= p_dst <= 3 */
    int mouv_index ;/* numéro du mouvement (1er, 2eme...)*/
} Move;

/**
 * @brief Contient les informations de déplacement et le résultat obtenu
 *
 */
typedef struct s_action{
    State before; /* Etat avant déplacement */
    Move move;    /* Déplacement */
    State after;  /* Etat après déplacement */
}Action;

void displayState(State e);

void displayMove(Move m);

void displayAction(Action *a);

/**
 * @brief Renvoie les mouvements Possible d'un Etat
 *
 * @param e Etat initiale
 * @return Un tableau de tout les mouvements possible de l'Etat
 */

void findMoves(State e, Move* moves);

/**
 * @brief Renvoie le nouvel Etat suite à un déplacement
 *
 * @param s Etat initiale
 * @param m Mouvement à appliquer
 * @return Nouvel Etat
 */
Action applyMove(State s, Move m);

/**
 * @brief Test si l'état correspond au but
 *
 * @param s Etat testé
 * @return true si égal au but, false sinon
 */
bool stateIsGoal(State s, State but);

/**
 * @brief Test si 2 états son similaires
 *
 * @param s1 Etat 1
 * @param s2 Etat 2
 * @return true si égaux, false sinon
 */
bool equalState(State s1, State s2);

/**
 * @brief Renvoie la liste des mouvement-etats possibles
 *
 * @param s Etat initiale
 * @return res Tableau des mouvement-etats possibles
 * @param nb_move Nombre de mouvement-etats possibles
 */
 void stateFindNextActions(State s, int *nb_move, Action* action_possibles);

/**
 * @brief Test si un coup est valide (respect des prédicats)
 * @param c coup à tester
 * @param fun_name Nom de la fonction appelante
 *
 */
void testMoveIsValid(Move m, char* fun_name);


/**
 * @brief Renvoie si le mouvement est possible
 *
 * @param s Etat initiale
 * @param m Movement à tester sur l'Etat
 * @return true si possible, false sinon
 */
bool isMovePossible(State s, Move m);

/**
 * @brief Test si la valeur d'un pique est valide
 *
 * @param stem Pique à tester
 * @param fun_name Nom de la fonction appelante
 */
void testStemValid(int stem, char* fun_name);


/**
 * @brief Crée une structure Action (malloc)
 * 
 * @param before Etat de depart
 * @param move Mouvement appliqué
 * @param after Etat d'arrivée
 * @return structure Action
 */
Action* createAction(State before, Move move, State after);

/**
 * @brief Supprime l'action et libère la mémoire
 * 
 * @param act Action à supprimer
 */
void deleteAction(Action* act);

/**
 * @brief Rempli l'état de 0
 * 
 * @param s Etat à remplir
 */
void stateEmpty(State* s);

/**
 * @brief Copy les information de src dans dst
 * 
 * @param src Source 
 * @param dst Destination
 */
void copyState(State *src, State *dst);

/**
 * @brief Copy les information de src dans dst
 * 
 * @param src Source 
 * @param dst Destination
 */
void copyMove(Move *src, Move *dst);


 /**
 * @brief Copy les information de src dans dst
 * 
 * @param src Source 
 * @param dst Destination
 */
void copyAction(Action *src, Action *dst);
#endif
