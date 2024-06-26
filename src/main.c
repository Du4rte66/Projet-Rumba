#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "data-structure/state.h"
#include "modules/modules_utils.h"
#include "modules/depth.h"
#include "modules/ida.h"
#include "modules/ida_param.h"
#include "../test/dataTest.h"
#include "modules/modules_utils.h"


Algo getAlgoFromUser() {
    int res;
    printf("Choisir algorithme :\n");
    printf("\t 1 - Profondeur bornée\n");
    printf("\t 2 - IDA* \n");
    printf("Votre choix : ");
    scanf("%d",&res);
    if(res != DEPTH && res != IDA)
        error("Choix de l'algorithme invalide ", EXIT_FAILURE);
    return res;
}

State getStateStartFromUser() {
    int res;
    State start;
    printf("\nChoisir départ :\n");
    printf("\tliste états départ : {1,...,7}\n");
    printf("Votre choix : ");
    scanf("%d",&res);
    if(res < 1 || res > 7)
        error("Choix de l'état départ invalide ", EXIT_FAILURE);
    getEtatStart(res, &start);
    return start;
}

State getStateEndFromUser() {
    int res;
    State end;
    printf("\nChoisir arrivée :\n");
    printf("\tliste états arrivée : {1,...,15}\n");
    printf("Votre choix : ");
    scanf("%d",&res);
    if(res < 1 || res > 15)
        error("Choix de l'état arrivée invalide ", EXIT_FAILURE);
    getEtatEnd(res, &end);
    return end;
}

unsigned getMaxDepthFromUser() {
    int res;
    printf("\nChoisir profondeur max :\n");
    printf("Votre choix : ");
    scanf("%d",&res);
    if(res < 1 )
        error("Choix de la profondeur invalide ", EXIT_FAILURE);
    return res;
}

fun_heuristic getHeuristicFromUser() {
    int res;
    printf("\nChoisir heuristique :\n");
    printf("\t 1 - Etat mal positionné\n");
    printf("\t 2 - Profondeur source\n");
    printf("\t 3 - Profondeur source + destination\n");
    printf("\t 4 - Mal positionné pour cas coup = id\n");
    printf("Votre choix : ");
    scanf("%d",&res);
    if(res < 1 || res > 4)
        error("Choix de l'heuristique invalide ", EXIT_FAILURE);
    return getFunHeuristic(res);
}

fun_move_cost getMoveCostFromUser() {
    int res;
    printf("\nChoisir calcul des coups :\n");
    printf("\t 1 - Coup 1\n");
    printf("\t 2 - Coup = id de l'anneau\n");
    printf("Votre choix : ");
    scanf("%d",&res);
    if(res < 1 || res > 2)
        error("Choix de l'heuristique invalide ", EXIT_FAILURE);
    return getFunMoveCost(res);
}

int main(){
    Algo algo = getAlgoFromUser();

    State start = getStateStartFromUser();
    State end = getStateEndFromUser();

    ResSearch *res;

    if(algo == DEPTH) {
        unsigned max_depth = getMaxDepthFromUser();

        displayState(start);
        displayState(end);

        res = search_depth(start, end, max_depth);
    }

    if (algo == IDA) {
        fun_heuristic heuristic = getHeuristicFromUser();
        fun_move_cost move_cost = getMoveCostFromUser();
        
        displayState(start);
        displayState(end);

        res = ida(start, end, heuristic, move_cost);
    }

    displayResSearch(res);
    printf("ETAT INITIAL:\n\n");
    displayState(start);
    printf("ETAT FINAL:\n\n");
    displayState(end);
    sleep(1);
    printf("MOUVEMENTS :\n\n");
    showGameAnimation(res);
    deleteResSearch(res);

    return 0;
}
