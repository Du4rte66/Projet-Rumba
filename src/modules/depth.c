#include <stdio.h>
#include <stdlib.h>
#include "depth.h"
#include <unistd.h>
#include <time.h>
#include "modules_utils.h"
#include "../data-structure/list.h"
#include "../data-structure/state.h"

State emptyState = {{
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
}};

void addToPath(List* path, Action* act, unsigned* current_depth) {
    /* si à la racine */
    if(act->move.mouv_index == 0) {
        listAdd(path, act);
        *current_depth = 1; // Définir la profondeur à 1 car on ajoute le premier élément
        return;
    }

    /* cherche le parent de act */
    while(!listIsEmpty(path) && !equalState(act->before, listLast(path)->after)) {
        listPop(path);
        (*current_depth)--; // Décrémenter la profondeur car on retire un élément du chemin
    }

    /* si aucun parent : echec */
    if(listIsEmpty(path)) {
        error("Echec ajout au chemin", EXIT_FAILURE);
    } else {
        /* ajout au chemin */
        listAdd(path, act);
        (*current_depth)++; // Incrémenter la profondeur car on ajoute un élément au chemin
    }
}

bool search_depth_main(List* buff, List* done, List *path, State goal, unsigned max_depth, unsigned *created, unsigned *processed, unsigned *ite, unsigned current_depth) {
    Action* cur;    /* etat en train d'être traité */
    Action* next_actions = malloc(sizeof(Action)*16); /* actions depuis cur */
    Action tmp_action;
    bool found = false; /* true si état but trouvé */
    int nb_next_actions, res_search;
    unsigned path_size;

    /* info du programme */
    unsigned nb_created = 0;     /* nombre d'états créés */
    unsigned nb_processed = 0;  /* nombre d'états explorés */
    unsigned nb_ite = 0;            /* nombre d'itérations */

    while(!found && !listIsEmpty(buff) && current_depth <= max_depth) {
        /* information itération */
        nb_ite++;
        nb_processed++;
        loadingBarDepth(listSize(path), max_depth+1, nb_ite);

        /* récupérer prochain élément */
        cur = listPop(buff);

        /* ajouter à la liste des vus */
        listAdd(done, cur);

        /* ajouter au chemin */
        addToPath(path, cur, &current_depth);
        path_size = listSize(path);

        if(equalState(cur->after, goal)) {
            found = true;
        } else if (current_depth < max_depth) {
            /* récupérer les prochaines actions */
            stateFindNextActions(cur->after, &nb_next_actions, next_actions);
            nb_created+=nb_next_actions;

            /* les ajouter au buffer */
            for(int i = 0; i < nb_next_actions; i++) {
                tmp_action = next_actions[i];
                tmp_action.move.mouv_index = path_size;

                /* recherche recherche du nouveau état dans les états déjà traités  */
                res_search = searchElem(done, &tmp_action, equal_action);

                if(res_search == -1 && current_depth + 1 <= max_depth) {
                    /* si jamais rencontré */
                    //ajouté pour réduire le temps de traitement car tres long
                    listAdd(buff, createAction(tmp_action.before, tmp_action.move, tmp_action.after));
                } else if (res_search != -1) {
                    /* si déjà rencontré */
                    if (listGet(done, res_search)->move.mouv_index > (int)path_size) {
                        /* si meilleur chemin */
                        listAdd(buff, createAction(tmp_action.before, 
                            tmp_action.move, tmp_action.after));
                        deleteAction(listRemove(done, res_search));
                        nb_processed--;
                    }
                }
            }
        }
    }
    free(next_actions);
    *created = nb_created;
    *processed = nb_processed;
    *ite = nb_ite; 
    return found;
}

ResSearch* search_depth(State initial_state, State goal, unsigned max_depth) {
    /* Initialisation structure résultat*/
    ResSearch *res = createResSearch(DEPTH, max_depth+2);
    res->depth = max_depth;
    res->initial_state = initial_state;

    /* initialisation outils */
    List *buff = createList(STACK_MAX_SIZE);    /* noeuds à explorer */
    List* path = createList(max_depth+4);       /* chemin actuel */
    List* done = createList(LIST_DONE_SIZE);    /* liste des noeuds vus */

    /* création racine */
    State state; 
    copyState(&emptyState, &state);
    Move mv = {0,0,0,0,0,0};
    Action *act = createAction(state, mv, initial_state);
    listAdd(buff, act);

    /* lancement du timer */
    clock_t start, end;
    start = clock();
    
    /* recherche du chemin */
    res->found = search_depth_main(buff, done, path, goal, max_depth, 
        &(res->nb_state_created), &(res->nb_state_processed), &(res->nb_ite), 0);
    
    /* fin du timer */
    end = clock();
    res->time = ((double) (end - start)) / CLOCKS_PER_SEC;

    /* récupérations du chemin */
    res->size_path = listSize(path) -1;
    for(unsigned i = 1; i < listSize(path); i++) {
        if (i < listSize(path)) {
            copyMove(&(listGet(path, i)->move), &res->path[i-1]);
            res->cost += listGet(path, i)->move.weight;
        } else {
            printf("Erreur: Position invalide dans la liste à l'indice %u.\n", i);
            // Gérer l'erreur ou ignorer l'accès
        }
    }

    /* libération de la mémoire */
    while(!listIsEmpty(done))
        deleteAction(listPop(done));
    while(!listIsEmpty(buff)) {
        deleteAction(listPop(buff));
    }
    deleteList(&path);
    deleteList(&buff);
    deleteList(&done);

    return res;
}
