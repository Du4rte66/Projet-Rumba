#include "dataTest.h"
#include "../src/data-structure/state.h"
#include <stdlib.h>

/* Association des tests : 
    start -> end
    1     -> 1,2
    2     -> 3,4,5,6
    3     -> 7
    4     -> 8
    5     -> 9
    6     -> 10,11,12,13,14
    7     -> 15
*/


/*
    ===================
    ====== STARTS =====
    ===================
*/


State start_1 = {
    {{2,3,2,0},
    {1,1,0,0},
    {3,6,5,4},
    {3,9,8,7}}
};  

State start_2 = {
    {{3,3,2,1},
    {3,6,5,4},
    {3,9,8,7},
    {0,0,0,0}}
};

State start_3 = {
    {{0,0,0,0},
    {3,1,2,3},
    {3,4,5,6},
    {3,7,8,9}}
};

State start_4 = {
    {{3,1,5,9},
    {2,2,6,0},
    {2,3,7,0},
    {2,4,8,0}}
};

State start_5 = {
    {{3,1,2,3},
    {3,4,5,6},
    {3,7,8,9},
    {0,0,0,0}}
};  

State start_6 = {
    {{3,1,2,3},
    {3,4,5,6},
    {3,7,8,9},
    {0,0,0,0}}
}; 

State start_7 = {
    {{3,1,2,3},
    {3,4,5,6},
    {3,7,8,9},
    {0,0,0,0}}
};  

/*
    ====================
    ======= ENDS =======
    ====================
*/


State end_1 = {
    {{3,3,2,1},
    {0,0,0,0},
    {3,6,5,4},
    {3,9,8,7}}
};

State end_2 = {
    {{3,3,2,1},
    {3,7,8,9},
    {3,6,5,4},
    {0,0,0,0}}
};

State end_3 = {
    {{3,3,2,7},
    {3,6,4,8},
    {3,9,5,1},
    {0,0,0,0}}
};

State end_4 = {
    {{3,3,1,2},
    {3,6,4,5},
    {3,9,7,8},
    {0,0,0,0}}
};

State end_5 = {
    {{3,3,2,8},
    {2,6,4,0},
    {3,9,7,5},
    {1,1,0,0}}
};

State end_6 = {
    {{3,4,7,1},
    {3,5,8,2},
    {3,6,9,3},
    {0,0,0,0}}
};

State end_7 = {
    {{3,6,3,9},
    {2,1,2,0},
    {2,4,5,0},
    {2,7,8,0}}
};

State end_8 = {
    {{3,1,5,9},
    {2,2,6,0},
    {2,3,7,0},
    {2,4,8,0}}
};

State end_9 = {
    {{3,8,1,7},
    {2,2,6,0},
    {2,3,5,0},
    {2,4,9,0}}
};

State end_10 = {
    {{2,1,2,0},
    {3,4,5,6},
    {3,7,8,9},
    {1,3,0,0}}
};

State end_11 = {
    {{1,1,0,0},
    {3,4,5,6},
    {3,7,8,9},
    {2,3,2,0}}
};

State end_12 = {
    {{2,1,9,0},
    {3,4,5,6},
    {2,7,8,0},
    {2,3,2,0}}
};

State end_13 = {
    {{3,1,9,6},
    {2,4,5,0},
    {2,7,8,0},
    {2,3,2,0}}
};


State end_14 = {
    {{3,1,9,6},
    {2,4,5,0},
    {1,7,0,0},
    {3,3,2,8}}
};

State end_15 = {
    {{1,6,0,0},
    {3,9,2,1},
    {2,7,8,0},
    {3,3,5,4}}
};

/*
>> 1
{3,1,2,0},
{3,4,5,6},
{3,7,8,9},
{0,3,0,0}
>> 2
{3,1,2,6},
{3,4,5,0},
{3,7,8,9},
{0,3,0,0}
>> 3
{3,1,2,6},
{3,4,0,0},
{3,7,8,9},
{0,3,5,0}
>> 4
{3,1,2,6},
{3,0,0,0},
{3,7,8,9},
{0,3,5,4}
>> 5
{3,1,2,6},
{1,9,0,0},
{2,7,8,0},
{3,3,5,4}
>> 6
{3,1,2,0},
{3,9,0,0},
{3,7,8,6},
{0,3,5,4}
>> 7
{1,1,0,0},
{2,9,2,0},
{3,7,8,6},
{3,3,5,4}
>> 8
{0,0,0,0},
{3,9,2,1},
{3,7,8,6},
{3,3,5,4}
>> 9
{1,6,0,0},
{3,9,2,1},
{2,7,8,0},
{3,3,5,4}
>> 10
{2,6,1,0},
{2,9,2,0},
{2,7,8,0},
{3,3,5,4}
>> 11
{3,6,1,8},
{2,9,2,0},
{1,7,0,0},
{3,3,5,4}
*/

int stateToCopy[4][4];
 
void getEtatStart(int id, State *start) {
    switch (id) 
    {
        case 1:
            *start = start_1;
            break;
        case 2:
            *start = start_2;
            break;
        case 3:
            *start = start_3;
            break;
        case 4:
            *start = start_4;
            break;
        case 5:
            *start = start_5;
            break;
        case 6:
            *start = start_6;
            break;
        case 7:
            *start = start_7;
            break;
    }
}

void getEtatEnd(int id, State *end) {
    switch (id) 
    {
        case 1:
            *end = end_1;
            break;
        case 2:
            *end = end_2;
            break;
        case 3:
            *end = end_3;
            break;
        case 4:
            *end = end_4;
            break;
        case 5:
            *end = end_5;
            break;
        case 6:
            *end = end_6;
            break;
        case 7:
            *end = end_7;
            break;
        case 8:
            *end = end_8;
            break;
        case 9:
            *end = end_9;
            break;
        case 10:
            *end = end_10;
            break;
        case 11:
            *end = end_11;
            break;
        case 12:
            *end = end_12;
            break;
        case 13:
            *end = end_13;
            break;
        case 14:
            *end = end_14;
            break;
        case 15:
            *end = end_15;
            break;
    }
}