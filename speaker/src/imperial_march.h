#ifndef IMPERIAL_MARCH
#define IMPERIAL_MARCH

#include "tones.h"

struct tone imperial_march[] = {
    {NOTE_A4, 500}, 
    {PAUSE, 1}, 
    {NOTE_A4, 500}, 
    {PAUSE, 1}, 
    {NOTE_A4, 500}, 
    {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 500}, 
    {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 1000},

    {NOTE_E5, 500}, 
    {PAUSE, 1}, 
    {NOTE_E5, 500}, 
    {PAUSE, 1}, 
    {NOTE_E5, 500},
    {NOTE_F5, 350}, {NOTE_C5, 150}, {NOTE_GS4, 500}, 
    {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 1000},

    {NOTE_A5, 500}, {NOTE_A4, 350}, {NOTE_A4, 150}, {NOTE_A5, 500},
    {NOTE_GS5, 250}, {NOTE_G5, 250}, {NOTE_FS5, 125}, {NOTE_F5, 125}, {NOTE_FS5, 250},

    {PAUSE, 250}, {NOTE_AS4, 250}, {NOTE_DS5, 500}, {NOTE_D5, 250}, {NOTE_CS5, 250},
    {NOTE_C5, 125}, {NOTE_B4, 125}, {NOTE_C5, 250},

    {PAUSE, 250}, {NOTE_F4, 250}, {NOTE_GS4, 500}, {NOTE_F4, 375}, {NOTE_A4, 125},
    {NOTE_C5, 500}, {NOTE_A4, 375}, {NOTE_C5, 125}, {NOTE_E5, 1000}, 

    {NOTE_A5, 500}, {NOTE_A4, 350}, {NOTE_A4, 150}, {NOTE_A5, 500},
    {NOTE_GS5, 250}, {NOTE_G5, 250}, {NOTE_FS5, 125}, {NOTE_F5, 125}, {NOTE_FS5, 250},

    {PAUSE, 250}, {NOTE_AS4, 250}, {NOTE_DS5, 500}, {NOTE_D5, 250}, {NOTE_CS5, 250},
    {NOTE_C5, 125}, {NOTE_B4, 125}, {NOTE_C5, 250},

    {PAUSE, 250}, {NOTE_F4, 250}, {NOTE_GS4, 500}, 

    {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 500}, 
    {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 1000},

    {PAUSE, 500}
};

#endif