#include "fish.h"

FishParams set_current_fish(FishParams *fish, char identifier) {
    switch (identifier) {
        case 1:
            fish->identifier = identifier;
            fish->max_ph = 8;
            fish->min_ph = 6;
            fish->average_temperature = 23.5;
            break;

        case 2:
            fish->identifier = identifier;
            fish->max_ph = 8;
            fish->min_ph = 6.5;
            fish->average_temperature = 25;
            break;

        case 3:
            fish->identifier = identifier;
            fish->max_ph = 7.8;
            fish->min_ph = 5.5;
            fish->average_temperature = 26;
            break;

        case 4:
            fish->identifier = identifier;
            fish->max_ph = 8.5;
            fish->min_ph = 6.5;
            fish->average_temperature = 27.5;
            break;

        case 5:
            fish->identifier = identifier;
            fish->max_ph = 7.9;
            fish->min_ph = 6;
            fish->average_temperature = 23;
            break;

        case 6:
            fish->identifier = identifier;
            fish->max_ph = 7;
            fish->min_ph = 6;
            fish->average_temperature = 30;
            break;
    }
    return *fish;
}

