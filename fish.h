#ifndef FISH_H
#define	FISH_H

typedef struct {
    char *name;
    float min_ph;
    float max_ph;
    float average_temperature;
    char identifier;
} FishParams;

FishParams set_current_fish(FishParams *fish, char identifier);

#endif
