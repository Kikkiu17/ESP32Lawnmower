#ifndef _MAP_H_
#define _MAP_H_

#include <Arduino.h>
#include "maputils.h"
#include "psramvector.h"
#include "SETTINGS.h"

class Map
{
public:
    psvec<Point> a_star(psvec<Point>* obstacles, Point source, Point target);
    void initArrays();
    void freeMemory();
};

#endif
