#ifndef _MAPUTILS_H_
#define _MAPUTILS_H_

#include "SETTINGS.h"

class Point
{
public:
    int16_t x = 0;
    int16_t y = 0;
    int16_t id = 0;
    Point() {}
    Point(int16_t inx, int16_t iny) { x = inx; y = iny; }
    Point(int16_t inx, int16_t iny, int16_t inid) { x = inx; y = iny; id = inid; }

    bool operator==(const Point& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const
    {
        return x != other.x || y != other.y;
    }

    bool operator<(const Point& other) const
    {
        return x < other.x || (!(other.x < x) && y < other.y);
    }

    bool operator>(const Point& other) const
    {
        return x > other.x || (!(other.x > x) && y > other.y);
    }
};


// Punto con anche informazioni su blocco, idx, id. Usare Point se queste informazioni non servono, dato che è più leggero (14 byte più leggero)
class MapPoint
{
public:
    int32_t x = 0;
    int32_t y = 0;
    int32_t block = 0;
    uint16_t idx = 0;
    uint8_t id = 0;
    bool exists = true;

    MapPoint(int32_t inx, int32_t iny, int32_t inblock, uint16_t inidx, uint32_t inid)
    {
        x = inx;
        y = iny;
        block = inblock,
        idx = inidx;
        id = inid;
    }

    MapPoint(bool does_exist) { exists = does_exist; }
    MapPoint() {}
};


class OneDimensionPoint
{
public:
    int32_t value = 0;
    int32_t block = 0;
    uint16_t idx = 0;
    uint8_t id = 0;
    bool exists = true;

    OneDimensionPoint(int32_t invalue, int32_t inblock, uint16_t inidx, uint32_t inid)
    {
        value = invalue;
        block = inblock,
        idx = inidx;
        id = inid;
    }

    OneDimensionPoint(bool does_exist) { exists = does_exist; }
    OneDimensionPoint() {}
};


class MapCreationInfo
{
    public:
        uint32_t fill_start_pos = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        int32_t minx = 0;
        int32_t miny = 0;
        int32_t maxx = 0;
        int32_t maxy = 0;
        uint32_t height_in_squares = 0;
        uint32_t width_in_squares = 0;
        bool is_processed = false;
};


#endif
