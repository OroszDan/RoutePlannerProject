#pragma once

#include <cmath>

const uint32_t earthRadius = 6371000;  //in metres

class Util
{
public:
    static float_t CalculateDistanceBetweenTwoLatLonsInMetres(const float_t lat1, const float_t lat2, const float_t lon1, const float_t lon2);
};

