#pragma once

#include <cmath>

const double kBaseAirDensity = 1.2; // at msl, used with tas
const double kPi = std::acos(-1);
inline double degToRad(double deg) { return deg * kPi / 180.0; }