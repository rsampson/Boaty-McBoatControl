// navigation.h - autopilot maths. No hardware access, so it can be tested on a PC.
#pragma once

#include <math.h>
#include <stdint.h>

// Wrap an angle into -180..+180 (for "how far and which way to turn").
static inline float navWrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// Wrap an angle into 0..360 (for compass headings and bearings).
static inline float navWrap360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

// Distance (metres) and true bearing (degrees, 0 = north, 90 = east) from one
// position to another. Positions are degrees x 10^7. Uses a flat-earth
// approximation, which is accurate to well under a metre over a few kilometres.
static inline void navDistanceBearing(int32_t latE7, int32_t lonE7, int32_t toLatE7,
                                      int32_t toLonE7, float &distanceM, float &bearingDeg) {
  const double EARTH_RADIUS_M = 6371000.0;
  const double DEG_TO_RADIANS = M_PI / 180.0;
  double dLat = ((double)toLatE7 - (double)latE7) * 1e-7 * DEG_TO_RADIANS;
  double dLon = ((double)toLonE7 - (double)lonE7) * 1e-7 * DEG_TO_RADIANS;
  double meanLat = ((double)toLatE7 + (double)latE7) * 0.5e-7 * DEG_TO_RADIANS;
  double north = dLat * EARTH_RADIUS_M;
  double east = dLon * cos(meanLat) * EARTH_RADIUS_M;
  distanceM = (float)sqrt(north * north + east * east);
  bearingDeg = navWrap360((float)(atan2(east, north) / DEG_TO_RADIANS));
}

// Turn a heading error into left/right motor commands.
//   errorDeg   : bearing to target minus current heading, -180..+180.
//                Positive means the target is to starboard (turn right).
//   cruise     : forward power when pointing at the target.
//   pivotPower : power used to spin on the spot when badly off course.
//   pivotDeg   : error beyond which the boat spins instead of driving.
static inline void navSteer(float errorDeg, int cruise, int pivotPower, float pivotDeg,
                            int cmdMax, int &left, int &right) {
  if (fabsf(errorDeg) > pivotDeg) {
    int turn = (errorDeg > 0) ? pivotPower : -pivotPower;
    left = turn;
    right = -turn;
    return;
  }
  // Proportional: at the pivot limit the inner motor is stopped.
  float steer = errorDeg / pivotDeg * (float)cruise;
  float l = (float)cruise + steer;
  float r = (float)cruise - steer;
  float peak = fmaxf(fabsf(l), fabsf(r));
  if (peak > (float)cmdMax) {
    l *= (float)cmdMax / peak;
    r *= (float)cmdMax / peak;
  }
  left = (int)lroundf(l);
  right = (int)lroundf(r);
}
