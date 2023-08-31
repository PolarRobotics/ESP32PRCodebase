#ifndef PAIRING_H_
#define PAIRING_H_
#include "PolarRobotics.h"
#define DEFAULT_BT_DISCOVER_TIME 10000
#define DEFAULT_REPAIRING_TIME 1000000
void activatePairing(bool doRePair = true, int discoverTime = DEFAULT_BT_DISCOVER_TIME);
#endif