#ifndef PAIRING_H
#define PAIRING_H

#include "PolarRobotics.h"
#define DEFAULT_BT_DISCOVER_TIME 10000
#define DEFAULT_REPAIRING_TIME 1000000
bool addressIsController(const char * addrCharPtr);
bool startDiscovery();
void storeAddress(const char *addr, bool clear);
void getAddress(const char *&addr);
void activatePairing(bool doRePair = true, int discoverTime = DEFAULT_BT_DISCOVER_TIME);

#endif // PAIRING_H