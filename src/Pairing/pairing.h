#ifndef PAIRING_H
#define PAIRING_H

#include "PolarRobotics.h"
#define DEFAULT_BT_DISCOVER_TIME 10000 // milliseconds
#define DEFAULT_BT_REPAIR_TIME 1000000 // milliseconds
bool addressIsController(const char* addrCharPtr);
bool startDiscovery();
void storeAddress(const char* addr, bool clear);
void getAddress(const char* &addr);
void activatePairing(int discoverTime = DEFAULT_BT_DISCOVER_TIME, int rePairTime = DEFAULT_BT_REPAIR_TIME);

#endif // PAIRING_H