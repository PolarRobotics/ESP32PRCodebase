#pragma once

#ifndef BUILTINLED_H_
#define BUILTINLED_H_
/**
 * @brief Header for Global ESP Built-In LED Handling
 * @authors Max Phillips
 * 
 * Provides a framework for accessing the LED built into
 * the ESP32 board. Is globally scoped to avoid multiple
 * drivers of the LED. Basically functions like a mutex.
 * 
 * This file declares prototypes of the functions 
 * implemented in `builtInLED.cpp`.
 * 
 **/

bool builtInLedOn();
void toggleBuiltInLED();
void setBuiltInLED(bool on = true);
#endif