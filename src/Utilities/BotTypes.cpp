#include <Arduino.h>
#include <string>

#include "BotTypes.h"

BotTypes::BotTypes() {

}

void BotTypes::readBotInfo() {
    
}

const char* BotTypes::botName2_str(uint8_t n_idx) {
    switch(n_idx) {
        case 0:  return "i++";
        case 1:  return "sqrt(-1)";
        case 2:  return "pi";
        case 3:  return "rho";
        case 4:  return "2.72";
        case 5:  return ":)";
        case 6:  return ">=";
        case 7:  return "32.2";
        case 8:  return "9.8";
        case 9:  return "c";
        case 10: return "phi";
        case 11: return "inf";
        case 12: return "theta";
        default: return "Robot"; 
    }
}

// string BotTypes::botType2String(uint8_t t_idx) {
//   switch (t_idx) {
//     case 0: return "lineman";
//     case 1: return "receiver";
//     case 2: return "runningback";
//     case 3: return "center";
//     case 4: return "mecanum_center";
//     case 5: return "quarterback";
//     case 6: return "kicker";
//     default: return "incorrect Bot Type declared";
//   }
// }