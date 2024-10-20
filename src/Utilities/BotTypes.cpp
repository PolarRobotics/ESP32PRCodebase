#include "BotTypes.h"

// constexpr to be evaluated at compile time
constexpr Pair<BotType, const char*> 
botTypeStrings[NUM_POSITIONS] = {
  { lineman,            "lineman"            },
  { receiver,           "receiver"           },
  { runningback,        "runningback"        },
  { center,             "center"             },
  { kicker,             "kicker"             },
  { mecanum_center,     "mecanum_center"     },
  { quarterback_old,    "quarterback_old"    },
  { quarterback_base,   "quarterback_base"   },
  { quarterback_turret, "quarterback_turret" }
};

// Function to map BotTypes to human-readable C-strings
const char* getBotTypeString(BotType type) {
  return botTypeStrings[static_cast<int>(type)].value;
}

/**
 * @brief getBotName uses the index of the bot, to index the bot configuration array, defined in BotTypes.h
 * if the bot index is out of range of the number of possible bots, returns "Robot"
 * 
 * @param n_idx the index of the bot in the array
 * @return const char* if the index is within range return the bot name
 */
const char* getBotName(uint8_t index) {
  return (index >= 0 && index < NUM_BOTS) ? botConfigArray[index].bot_name : "Robot"; 
}
