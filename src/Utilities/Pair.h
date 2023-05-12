#ifndef PAIR_H
#define PAIR_H

/**
 * @brief Pair struct used to create simple mappings, for instance of enum values to strings.
 * @author Max Phillips
 */

template<typename K, typename V>
struct Pair {
  K key;
  V value;
};

/* Sample Usage

//* Define Number of Elements in Array
#define SIZE 10

//* Define core enum
typedef enum {
  RED,
  GREEN,
  BLUE
} COLOR;

//* Define compile time mapping of enum values to strings (const char*)
constexpr Pair<COLOR, const char*> colors[SIZE] = {
  {RED, "red"},
  {GREEN, "green"},
  {BLUE, "blue"}
};

//* Define string conversion function
const char* getColorName(COLOR key) {
  return colors[static_cast<int>(key)].value;
}

*/

#endif // PAIR_H