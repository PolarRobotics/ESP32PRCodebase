#include <Robot/Robot.h>

/**
 * @brief Lineman Subclass Header
 * @authors Max Phillips
 */

class Lineman : public Robot {
  // private: 
  //   int test;
  public:
    Lineman();
    // void initialize(); // Override virtual functions
    void action() override;
};

Lineman::Lineman() {} // empty constructor (must be defined)

void Lineman::action() {} // empty action function (must be defined)