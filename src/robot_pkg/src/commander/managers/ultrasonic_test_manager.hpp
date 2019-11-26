#ifndef __ULTRASONIC_TEST_MANAGER__
#define __ULTRASONIC_TEST_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class UltrasonicTestManager : public StateMachine {
  bool start();

  private:
    bool printUltrasonics();

  public:
    UltrasonicTestManager();
    virtual string getName();
};

#endif
