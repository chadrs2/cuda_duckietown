#define CATCH_CONFIG_MAIN

#include "htk.h"
#include "vendor/catch.hpp"

TEST_CASE("Can use basic functions", "[HTK]") {
  htkTime_start(GPU, "timer."); //@@ start a timer
  htkTime_stop(GPU, "timer."); //@@ stop the timer
}
