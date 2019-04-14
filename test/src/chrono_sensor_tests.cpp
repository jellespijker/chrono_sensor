//
// Created by Konstantin Gredeskoul on 5/16/17.
//

#include <gtest/gtest.h>
#include "chrono_sensor/ChSensor.h"

using namespace chrono;
using namespace chrono::sensor;

TEST(sensor, test1) {
  auto test = ChSensor();
  ASSERT_EQ(1, 1);
}