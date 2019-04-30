// MIT License
//
// Copyright (c) 2019 Jelle Spijker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <gtest/gtest.h>

#ifndef STAT_TEST // Don't perform statistical tests if boost is not found
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#endif

#include "chrono/motion_functions/ChFunction_Ramp.h"

#include "chrono_sensor/ChFunction_SensorNoise.h"
#include "chrono_sensor/ChFunction_SensorBias.h"
#include "chrono_sensor/ChFunction_SensorDigitize.h"

using namespace chrono;
using namespace chrono::vehicle::sensor;

#ifndef STAT_TEST // Don't perform statistical tests if boost is not found
using namespace boost::accumulators;

TEST(Function_Noise, stddev_mean_single_value) {
  ChFunction_Ramp f_ramp;
  f_ramp.Set_ang(0.1);
  f_ramp.Set_y0(0.4);

  double mean_val(0.5);
  double stddev_val(0.2);

  ChFunction_SensorNoise<> f_noise(mean_val, stddev_val);

  accumulator_set<double, stats<tag::mean, tag::variance>> acc;

  for (int i = 0; i < 1000; i++) {
    double x = (double) i / 50.0;
    double y = f_ramp.Get_y(x);
    double y_noise = f_noise.Get_y(y);
    double noise = y_noise - y;
    acc(noise);
  }
  auto m = mean(acc);
  auto st = sqrt(variance(acc));
  ASSERT_NEAR(mean(acc), mean_val, mean_val / 30.);
  ASSERT_NEAR(sqrt(variance(acc)), stddev_val, stddev_val / 30.);
}

TEST(Function_Noise, stddev_mean_vector) {
  ChFunction_Ramp f_ramp;
  f_ramp.Set_ang(0.1);
  f_ramp.Set_y0(0.4);

  ChVector<> mean_val(0.5, 0.25, 0.);
  ChVector<> stddev_val(0.2, 0.4, 0.);

  ChFunction_SensorNoise<ChVector<>> f_noise(mean_val, stddev_val);

  accumulator_set<double, stats<tag::mean, tag::variance>> acc_x;
  accumulator_set<double, stats<tag::mean, tag::variance>> acc_y;
  accumulator_set<double, stats<tag::mean, tag::variance>> acc_z;

  for (int i = 0; i < 10000; i++) {
    double id = static_cast<double>(i) / 50.;
    ChVector<> x = ChVector<>(id);
    ChVector<> y = ChVector<>(f_ramp.Get_y(x.x()), f_ramp.Get_y(x.y()), f_ramp.Get_y(x.z()));
    ChVector<> y_noise = f_noise.Get_y(y);
    ChVector<> noise = y_noise - y;
    acc_x(noise.x());
    acc_y(noise.y());
    acc_z(noise.z());
  }

  ASSERT_NEAR(mean(acc_x), mean_val.x(), mean_val.x() / 30.);
  ASSERT_NEAR(sqrt(variance(acc_x)), stddev_val.x(), stddev_val.x() / 30.);
  ASSERT_NEAR(mean(acc_y), mean_val.y(), mean_val.y() / 30.);
  ASSERT_NEAR(sqrt(variance(acc_y)), stddev_val.y(), stddev_val.y() / 30.);
  ASSERT_NEAR(mean(acc_z), mean_val.z(), mean_val.z() / 30.);
  ASSERT_NEAR(sqrt(variance(acc_z)), stddev_val.z(), stddev_val.z() / 30.);
}
#endif

TEST(Function_Noise, clone) {
  double mean_val = 0.5;
  double stddev_val = 0.2;

  ChFunction_SensorNoise<> f_noise(mean_val, stddev_val);
  auto f_noise_clone = f_noise.Clone();
  ASSERT_EQ(f_noise, *f_noise_clone);
  delete f_noise_clone;
}

TEST(Function_Bias, single_value) {
  double bias = 5.;
  double x = 1.;
  ChFunction_SensorBias<> f_bias(bias);
  ASSERT_EQ(x + bias, f_bias.Get_y(x));
}

TEST(Function_Bias, vector) {
  ChVector<> bias(5., 4., 3.);
  ChVector<> x(2., 4., 8.);
  ChFunction_SensorBias<ChVector<>> f_bias(bias);
  ASSERT_EQ(x + bias, f_bias.Get_y(x));
}

TEST(Function_Bias, quaternion) {
  ChQuaternion<> initial_rot = Q_from_AngAxis(20 * CH_C_DEG_TO_RAD, ChVector<>(0., 0., 1.));
  ChQuaternion<> bias = Q_from_AngAxis(25 * CH_C_DEG_TO_RAD, ChVector<>(0., 0., 1.));
  ChFunction_SensorBias<ChQuaternion<>> f_bias(bias);
  auto ret = f_bias.Get_y(initial_rot);
  ChVector<> va(5., 0., 0.);
  ChVector<> vb = ret.Rotate(va);
  ASSERT_NEAR(vb.x(), 3.53553390593, 1e-6);
  ASSERT_NEAR(vb.y(), 3.53553390593, 1e-6);
}

TEST(Function_Bias, clone) {
  double bias = 5.;
  ChFunction_SensorBias<> f_bias(bias);

  auto f_bias_clone = f_bias.Clone();
  ASSERT_EQ(f_bias, *f_bias_clone);
  delete f_bias_clone;
}

TEST(Function_Digitize, single_value) {
  double bits = 4;
  double min = 0.0;
  double max = 50.;
  double res = (max - min) / pow(2, bits);
  ChFunction_SensorDigitize<> f_dig(bits, max - min);
  auto test = f_dig.Get_y(3.45);
  ASSERT_EQ(f_dig.Get_y(3.45), res * round(3.45 / res));
}

TEST(Function_Digitize, vector) {
  double bits(12.);
  ChVector<> min(0.0);
  ChVector<> max(50.);
  ChVector<> res;
  ChFunction_SensorDigitize<ChVector<>> f_dig(bits, max - min);
  for (int i = 0; i < 3; ++i) {
    res[i] = (max[i] - min[i]) / pow(2., bits);
    ASSERT_EQ(f_dig.Get_y(ChVector<>(3.45))[i], res[i] * round(3.45 / res[i]));
  }
}

TEST(Function_Digitize, quaternion) {
  double bits(12.);
  ChVector<> min(0.);
  ChVector<> max(50.);
  ChQuaternion<> res;
  ChFunction_SensorDigitize<ChQuaternion<>> f_dig(bits, max - min);
  auto t = f_dig.Get_y(ChQuaternion<>(3.45, ChVector<>(3.45)));
  auto r = ChQuaternion<>(0.49950151852068797, ChVector<>(0.50016605009254234));
  ASSERT_TRUE(t.Equals(r));
}
