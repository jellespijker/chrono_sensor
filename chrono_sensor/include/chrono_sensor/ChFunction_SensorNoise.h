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

#ifndef CHRONO_SENSOR_CHFUNCTION_SENSORNOISE_H
#define CHRONO_SENSOR_CHFUNCTION_SENSORNOISE_H

#include <random>
#include <chrono>

#include "ChFunction_Sensor.h"

#include "chrono/core/ChVectorDynamic.h"
#include "chrono/core/ChVector.h"
#include <chrono/core/ChQuaternion.h>

namespace chrono {
namespace vehicle {
namespace sensor {

template<typename T = double>
class ChApi ChFunction_SensorNoise : public ChFunction_Sensor<T> {
 public:
  ChFunction_SensorNoise() : m_mean(0.), m_stddev(0.) {
    static_assert(
        std::is_same<T, double>::value || std::is_same<T, ChVector<>>::value || std::is_same<T, ChQuaternion<>>::value,
        "ChFunction_SensorNoise requires a double, chrono::ChVector<double> of ChQuaternion<double> type");
    m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
  };

  ChFunction_SensorNoise(const T &Mean,
                         const T &Stddev)
      : m_mean(1), m_stddev(1) {
    m_mean = Mean;
    m_stddev = Stddev;
    m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
  };

  ChFunction_SensorNoise(const ChFunction_Sensor<T> &other)
      : m_mean(other.m_mean), m_stddev(other.m_stddev), m_gen(other.m_gen) {};

  ChFunction_SensorNoise<T> *Clone() const override {
    return new ChFunction_SensorNoise<T>(*this);
  };

  bool operator==(const ChFunction_SensorNoise &rhs) const {
    return m_gen == rhs.m_gen &&
        static_cast<T>(m_mean) == static_cast<T>(rhs.m_mean) &&
        static_cast<T>(m_stddev) == static_cast<T>(rhs.m_stddev);
  }

  bool operator!=(const ChFunction_SensorNoise &rhs) const {
    return !(rhs == *this);
  }

  FunctionType Get_Type() const override {
    return FUNCT_NOISE;
  }

  T Get_y(const T &x) const override {
    if constexpr(std::is_same<T, ChQuaternion<>>::value) {
      return x * Get_Noise(m_mean, m_stddev);
    } else {
      return x + Get_Noise(m_mean, m_stddev);
    }
  };

  T &Get_Mean() const {
    return m_mean;
  }

  void Set_Mean(const T &Mean) {
    m_mean = Mean;
  }

  T &Get_Stddev() const {
    return m_stddev;
  }

  void Set_Stddev(const T &Stddev) {
    m_stddev = Stddev;
  }

 protected:
  static unsigned int Get_Seed() {
    typedef std::chrono::high_resolution_clock seed_clock;
    seed_clock::time_point beginning = seed_clock::now();
    seed_clock::duration d = seed_clock::now() - beginning;
    unsigned seed = d.count();
    return seed;
  };

  T Get_Noise(const T &mean, const T &stddev) const {
    if constexpr(std::is_same<T, double>::value) {
      return Get_Noise_Scalar(mean, stddev);
    } else {
      size_t last_elem;
      if constexpr(std::is_same<T, ChVector<>>::value) {
        last_elem = 3;
      } else {
        last_elem = 4;
      }
      T ret;
      for (int i = 0; i < last_elem; ++i) {
        ret[i] = Get_Noise_Scalar(mean[i], stddev[i]);
      }
      return ret;
    }
  };

  double Get_Noise_Scalar(const double &mean, const double &stddev) const {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(*m_gen);
  }

  T m_mean;
  T m_stddev;
  std::shared_ptr<std::default_random_engine> m_gen;
};
} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHFUNCTION_SENSORNOISE_H
