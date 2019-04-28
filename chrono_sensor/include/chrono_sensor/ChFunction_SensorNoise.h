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
#include <vector>

#include "ChFunction_Sensor.h"

#include "chrono/core/ChVectorDynamic.h"
#include "chrono/core/ChVector.h"
#include <chrono/core/ChQuaternion.h>

namespace chrono {
namespace vehicle {
namespace sensor {
/**
 * Sensor Noise
 * @tparam T currently supports double, ChVector, ChQuaternions
 */
template<class T = double>
class ChApi ChFunction_SensorNoise : public ChFunction_Sensor<T> {
 public:
  ChFunction_SensorNoise() : m_mean(0.), m_stddev(0.) {
    m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
  };

  ChFunction_SensorNoise(const double &Mean,
                         const double &Stddev)
      : m_mean(1), m_stddev(1) {
    *m_mean[0] = Mean;
    *m_stddev[0] = Stddev;
    m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
  };

  ChFunction_SensorNoise(const ChVector<> &Mean,
                         const ChVector<> &Stddev)
      : m_mean(3), m_stddev(3) {
    m_mean.PasteVector(Mean, 0, 0);
    m_stddev.PasteVector(Stddev, 0, 0);
    m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
  };

  ChFunction_SensorNoise(const ChVectorDynamic<> &Mean,
                         const ChVectorDynamic<> &Stddev)
      : m_mean(Mean), m_stddev(Stddev) {
    m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
  };

  ChFunction_SensorNoise(const ChFunction_Sensor<T> &other)
      : m_mean(other.m_mean), m_stddev(other.m_stddev), m_gen(other.m_gen) {};

  ChFunction_SensorNoise<T> *Clone() const override {
    return new ChFunction_SensorNoise<T>(*this);
  };

  bool operator==(const ChFunction_SensorNoise &rhs) const {
    return m_gen == rhs.m_gen &&
        static_cast<ChVectorDynamic<>>(m_mean) == static_cast<ChVectorDynamic<>>(rhs.m_mean) &&
        static_cast<ChVectorDynamic<>>(m_stddev) == static_cast<ChVectorDynamic<>>(rhs.m_stddev);
  }

  bool operator!=(const ChFunction_SensorNoise &rhs) const {
    return !(rhs == *this);
  }

  FunctionType Get_Type() const override {
    return FUNCT_NOISE;
  }

  T Get_y(const T &x) const override {
    assert(m_mean.GetLength() == m_stddev.GetLength());
    assert(m_mean.GetLength() == 1);
    return x + Get_Noise_Scalar(*m_mean[0], *m_stddev[0]);
  };

  ChVectorDynamic<> Get_Mean() const {
    return m_mean;
  }

  void Set_Mean(ChVectorDynamic<> Mean) {
    m_mean = Mean;
  }

  void Set_Mean(double Mean) {
    *m_mean[0] = Mean;
  }

  ChVectorDynamic<> Get_Stddev() const {
    return m_stddev;
  }

  void Set_Stddev(ChVectorDynamic<> Stddev) {
    m_stddev = Stddev;
  }

  void Set_Stddev(double Stddev) {
    *m_stddev[0] = Stddev;
  }
 protected:
  static unsigned int Get_Seed() {
    typedef std::chrono::high_resolution_clock seed_clock;
    seed_clock::time_point beginning = seed_clock::now();
    seed_clock::duration d = seed_clock::now() - beginning;
    unsigned seed = d.count();
    return seed;
  };

  double Get_Noise_Scalar(const double mean, const double stddev) const {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(*m_gen);
  };

  ChVectorDynamic<> Get_Noise(const ChVectorDynamic<> mean, const ChVectorDynamic<> stddev) const {
    assert(mean.GetLength() == stddev.GetLength());
    ChVectorDynamic<> vect(mean.GetLength());
    for (int i = 0; i < mean.GetLength(); ++i) {
      *vect[i] = Get_Noise_Scalar(*mean[i], *stddev[i]);
    }
    return vect;
  };

  ChVectorDynamic<> m_mean;
  ChVectorDynamic<> m_stddev;
  std::shared_ptr<std::default_random_engine> m_gen;
};

template<>
ChVector<> ChFunction_SensorNoise<ChVector<>>::Get_y(const ChVector<> &x) const {
  ChVector<> noise_vec = Get_Noise(m_mean, m_stddev).ClipVector(0, 0);
  return x + noise_vec;
}

template<>
ChQuaternion<> ChFunction_SensorNoise<ChQuaternion<>>::Get_y(const chrono::ChQuaternion<> &x) const {
  ChQuaternion<> noise_vec = Get_Noise(m_mean, m_stddev).ClipQuaternion(0, 0).GetNormalized();
  ChQuaternion<> y = noise_vec * x;
  y.Normalize();
  return y;
}
} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHFUNCTION_SENSORNOISE_H
