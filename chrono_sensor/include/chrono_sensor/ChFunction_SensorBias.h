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

#ifndef CHRONO_SENSOR_CHFUNCTION_SENSORBIAS_H
#define CHRONO_SENSOR_CHFUNCTION_SENSORBIAS_H

#include "ChFunction_Sensor.h"

namespace chrono {
namespace vehicle {
namespace sensor {
template<class T = double>
class ChApi ChFunction_SensorBias : public ChFunction_Sensor<T> {
 public:
  explicit ChFunction_SensorBias<T>(const T &bias) : m_bias(bias) {};
  ChFunction_SensorBias<T>(const ChFunction_SensorBias<T> &other) : m_bias(other.m_bias) {}

  ChFunction_SensorBias<T> *Clone() const override {
    return new ChFunction_SensorBias<T>(*this);
  }

  FunctionType Get_Type() const override {
    return FUNCT_BIAS;
  }

  bool operator==(const ChFunction_SensorBias &rhs) const {
    return m_bias == rhs.m_bias;
  }

  bool operator!=(const ChFunction_SensorBias &rhs) const {
    return !(rhs == *this);
  }

  T Get_y(const T &x) const override {
    return x + m_bias;
  }

  T Get_Bias() const {
    return m_bias;
  }

  void Get_Bias(T Bias) {
    m_bias = Bias;
  }

 protected:
  T m_bias;
};

template<>
ChQuaternion<> ChFunction_SensorBias<ChQuaternion<>>::Get_y(const ChQuaternion<> &x) const {
  ChQuaternion<> y = x * m_bias;
  y.Normalize();
  return y;
}
} /// sensor
} /// vehicle
} /// chrono

#endif //CHRONO_SENSOR_CHFUNCTION_SENSORBIAS_H
