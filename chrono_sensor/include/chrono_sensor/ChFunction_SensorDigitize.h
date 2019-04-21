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

#ifndef CHRONO_SENSOR_CHFUNCTION_SENSORDIGITIZE_H
#define CHRONO_SENSOR_CHFUNCTION_SENSORDIGITIZE_H

#include "ChFunction_Sensor.h"

namespace chrono {
namespace vehicle {
namespace sensor {

template<class T = double>
class ChApi ChFunction_SensorDigitize : public ChFunction_Sensor<T> {
 public:
  ChFunction_SensorDigitize<T>(const T &bits, const T &range) : m_range(range) {
    Set_Bits(bits);
  }

  ChFunction_SensorDigitize<T>(const ChFunction_SensorDigitize<T> &other)
      : m_range(other.m_range), m_res(other.m_res), m_bits(other.m_bits) {}

  ChFunction_SensorDigitize<T> *Clone() const override {
    return new ChFunction_SensorDigitize<T>(*this);
  }

  FunctionType Get_Type() const override {
    return FUNCT_DIGITIZE;
  }

  T Get_y(const T &x) const override {
    return m_res * Round(x / m_res);
  }

  T Get_Range() const {
    return m_range;
  }

  void Set_Range(T Range) {
    m_range = Range;
    m_res = m_range / Pow(2., m_bits);
  }

  T Get_Bits() const {
    return m_bits;
  }

  void Set_Bits(T Bits) {
    m_bits = Bits;
    m_res = m_range / Pow(2., m_bits);
  }

 protected:
  T Round(const T &x) const { return round(x); };
  T Pow(const double &base, const T &x) const { return pow(base, x); }
  T m_range;
  T m_res;
  T m_bits;
};

template<>
ChVector<> ChFunction_SensorDigitize<ChVector<>>::Round(const ChVector<> &x) const {
  ChVector<> vec;
  for (int i = 0; i < 3; ++i) {
    vec[i] = round(x[i]);
  }
  return vec;
}

template<>
ChVector<> ChFunction_SensorDigitize<ChVector<>>::Pow(const double &base, const ChVector<> &x) const {
  ChVector<> vec;
  for (int i = 0; i < 3; ++i) {
    vec[i] = pow(base, x[i]);
  }
  return vec;
}
} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHFUNCTION_SENSORDIGITIZE_H
