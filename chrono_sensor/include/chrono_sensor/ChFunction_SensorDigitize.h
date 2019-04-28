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

#include <algorithm>
#include <type_traits>
#include "ChFunction_Sensor.h"

#include "chrono/core/ChVectorDynamic.h"

namespace chrono {
namespace vehicle {
namespace sensor {
template<typename T = double>
using opt_vect_t = typename std::conditional<std::is_same<T, ChVectorDynamic<>>::value, ChVectorDynamic<>, T>::type;

template<typename T = double>
class ChApi ChFunction_SensorDigitize : public ChFunction_Sensor<T> {
 public:
  ChFunction_SensorDigitize<T>() : m_range(0.), m_bits(0.), m_res(0.) {};

  ChFunction_SensorDigitize<T>(const double &bits, const opt_vect_t<T> &range) : m_range(range) {
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

  constexpr T Get_y(const T &x) const override {
    return m_res * Round(x / m_res);
  }

  opt_vect_t<T> &Get_Range() const {
    return m_range;
  }

  void Set_Range(const opt_vect_t<T> &Range) {
    m_range = Range;
    Set_Resolution(m_range, m_bits);
  }

  double Get_Bits() const {
    return m_bits;
  }

  void Set_Bits(const double Bits) {
    m_bits = Bits;
    Set_Resolution(m_range, m_bits);
  }

 protected:
  constexpr void Set_Resolution(const opt_vect_t<T> &range, const double bits) {
    m_res = range / pow(2., bits);
  }

  constexpr opt_vect_t<T> Round(const T &x) const {
    opt_vect_t<T> ret(x);
    if constexpr(std::is_scalar<T>::value) {
      ret = round(x);
    } else {
      if constexpr(std::is_same<T, ChVector<>>::value) {
        ret.Set(round(x.x()), round(x.y()), round(x.z()));
      } else {
        std::for_each(ret.GetAddress()[0], ret.GetAddress()[x.GetLength()], round);
      }
    }
    return ret;
  };

  opt_vect_t<T> m_range;
  opt_vect_t<T> m_res;
  double m_bits;
};

} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHFUNCTION_SENSORDIGITIZE_H
