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

#include <array>

#include "ChFunction_Sensor.h"

namespace chrono {
namespace vehicle {
namespace sensor {
template<typename T = double>
using opt_vect_t = typename std::conditional<std::is_same<T, ChQuaternion<>>::value,
                                             ChVector<>, T>::type;

template<typename T = double>
class ChApi ChFunction_SensorDigitize : public ChFunction_Sensor<T> {
 public:
  ChFunction_SensorDigitize<T>() {
    static_assert(
        std::is_same<T, double>::value || std::is_same<T, ChVector<>>::value || std::is_same<T, ChQuaternion<>>::value,
        "ChFunction_SensorDigitize requires a double, chrono::ChVector<double> of ChQuaternion<double> type");
    m_range = T(0.);
    m_bits = 0.;
    m_res = T(0.);
  };

  ChFunction_SensorDigitize<T>(const double &bits, const opt_vect_t<T> &range) {
    static_assert(
        std::is_same<T, double>::value || std::is_same<T, ChVector<>>::value || std::is_same<T, ChQuaternion<>>::value,
        "ChFunction_SensorDigitize requires a double, chrono::ChVector<double> of ChQuaternion<double> type");
    m_range = range;
    m_bits = bits;
    m_res = Calc_Resolution(m_range, m_bits);
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
    if constexpr(std::is_same<T, ChQuaternion<>>::value) {
      auto x_p = ChVector<>(x.e1(), x.e2(), x.e3());
      auto x_d_vec = ChVector<>(m_res * Round(x_p / m_res));
      return ChQuaternion<>(x.e0(), x_d_vec).GetNormalized();
    } else {
      return m_res * Round(x / m_res);
    }
  }

  opt_vect_t<T> &Get_Range() const {
    return m_range;
  }

  void Set_Range(const opt_vect_t<T> &Range) {
    m_range = Range;
    m_res = Calc_Resolution(m_range, m_bits);
  }

  double Get_Bits() const {
    return m_bits;
  }

  void Set_Bits(const double Bits) {
    m_bits = Bits;
    m_res = Calc_Resolution(m_range, m_bits);
  }

 protected:
  constexpr opt_vect_t<T> Calc_Resolution(const opt_vect_t<T> &range, const double bits) {
    return range / pow(2., bits);
  }

  opt_vect_t<T> Round(const opt_vect_t<T> &x) const {
    if constexpr(std::is_same<T, double>::value) {
      return round(x);
    } else {
      opt_vect_t<T> ret;
      for (int i = 0; i < 3; ++i) {
        ret[i] = round(x[i]);
      }
      return ret;
    }
  };

  opt_vect_t<T> m_range;
  opt_vect_t<T> m_res;
  double m_bits;
};

} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHFUNCTION_SENSORDIGITIZE_H
