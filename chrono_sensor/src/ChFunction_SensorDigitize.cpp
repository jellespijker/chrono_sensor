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

#include "chrono_sensor/ChFunction_SensorDigitize.h"

namespace chrono {

ChFunction_SensorDigitize::ChFunction_SensorDigitize(u_int8_t bits, double min, double max) :
    m_bits(bits),
    m_min(min),
    m_max(max) {
  m_res = (max - min) / pow(2, bits);
}

ChFunction_SensorDigitize::ChFunction_SensorDigitize(const ChFunction_SensorDigitize &other) :
    m_bits(other.m_bits),
    m_res(other.m_res),
    m_min(other.m_min),
    m_max(other.m_max) {
}

ChFunction_SensorDigitize *ChFunction_SensorDigitize::Clone() const {
  return new ChFunction_SensorDigitize(*this);
}

bool ChFunction_SensorDigitize::operator==(const ChFunction_SensorDigitize &rhs) const {
  return m_bits == rhs.m_bits &&
      m_res == rhs.m_res &&
      m_min == rhs.m_min &&
      m_max == rhs.m_max;
}

bool ChFunction_SensorDigitize::operator!=(const ChFunction_SensorDigitize &rhs) const {
  return !(*this == rhs);
}

double ChFunction_SensorDigitize::Get_y(double x) const {
  double val = m_res * round(x / m_res);
  if (val < m_min) {
    return m_min;
  } else if (val > m_max) {
    return m_max;
  }
  return val;
}

ChVector<> ChFunction_SensorDigitize::Get_y(ChVector<> &vec) const {
  return ChVector<>(Get_y(vec.x()), Get_y(vec.y()), Get_y(vec.z()));
}
} /// chrono
