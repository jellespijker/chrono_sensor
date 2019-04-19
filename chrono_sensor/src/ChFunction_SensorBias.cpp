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

#include "chrono_sensor/ChFunction_SensorBias.h"

namespace chrono {

ChFunction_SensorBias::ChFunction_SensorBias(double bias) : m_bias(bias) {}

ChFunction_SensorBias::ChFunction_SensorBias(const ChFunction_SensorBias &other)
    : m_bias(other.m_bias), m_bias_v(other.m_bias_v) {}

ChFunction_SensorBias::ChFunction_SensorBias(ChVector<> bias) : m_bias_v(bias) {

}

ChFunction_SensorBias *ChFunction_SensorBias::Clone() const {
  return new ChFunction_SensorBias(*this);
}

double ChFunction_SensorBias::Get_y(double x) const {
  return x + m_bias;
}

ChVector<> ChFunction_SensorBias::Get_y(ChVector<> &vec) const {
  return vec + m_bias_v;
}

bool ChFunction_SensorBias::operator==(const ChFunction_SensorBias &rhs) const {
  return m_bias == rhs.m_bias &&
      m_bias_v == rhs.m_bias_v;
}

bool ChFunction_SensorBias::operator!=(const ChFunction_SensorBias &rhs) const {
  return !(rhs == *this);
}

} /// chrono
