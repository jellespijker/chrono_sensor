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

#include "chrono_sensor/ChFunction_SensorNoise.h"

#include <chrono>

namespace chrono {

ChFunction_SensorNoise::ChFunction_SensorNoise(double mean, double stddev) :
    m_mean(mean),
    m_stddev(stddev),
    m_dist(std::make_shared<std::normal_distribution<double>>(mean, stddev)) {
  m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
}

ChFunction_SensorNoise::ChFunction_SensorNoise(const ChFunction_SensorNoise &other) :
    m_mean(other.m_mean),
    m_stddev(other.m_stddev),
    m_dist(std::make_shared<std::normal_distribution<double>>(other.m_mean, other.m_stddev)) {
  m_gen = std::make_shared<std::default_random_engine>(Get_Seed());
}

unsigned int ChFunction_SensorNoise::Get_Seed() const {
  typedef std::chrono::high_resolution_clock myclock;
  myclock::time_point beginning = myclock::now();
  myclock::duration d = myclock::now() - beginning;
  unsigned seed = d.count();
  return seed;
}

ChFunction_SensorNoise *ChFunction_SensorNoise::Clone() const {
  return new ChFunction_SensorNoise(*this);
}

double ChFunction_SensorNoise::Get_y(double x) const {
  return x + m_dist->operator()(*m_gen);
}

ChVector<> ChFunction_SensorNoise::Get_y(ChVector<> &vec) const {
  double x = Get_y(vec.x());
  double y = Get_y(vec.y());
  double z = Get_y(vec.z());
  return ChVector<>(x, y, z);
}
bool ChFunction_SensorNoise::operator==(const ChFunction_SensorNoise &rhs) const {
  return m_mean == rhs.m_mean &&
      m_stddev == rhs.m_stddev;
}
bool ChFunction_SensorNoise::operator!=(const ChFunction_SensorNoise &rhs) const {
  return !(rhs == *this);
}

} /// chrono