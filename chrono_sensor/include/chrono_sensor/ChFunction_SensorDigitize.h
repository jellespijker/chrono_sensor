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

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

class ChApi ChFunction_SensorDigitize : public ChFunction {
 public:
  ChFunction_SensorDigitize(u_int8_t bits = 12, double min = 0., double max = 1.);
  ChFunction_SensorDigitize(const ChFunction_SensorDigitize &other);

  ChFunction_SensorDigitize *Clone() const override;
  bool operator==(const ChFunction_SensorDigitize &rhs) const;
  bool operator!=(const ChFunction_SensorDigitize &rhs) const;

  double Get_y(double x) const override;
  virtual ChVector<> Get_y(ChVector<> &vec) const;
  //Todo: Create 1ste and 2nd derivatives

 private:
  double m_min;
  double m_max;
  double m_res;
  double m_bits;
};

} /// chrono


#endif //CHRONO_SENSOR_CHFUNCTION_SENSORDIGITIZE_H
