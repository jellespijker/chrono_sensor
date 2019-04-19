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

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

class ChApi ChFunction_SensorBias : public ChFunction {
 public:
  ChFunction_SensorBias(double bias = 0.);
  ChFunction_SensorBias(ChVector<> bias);
  ChFunction_SensorBias(const ChFunction_SensorBias &other);

  ChFunction_SensorBias *Clone() const override;
  bool operator==(const ChFunction_SensorBias &rhs) const;
  bool operator!=(const ChFunction_SensorBias &rhs) const;
  double Get_y(double x) const override;
  virtual ChVector<> Get_y(ChVector<> &vec) const;
  //Todo: Create 1ste and 2nd derivatives

 private:
  double m_bias;
  ChVector<> m_bias_v;
};
} /// chrono

#endif //CHRONO_SENSOR_CHFUNCTION_SENSORBIAS_H
