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

#include "chrono_sensor/Accelerometer.h"

namespace chrono {
namespace vehicle {
namespace sensor {

Accelerometer::Accelerometer(ChVehicle &vehicle, const double sample_rate, const double delay) : ChSensor<ChVector<>>(
    vehicle,
    sample_rate,
    delay) {
  auto noise = std::make_shared<ChFunction_SensorNoise<ChVector<>>>();
  auto digitize = std::make_shared<ChFunction_SensorDigitize<ChVector<>>>();
  m_transform.push_back(noise);
  m_transform.push_back(digitize);
}

void Accelerometer::Initialize(const double &bits,
                               const ChVector<> &range,
                               const ChVector<> &mean,
                               const ChVector<> &stddev) {
  Get_DigitalTransform()->Set_Bits(bits);
  Get_DigitalTransform()->Set_Range(range);
  Get_NoiseTransform()->Set_Mean(mean);
  Get_NoiseTransform()->Set_Stddev(stddev);
  ChSensor::Initialize();
}

std::shared_ptr<ChFunction_SensorDigitize<ChVector<>>> Accelerometer::Get_DigitalTransform() {
  return std::dynamic_pointer_cast<ChFunction_SensorDigitize<ChVector<>>>(m_transform[1]);
}

std::shared_ptr<ChFunction_SensorNoise<ChVector<>>> Accelerometer::Get_NoiseTransform() {
  return std::dynamic_pointer_cast<ChFunction_SensorNoise<ChVector<>>>(m_transform[0]);
}
} /// sensor
} /// vehicle
} /// chrono