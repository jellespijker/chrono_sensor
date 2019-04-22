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

#ifndef CHRONO_SENSOR_CHSENSOR_H
#define CHRONO_SENSOR_CHSENSOR_H

#include <vector>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_sensor/ChFunction_Sensor.h"

namespace chrono {
namespace vehicle {
namespace sensor {

/// Base class for a vehicle sensor system.
template<class T>
class CH_VEHICLE_API ChSensor {
 public:
  ChSensor() : m_sample_rate(0.), m_log_filename(""), m_prev_sample_time(0.), m_sample(true) {}

  ChSensor(ChVehicle &vehicle, double sample_rate = 0.)
      : m_vehicle(vehicle), m_sample_rate(sample_rate), m_log_filename(""), m_prev_sample_time(0.), m_sample(true) {};

  virtual ~ChSensor() = default;

  /// Initialize this Sensor System
  virtual void Initialize() {};

  ChVehicle &Get_Vehicle() const { return m_vehicle; }

  void Set_Vehicle(ChVehicle &Vehicle) { m_vehicle = &Vehicle; }

  double Get_SampleRate() const { return m_sample_rate; }

  void Set_SampleRate(double SampleRate) { m_sample_rate = SampleRate; }

  void Set_Input(T input) { m_input = input; };

  T &Get_Input() { return m_input; };

  void Set_Output(T output) { m_output = output; };

  T &Get_Output() { return m_output; };

  /// Update the state of this driver system at the current time.
  virtual void Synchronize(double time) {
    double dt = time - m_prev_sample_time;
    if (dt >= m_sample_rate) {
      m_sample = true;
      m_prev_sample_time = time;
    } else {
      m_sample = false;
    }
  };

  /// Advance the state of this driver system by the specified time step
  virtual void Advance(double step) {
    if (!m_sample) // Only perform measurement at specified sample rate
      return;

    m_output = m_input;
    for (auto transform : m_transform) {
      m_output = transform->Get_y(m_output);
    }
  }

  /// Initialize output file for recording sensor inputs.
  bool LogInit(const std::string &filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename.c_str(), std::ios::out);
    if (!ofile)
      return false;

    ofile << "Time, Input, Output" << std::endl;
    ofile.close();
    return true;
  };

  /// Record the current sensor inputs to the log file.
  bool Log(double time) {
    if (m_log_filename.empty())
      return false;

    std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
    if (!ofile)
      return false;

    ofile << time << ", " << *m_input << ", " << *m_output << std::endl;
    ofile.close();
    return true;
  }

 protected:
  ChVehicle &m_vehicle;
  double m_sample_rate;
  T m_input;
  T m_output;
  std::vector<std::shared_ptr<ChFunction_Sensor<T>>> m_transform;
  double m_prev_sample_time;
  bool m_sample;

 private:
  std::string m_log_filename;
};
} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHSENSOR_H
