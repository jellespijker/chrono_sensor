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

#ifndef CHRONO_SENSOR_CHFUNCTION_SENSOR_H
#define CHRONO_SENSOR_CHFUNCTION_SENSOR_H

#include <typeinfo>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChClassFactory.h"

namespace chrono {
namespace vehicle {
namespace sensor {

enum FunctionType {
  FUNCT_CUSTOM,
  FUNCT_NOISE,
  FUNCT_BIAS,
  FUNCT_DIGITIZE
};

template<typename T = double>
class ChApi ChFunction_Sensor {
 public:
  ChFunction_Sensor() : M_BSL(1., BDF_STEP_LOW) { M_BSL.Normalize(); };
  ChFunction_Sensor(const ChFunction_Sensor &other) : M_BSL(other.M_BSL) {};
  virtual ~ChFunction_Sensor() = default;

  /// "Virtual" copy constructor.
  virtual ChFunction_Sensor *Clone() const = 0;

  /// Return the unique function type identifier.
  virtual FunctionType Get_Type() const { return FUNCT_CUSTOM; }

  // THE MOST IMPORTANT MEMBER FUNCTIONS
  // At least Get_y() should be overridden by derived classes.

  /// Return the y value of the function, at position x.
  virtual T Get_y(const T &x) const = 0;

  /// Return the dy/dx derivative of the function, at position x.
  /// Note that inherited classes may also avoid overriding this method,
  /// because this base method already provide a general-purpose numerical differentiation
  /// to get dy/dx only from the Get_y() function. (however, if the analytical derivative
  /// is known, it may better to implement a custom method).
  virtual T Get_y_dx(const T &x) const {
    if constexpr(std::is_same<T, ChQuaternion<>>::value) {
      ChQuaternion<> dy = Get_y(x * M_BSL) - Get_y(x);
      dy /= BDF_STEP_LOW;
      dy.Normalize();
      return dy;
    } else {
      return ((Get_y(x + BDF_STEP_LOW) - Get_y(x)) / BDF_STEP_LOW);
    }
  }

  /// Return the ddy/dxdx double derivative of the function, at position x.
  /// Note that inherited classes may also avoid overriding this method,
  /// because this base method already provide a general-purpose numerical differentiation
  /// to get ddy/dxdx only from the Get_y() function. (however, if the analytical derivative
  /// is known, it may be better to implement a custom method).
  virtual T Get_y_dxdx(const T &x) const {
    if constexpr(std::is_same<T, ChQuaternion<>>::value) {
      ChQuaternion<> dy = Get_y_dx(x * M_BSL) - Get_y_dx(x);
      dy /= BDF_STEP_LOW;
      dy.Normalize();
      return dy;
    } else {
      return ((Get_y_dx(x + BDF_STEP_LOW) - Get_y_dx(x)) / BDF_STEP_LOW);
    }
  };

  /// Return the weight of the function (useful for
  /// applications where you need to mix different weighted ChFunctions)
  virtual double Get_weight(T x) const { return 1.0; };

  /// Return the function derivative of specified order at the given point.
  /// Note that only order = 0, 1, or 2 is supported.
  virtual T Get_y_dN(T x, int derivate) const {
    switch (derivate) {
      case 0:return Get_y(x);
      case 1:return Get_y_dx(x);
      case 2:return Get_y_dxdx(x);
      default:return Get_y(x);
    }
  }

  /// Update could be implemented by children classes, ex. to launch callbacks
  virtual void Update(const double x) {}

  /// Method to allow serialization of transient data to archives
  virtual void ArchiveOUT(ChArchiveOut &marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Sensor<T>>();
  }

  /// Method to allow de-serialization of transient data from archives.
  virtual void ArchiveIN(ChArchiveIn &marchive) {
    // version number
    int version = marchive.VersionRead<ChFunction_Sensor<T>>();
  }

 private:
  ChQuaternion<> M_BSL;
};

} /// sensor
} /// vehicle
} /// chrono
#endif //CHRONO_SENSOR_CHFUNCTION_SENSOR_H
