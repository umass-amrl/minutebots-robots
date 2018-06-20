// Copyright 2017 - 2018 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include <string.h>

#include "radio_protocol_wrapper.pb.h"

#ifndef SRC_JOYSTICK_JOYSTICK_H_
#define SRC_JOYSTICK_JOYSTICK_H_

#define JoyModelEnd \
  { 0, 0, 2, 0, 0, 0.0, 0.0 }

namespace joystick {
class Joystick {
 public:
  struct Model {
    unsigned int vbutton;  // virtual button mask
    unsigned char vaxis;   // virtual axis
    unsigned char type;    // 0=axis 1=button
    unsigned char index;   // axis or button indexuint8_t
    int min_value;         // minimum threshold value before
    float weight, bias;    // weight and bias to apply in transformation
  };

  Joystick();

  bool IsOpen() { return (fd != -1); }
  bool IsJoy(const char *key);

  bool Open(const char *dev);
  bool Open(int joy_idx);

  // -1 means wait indefinitely
  int ProcessEvents(int max_wait_time_ms);
  int Close();

  float axis(int idx) { return (vaxes[idx]); }

  void SetModel(const Model *_model, int _model_size) {
    model = _model;
    model_size = _model_size;
  }
  void SetModel(const Model *_model);
  void ApplyModel();

  const int MaxAxisVal;
  const double MaxAxisValInv;

  // virtual button mask
  unsigned press;    // newly pressed buttons
  unsigned release;  // just released buttons
  unsigned hold;     // buttons being held down

  int num_axes, num_buttons;

 private:
  int fd;

  static const int MAX_NAME_LEN = 128;
  static const int MAX_AXES = 16;
  static const int MAX_BUTTONS = 16;
  static const int MAX_VIRT_AXES = 16;

  static const int MaxModelSize = 1024;

  const Model *model;
  int model_size;

 public:
  char name[MAX_NAME_LEN];
  int axes[MAX_AXES];
  char buttons[MAX_BUTTONS];
  float vaxes[MAX_VIRT_AXES];
};
}  // namespace joystick

#endif  // SRC_JOYSTICK_JOYSTICK_H_
