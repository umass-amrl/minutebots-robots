// Copyright 2017-2018 slane@cs.umass.edu
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


#include "joystick/joystick.h"

#include <linux/joystick.h>
#include <sys/poll.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

namespace joystick {
static const bool Debug = false;

Joystick::Joystick() : MaxAxisVal(32767), MaxAxisValInv(1.0 / MaxAxisVal) {
  fd = -1;
  model = NULL;
  model_size = 0;

  num_axes = num_buttons = 0;
  for (size_t i = 0; i < MAX_NAME_LEN; ++i) {
    name[i] = 0;
  }
  for (size_t i = 0; i < MAX_AXES; ++i) {
    axes[i] = 0;
    vaxes[i] = 0;
  }
  for (size_t i = 0; i < MAX_BUTTONS; ++i) {
    buttons[i] = 0;
  }
}

bool Joystick::IsJoy(const char *key) {
  bool match = (strstr(name, key) != NULL);
  if (Debug) printf("name=[%s] key=[%s] ret=%d\n", name, key, match);
  return(match);
}

bool Joystick::Open(const char *dev) {
  static const bool debug = false;
  if (debug) printf("opening %s\n", dev);

  Close();
  fd = ::open(dev, O_RDONLY);
  if (fd < 0) return(false);

  ioctl(fd, JSIOCGAXES,               &num_axes);
  ioctl(fd, JSIOCGBUTTONS,            &num_buttons);
  ioctl(fd, JSIOCGNAME(MAX_NAME_LEN), name);

  if (num_axes > MAX_AXES) num_axes = MAX_AXES;
  if (num_buttons > MAX_BUTTONS) num_buttons = MAX_BUTTONS;

  return(true);
}

bool Joystick::Open(int joy_idx) {
  static const int DevMax = 32;
  char dev[DevMax];

  snprintf(dev, DevMax, "/dev/input/js%d", joy_idx);
  if (Open(dev)) return(true);

  snprintf(dev, DevMax, "/dev/js%d", joy_idx);
  if (Open(dev)) return(true);

  return(false);
}

int Joystick::ProcessEvents(int max_wait_time_ms) {
  struct pollfd polls[1];
  int total_num_events = 0;
  int num_events;
  struct js_event jse;

  if (fd == -1) return(-1);

  polls[0].fd = fd;
  polls[0].events = POLLIN | POLLPRI;
  polls[0].revents = 0;

  while ((num_events = poll(polls, 1, max_wait_time_ms)) > 0) {
    total_num_events += num_events;
    if (polls[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
      fprintf(stderr, "error returned from poll on joystick\n");
      return(-1);
    }

    if (polls[0].revents & (POLLIN | POLLPRI)) {
      if (read(fd, &jse, sizeof(jse)) != sizeof(jse)) {
        perror("read failed on pollable joystick\n");
        return(-1);
      }

      switch (jse.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
          if (Debug) printf("axis %d now has value %d\n", jse.number,
                            jse.value);
          axes[jse.number] = jse.value;
          break;
        case JS_EVENT_BUTTON:
          if (Debug) printf("button %d now has value %d\n", jse.number,
                           jse.value);
          buttons[jse.number] = jse.value;
          break;
      }
    }
  }

  if (num_events < 0) {
    perror("Unable to poll joystick");
    return(-1);
  }

  ApplyModel();

  return(total_num_events);
}


float ApplyDeadZone(float axis, float dead_zone, float scale) {
  if (axis < dead_zone && axis >  -dead_zone) {
    return 0;
  }
  if (axis > dead_zone) {
    return (scale * (axis - dead_zone));
  }
  if (axis < -dead_zone) {
    return (scale * (axis + dead_zone));
  }
  return 0;
}

void Joystick::SetModel(const Model *_model) {
  // look for terminator at end of model
  int n = 0;
  while (n < MaxModelSize && _model[n].type < 2) n++;

  SetModel(_model, n);
}

void Joystick::ApplyModel() {
  static const bool debug = false;
  if (!model) return;

  // clear current virtual axes
  for (int i = 0; i < MAX_VIRT_AXES; i++) vaxes[i] = 0.0;
  unsigned old_hold = hold;
  hold = 0;

  if (debug) {
    for (int i = 0; i < num_buttons; i++) {
      printf("%d", buttons[i]);
    }
    printf("\n");
  }

  // apply model
  for (int i = 0; i < model_size; i++) {
    Model m = model[i];
    float v = 0.0;

    if (m.type) {
      v = buttons[m.index];
      if (v > 0.0) hold |= m.vbutton;
    } else {
      if (abs(axes[m.index]) > m.min_value) {
        v = axes[m.index]*MaxAxisValInv;
        if (axes[m.index] >= 0) {
          v -= m.min_value * MaxAxisValInv;
        } else {
          v += m.min_value * MaxAxisValInv;
        }
      }
    }

    float a = v * m.weight + m.bias;
    // if(a > 0.0) vbutton |= m.vbutton;

    if (debug) {
      printf("%d:(%d,%d) %f*%f + %f = %f\n",
             m.vaxis, m.type, m.index,
             v, m.weight, m.bias, a);
    }

    if (m.vaxis < MAX_VIRT_AXES) vaxes[m.vaxis] += a;
  }

  press   = (hold) & (~old_hold);
  release = (~hold) & (old_hold);

  if (debug) {
    int i, n = num_axes;
    while (n > 0 && vaxes[n-1] == 0.0) n--;

    printf("vaxes:");
    for (i = 0; i < n; i++) {
      printf("  %d:%+5.2f", i, vaxes[i]);
    }
    printf("\n");
  }

  if (false) {
    int MaxBits = 16;

    printf("press[");
    for (int i = 0; i < MaxBits; i++) printf("%d", (press >> i)&0x1);
    printf("] ");

    printf("release[");
    for (int i = 0; i < MaxBits; i++) printf("%d", (release >> i)&0x1);
    printf("] ");

    printf("hold[");
    for (int i = 0; i < MaxBits; i++) printf("%d", (hold >> i)&0x1);
    printf("]\n");
  }
}

int Joystick::Close() {
  if (fd >= 0) ::close(fd);

  return(1);
}
}  // namespace joystick
