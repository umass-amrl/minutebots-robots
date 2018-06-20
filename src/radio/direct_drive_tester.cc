// Copyright 2017 - 2018 joydeepb@cs.umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Direct radio command tester for UMass MinuteBots.
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

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "joystick/joystick.h"
#include "math/math_util.h"
#include "radio/crc.h"
#include "serial/serial.h"
#include "util/timer.h"

using joystick::Joystick;
using math_util::Ramp;
using math_util::DegToRad;
using std::max;
using std::min;
using std::string;

// Experimentally determined transmit rate to yield the most consistent latency
// with the XBee Pro SX radios.
const float kTransmitFrequency = 62.503125;

const int kNumRobotsPerPacket = 12;

struct __attribute__((packed)) RobotCommandPacket {
  // 8 bit Flags, from 0(LSB) to 7(MSB):
  // bit 0-3: Robot ID
  // bit 4: Flat Kick
  // bit 5: Chip Kick
  // bit 6: Send Feedback
  // bit 7: Beep
  uint8_t flags;

  // Velocity of individual wheels, with index 0 corresponding to the front left
  // wheel incrementing clockwise.
  int8_t wheel_velocity[4];

  // Kicker power.
  uint8_t kick_power;

  // Dribbler power.
  int8_t dribble_power;

  // Clear all command flags and values, preserving the robot ID.
  void Clear() {
    flags &= 0x0F;
    for (int i = 0; i < 4; ++i) {
      wheel_velocity[i] = 0;
    }
    kick_power = 0;
    dribble_power = 0;
  }

  // Issue a kick command.
  void Kick(int8_t power, bool chip_kick) {
    if (chip_kick) {
      flags |= 0x20;
    } else {
      flags |= 0x10;
    }
    kick_power = power;
  }

  static RobotCommandPacket IdleCommand() {
    RobotCommandPacket packet;
    packet.flags = 0xF;
    for (size_t i = 0; i < 4; ++i) {
      packet.wheel_velocity[i] = 0;
    }
    packet.kick_power = 0;
    packet.dribble_power = 0;
    return packet;
  }
};  // 7 bytes

struct __attribute__((packed)) RadioCommandPacket {
  // Prelude to radio packet, should be "RP" for "Robot Packet".
  uint8_t prelude[2];

  // RobotPacket data for up to 12 robots.
  RobotCommandPacket robots[kNumRobotsPerPacket];

  // CRC16 checksum.
  uint8_t fcs[2];

  // Default constructor: fill up prelude, no other initialization.
  RadioCommandPacket() {
    prelude[0] = 'R';
    prelude[1] = 'P';
  }

  // Copmute and fill up the CRC16 checksum.
  void ComputeCRC() {
    const uint16_t checksum = CCrc16::calc(
        reinterpret_cast<unsigned char*>(this), sizeof(RadioCommandPacket) - 2);
    fcs[0] = (checksum & 0xFF);
    fcs[1] = ((checksum >> 8) & 0xFF);
  }
};  // 2 + 7 * 12 + 2 = 88 bytes

// Serial port used for communication.
Serial serial_port_;

// Joystick used to drive the robot.
Joystick* joystick_ = nullptr;

// The ID of the robot commanded.
uint8_t robot_id_ = 0;

// Seed value for rand_r.
unsigned int rand_seed_ =
    static_cast<unsigned int>(fmod(GetMonotonicTime(), 10) * 1000000);

// Motion scale.
float motion_scale_ = 0.2;

// Default dribbler speed.
int dribbler_speed_ = 40;

// Default kick power.
int kick_power_ = 60;

// Simple test command.
void TestMotors(RobotCommandPacket* cmd_ptr) {
  static const double kAccelerationTime = 1.0;
  static const double kCruiseTime = 2.0;
  static const float kMaxSpeed = 50;
  RobotCommandPacket& cmd = *cmd_ptr;
  static double t_start = GetMonotonicTime();
  float v = 0;
  const double t = GetMonotonicTime() - t_start;
  if (t < kAccelerationTime) {
    v = Ramp<float>(t, 0, kAccelerationTime, 0, kMaxSpeed);
  } else if (t < kAccelerationTime + kCruiseTime) {
    v = kMaxSpeed;
  } else {
    v = Ramp<float>(t, kAccelerationTime + kCruiseTime,
                    kAccelerationTime + kCruiseTime + kAccelerationTime,
                    kMaxSpeed, 0);
  }
  if (t > kAccelerationTime + kCruiseTime + kAccelerationTime) {
    exit(0);
  }
  for (size_t i = 0; i < 4; ++i) {
    cmd.wheel_velocity[i] = v;
  }
}

float ApplyDeadZone(float axis, float dead_zone, float scale) {
  if (axis < dead_zone && axis > -dead_zone) {
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

float RateLimit(const float value, const float last_value, const float rate) {
  if (value < last_value - rate) {
    return (last_value - rate);
  } else if (value > last_value + rate) {
    return (last_value + rate);
  }
  return value;
}

// Read joystick axes and compute wheel speeds.
void ReadJoystick(RobotCommandPacket* cmd_ptr) {
  static const bool kPrintWheelCommands = false;
  static const int kKickTriggerThreshold = 30000;
  static const int kDeadZone = 8000;
  float speed_scale = 80.0;
  float rotation_scale = DegToRad(360.0f);
  const float kMaxVelocityRate = 2;
  const float kMaxRotationRate = 1;
  static float last_x_velocity = 0;
  static float last_y_velocity = 0;
  static float last_r_velocity = 0;

  RobotCommandPacket& cmd = *cmd_ptr;
  if (joystick_->ProcessEvents(0) < 0) {
    std::cerr << "Joystick Process Events Failure!\n";
    return;
  }

  if (joystick_->axes[5] > kKickTriggerThreshold) {
    // Issue a flat kick command.
    cmd_ptr->Kick(kick_power_, false);
    printf("Flat Kick %d\n", kick_power_);
  } else if (joystick_->axes[2] > kKickTriggerThreshold) {
    // Issue a flat kick command.
    cmd_ptr->Kick(kick_power_, true);
    printf("Chip Kick %d\n", kick_power_);
  }
  const float x_velocity =
      RateLimit(ApplyDeadZone(-static_cast<float>(joystick_->axes[4]),
                              kDeadZone, speed_scale / (32768.0f - kDeadZone)),
                last_x_velocity, kMaxVelocityRate);
  last_x_velocity = x_velocity;

  const float y_velocity =
      RateLimit(ApplyDeadZone(-static_cast<float>(joystick_->axes[3]),
                              kDeadZone, speed_scale / (32768.0f - kDeadZone)),
                last_y_velocity, kMaxVelocityRate);
  last_y_velocity = y_velocity;

  const float r_velocity = RateLimit(
      ApplyDeadZone(-static_cast<float>(joystick_->axes[0]), kDeadZone,
                    rotation_scale / (32768.0f - kDeadZone)),
      last_r_velocity, kMaxRotationRate);
  last_r_velocity = r_velocity;

  const float wheel_angles[] = {
      DegToRad<float>(90 + 54), DegToRad<float>(90 + 135),
      DegToRad<float>(90 - 135), DegToRad<float>(90 - 54)};
  for (int i = 0; i < 4; ++i) {
    const int v = -static_cast<int>(
      motion_scale_ * (x_velocity * cos(wheel_angles[i]) +
      y_velocity * sin(wheel_angles[i]) + r_velocity));
    if (kPrintWheelCommands) {
      printf("%4d ", v);
    }
    cmd.wheel_velocity[i] = v;
  }
  if (kPrintWheelCommands) {
    printf("\n");
  }
  if (abs(joystick_->axes[7]) > kDeadZone) {
    if (joystick_->axes[7] < 0) {
      kick_power_++;
    } else {
      kick_power_--;
    }
    kick_power_ = max(0, kick_power_);
    kick_power_ = min(200, kick_power_);
    printf("Kick: %d\n", kick_power_);
  }
  if (abs(joystick_->axes[6]) > kDeadZone) {
    if (joystick_->axes[6] > 0) {
      dribbler_speed_++;
    } else {
      dribbler_speed_--;
    }
    dribbler_speed_ = max(0, dribbler_speed_);
    dribbler_speed_ = min(100, dribbler_speed_);
    printf("Dribble: %d\n", dribbler_speed_);
  }
  if (joystick_->buttons[5]) {
    cmd.dribble_power = dribbler_speed_;
    printf("Dribble %d\n", dribbler_speed_);
  }
  if (joystick_->buttons[0]) {
    // Green button
    motion_scale_ = 0.2;
    printf("Motion scale = %f\n", motion_scale_);
  } else if (joystick_->buttons[2]) {
    // Blue button
    motion_scale_ = 0.4;
    printf("Motion scale = %f\n", motion_scale_);
  } else if (joystick_->buttons[3]) {
    // Yellow button
    motion_scale_ = 0.7;
    printf("Motion scale = %f\n", motion_scale_);
  } else if (joystick_->buttons[1]) {
    // Red button
    motion_scale_ = 1.0;
    printf("Motion scale = %f\n", motion_scale_);
  }
}

// Server mode infinite loop. Continuously sends packets at a fixed transmit
// rate.
void ServerMode() {
  static const int kJoystickID = 0;
  static const bool kRunTestMotors = false;
  static const char kGarbageData[16] = {0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0};
  static const bool kTestRobustness = false;

  if (!joystick_->Open(kJoystickID)) {
    fprintf(stderr, "ERROR: Unable to open joystick!\n");
    exit(1);
  }
  // We will be referring to hard-coded button indices 0-3 and 5.
  CHECK_GE(joystick_->num_buttons, 5);
  // Hard-coded usage of axes up to index 7.
  CHECK_GE(joystick_->num_axes, 8);

  RateLoop loop(kTransmitFrequency);
  RadioCommandPacket packet;
  for (size_t i = 0; i < kNumRobotsPerPacket; ++i) {
    packet.robots[i] = RobotCommandPacket::IdleCommand();
  }
  packet.ComputeCRC();
  RobotCommandPacket& cmd = packet.robots[0];
  cmd.flags = robot_id_;

  while (true) {
    cmd.Clear();
    if (kRunTestMotors) {
      TestMotors(&cmd);
      packet.ComputeCRC();
    } else {
      ReadJoystick(&cmd);
      packet.ComputeCRC();
    }

    // Write to serial_port
    const int bytes_written = serial_port_.write(&packet, sizeof(packet));
    if (bytes_written != sizeof(packet)) {
      perror("Error writing to serial port");
    }
    char read_buffer[4096];
    const int bytes_read = serial_port_.read(read_buffer, sizeof(read_buffer));
    if (bytes_read > 0) {
      // Append string termination character.
      read_buffer[bytes_read] = 0;
      printf("%s", read_buffer);
    }
    if (kTestRobustness) {
      const int garbage_length = rand_r(&rand_seed_) % sizeof(kGarbageData);
      serial_port_.write(kGarbageData, garbage_length);
      printf("Sent garbage data of length %d\n", garbage_length);
    }
    loop.Sleep();
  }
}

void PrintUsage() {
  printf("\n=========================================================\n");
  printf("Usage:\n");
  printf("Rotation:\t\tLeft stick, X-axis\n");
  printf("Translation:\t\tRight stick\n");
  printf("Chip Kick:\t\tLeft Trigger\n");
  printf("Flat Kick:\t\tRight Trigger\n");
  printf("Kick Power:\t\tD-Pad Up/Down\n");
  printf("Dribble:\t\tRight bumper\n");
  printf("Dribble Power:\t\tD-Pad Left/Right\n");
  printf("Motion Scale Buttons:\tGreen(A) = 0.2\n");
  printf("\t\t\tBlue(X) = 0.4\n");
  printf("\t\t\tYellow(Y) = 0.7\n");
  printf("\t\t\tRed(B) = 1.0\n");
  printf("=========================================================\n\n");
}

int main(int argc, char* argv[]) {
  // Ensure that the compiler has correctly packed the radio packet.
  CHECK_EQ(sizeof(RadioCommandPacket), 88u);
  // Ensure that the compiler has correctly packed the robot packet.
  CHECK_EQ(sizeof(RobotCommandPacket), 7u);
  if (argc < 2) {
    fprintf(stderr, "ERROR: Must specify serial port.\n");
  }
  if (argc < 3) {
    fprintf(stderr, "ERROR: Must robot ID.\n");
    return 1;
  }
  robot_id_ = atoi(argv[2]);
  printf("MinuteBots Direct Drive Tester\nRobot ID: %X\n", robot_id_);
  joystick_ = new Joystick();
  CHECK_NOTNULL(joystick_);
  const char* serial_port_name = argv[1];
  if (!serial_port_.open(serial_port_name, 230400)) {
    fprintf(stderr, "ERROR: Unable to open serial port.\n");
    exit(2);
  }
  PrintUsage();
  ServerMode();
  return 0;
}
