// Copyright 2017 not-us
//
//
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
// ========================================================================


#include<cstdint>

#ifndef SRC_YISIBOT_RADIO_CRC_H_
#define SRC_YISIBOT_RADIO_CRC_H_

class CCrc16 {
 public:
  static const uint16_t crc16_table[];

  // Recompute the FCS with one more character appended.
  static uint16_t calc_fcs(uint16_t fcs, char c) {
    return (((fcs) >> 8) ^ crc16_table[((fcs) ^ (c)) & 0xff]);
  }

  // Recompute the FCS with len bytes appended.
  static uint16_t calc(unsigned char  *buf, int len);
};

class CCrc8 {
 public:
  static const unsigned char crc8_table[];

  static unsigned char calc_fcs(uint16_t fcs, unsigned char c) {
    return crc8_table[(fcs) ^ (c)];
  }

  static unsigned char calc(unsigned char *buf, int len);
};

#endif  // SRC_YISIBOT_RADIO_CRC_H_

