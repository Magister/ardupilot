/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_NoopLoop.h"

#if AP_RANGEFINDER_NOOPLOOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

// format of serial packets received from NoopLoop TOF Sense P and F lidar
//
// Data Bit             Definition             Description
// -----------------------------------------------------------------------
// byte 0               Frame header           0x57
// byte 1               Function Mark          0x00
// byte 2               (Reserved)
// byte 3               Id                     id
// byte 4-7             System time            System time in ms
// byte 8-10            Distance*1000
// byte 11              Status                 Strength high 8 bits
// byte 12-13           Signal Strength
// byte 14              (Reserved)
// byte 15              Checksum

inline int32_t NLink_ParseInt24(nint24_t data)
{
    uint8_t *byte = (uint8_t *)(&data);
    return (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
}

inline bool verifyCheckSum(uint8_t *data, int32_t length)
{
    uint8_t sum = 0;
    for (int32_t i = 0; i < length - 1; ++i) {
        sum += data[i];
    }
    return sum == data[length - 1];
}

// distance returned in reading_m
bool AP_RangeFinder_NoopLoop::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    int32_t sum = 0;            // sum of all readings taken
    uint32_t sq = 0;            // sum of all signal qualities
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings

    // read any available lines from the lidar
    nts_frame0_raw_t *frame0 = (nts_frame0_raw_t *)buffer;
    uint32_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        // Add byte to buffer
        if (bufferPtr < NOOPLOOP_FRAME_LENGTH) {
            buffer[bufferPtr++] = c;
        }

        // Check header bytes
        if ((bufferPtr == 1) && (frame0->header[0] != NOOPLOOP_FRAME_HEADER)) {
            bufferPtr = 0;
            continue;
        }
        if ((bufferPtr == 2) && (frame0->header[1] != NOOPLOOP_FRAME_HEADER_1)) {
            bufferPtr = 0;
            continue;
        }

        // Check for complete packet
        if (bufferPtr == NOOPLOOP_FRAME_LENGTH) {
            if (verifyCheckSum(buffer, NOOPLOOP_FRAME_LENGTH)) {
                // Valid packet
                int32_t distance_mm = NLink_ParseInt24(frame0->dis);
                if (distance_mm == 0 || frame0->dis_status == 0) {
                    // invalid reading
                    invalid_count++;
                } else {
                    // correct data
                    max_reported_distance_mm = MAX(max_reported_distance_mm, distance_mm);
                    sum += distance_mm;
                    sq += frame0->signal_strength;
                    valid_count++;
                }
            }
            // Prepare for new packet
            bufferPtr = 0;
        }
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_m = (sum / valid_count) * 0.001f; // sensor reports mm, we need m;
        sq = sq / valid_count;
        if (sq > RangeFinder::SIGNAL_QUALITY_MAX) {
            signal_quality_pct = RangeFinder::SIGNAL_QUALITY_MAX;
        } else {
            signal_quality_pct = static_cast<uint8_t>(sq);
        }
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        signal_quality_pct = RangeFinder::SIGNAL_QUALITY_MIN;
        // return out of range distance + 1m
        reading_m = MIN(MAX(max_distance_cm(), max_reported_distance_mm * 0.01) + 100, UINT16_MAX) * 0.01f;
        return true;
    }

    // no readings so return false
    return false;
}

#endif  // AP_RANGEFINDER_NOOPLOOP_ENABLED
