#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NOOPLOOP_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define NOOPLOOP_FRAME_HEADER 0x57
#define NOOPLOOP_FRAME_HEADER_1 0x00

typedef struct __attribute__((packed))
{
    uint8_t byteArray[3];
} nint24_t;

typedef struct __attribute__((packed))
{
    uint8_t header[2];
    uint8_t reserved0;
    uint8_t id;
    uint32_t system_time;
    nint24_t dis;
    uint8_t dis_status;
    uint16_t signal_strength;
    uint8_t range_precision;
    uint8_t check_sum;
} nts_frame0_raw_t;

#define NOOPLOOP_FRAME_LENGTH sizeof(nts_frame0_raw_t)

class AP_RangeFinder_NoopLoop : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
            return NEW_NOTHROW AP_RangeFinder_NoopLoop(_state, _params);
     }

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }
    virtual int8_t get_signal_quality_pct() const override { return signal_quality_pct; };

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;


    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t buffer[NOOPLOOP_FRAME_LENGTH];
    int8_t signal_quality_pct = 0;
    unsigned bufferPtr = 0;
    uint32_t max_reported_distance_mm = 0;
};

#endif  // AP_RANGEFINDER_NOOPLOOP_ENABLED
