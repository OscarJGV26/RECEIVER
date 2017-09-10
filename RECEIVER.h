#pragma once
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#define RCIN_PRUSS_RAM_BASE 0x4a303000
class RECEIVER{
    private:
        static const uint32_t TICK_PER_US = 200;
        static const uint32_t NUM_RING_ENTRIES = 300;
        struct ring_buffer {
        volatile uint16_t ring_head; // owned by ARM CPU
        volatile uint16_t ring_tail; // owned by the PRU
            struct {
                volatile uint32_t s1_t; // 5ns per tick
                volatile uint32_t s0_t; // 5ns per tick
            } buffer[NUM_RING_ENTRIES];
        };
        volatile struct ring_buffer *ring_buffer;
        int channel_counter=0;
        int number_channels=8;
        uint16_t raw_channels[8];
        int channels[8];
        uint16_t upper_limits[8];
        uint16_t lower_limits[8];
        uint16_t mid_points[8];
        uint16_t pulse_limit=2700;              //Anything above this will be considered synching pulse
        uint16_t limit_ch(uint16_t ch);
        void setMidPoints();
        void setLimits();
        void printMidPoints();
        void printLimits();
        void offset_channels();
        void processPulse(uint16_t width_1,uint16_t width_2);
        void save_calibration();
        int read_calibration();
        void print_calibration();
        int check_calibration();
    public:
        void begin(int cal);
        void read();
        int calibrate();
};