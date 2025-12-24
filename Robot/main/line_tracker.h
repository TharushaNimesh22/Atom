#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <Arduino.h>
#include "ir_array.h"

class LINEtracker {
public:
    LINEtracker();
       
    void init();

    // Compute and Return position
    float computePosition(const bool readings[]); 
    bool isLineLost(const bool readings[]);

    // Return ture if a perpendicular line is detected (all 5 IR sensors return 1)
    bool isPerpendicularLine(const bool readings[]);

private:
    // weights[-2, -1, 0, +1, +2]
    float weights[NUM_SENSORS];
};

#endif