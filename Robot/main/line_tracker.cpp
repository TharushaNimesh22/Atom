#include "line_tracker.h"

LINEtracker::LINEtracker() {
    // Initialize all weights to 0; real setup happens in init()
    for (int i = 0; i < NUM_SENSORS; i++) {
        weights[i] = 0.0f;
    }
}

void LINEtracker::init() {
    // Assign symmetric weights around zero[-2, -1, 0, +1, +2]
    for (int i = 0; i < NUM_SENSORS; i++) {
        weights[i] = i - (NUM_SENSORS - 1)/2.0f;
    }
}

float LINEtracker::computePosition(const bool readings[]) {
    float sumWeights = 0.0f;
    float count = 0.0f;

    for (int i = 0; i < NUM_SENSORS; i++) {
        // Count only if ir reading is 1
        if (readings[i]) {
            sumWeights += weights[i];
            count += 1.0f;
        }
    }

    // Line lost( special case )
    if (count == 0) return 999.0f;   

    //Line position
    return sumWeights / count;       
}

// To be modified
bool LINEtracker::isLineLost(const bool readings[]) {
    for (int i = 0; i < NUM_SENSORS; i++)
        if (readings[i]) {
          return false;
        }
    return true;
}


bool LINEtracker::isPerpendicularLine(const bool readings[]){
    for (int i = 0; i < NUM_SENSORS; i++)
        if (!readings[i]) {
          return false;
        }
    return true;
}
