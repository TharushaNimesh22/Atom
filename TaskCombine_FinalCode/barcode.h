#ifndef BARCODE_READER_H
#define BARCODE_READER_H

#include <Arduino.h>

class Robot;

class BARCODEreader {
public:
    // Constructor
    BARCODEreader(Robot *robotPtr);

    // Initialize reader
    void init();

    ///////////////// Main function
    bool run();

    // Call this repeatedly in loop to detect transitions and record timing
    void update();

    // Returns true when barcode reading is complete
    bool isComplete();

    // Print raw timing data
    void printTimings();

    // Convert timings to binary barcode
    void decodeBarcode();

    // Print decoded binary barcode
    void printBarcode();

    bool isOnWhite();   // check if corner sensor sees white

    int barcode_Val();
    // Expose count for debugging
    int getCount();

    
  
private:
    // Change to pointers if any problem arises
    Robot* robot;  // Pointer to Robot object

    static const int MAX_TRANSITIONS = 10;   // Fixed limit of 10 transitions
    unsigned long timestamps[MAX_TRANSITIONS]; 
    unsigned long new_timestamps[MAX_TRANSITIONS/2]; // Time intervals between transitions
    bool barcodeBits[MAX_TRANSITIONS/2];          // Decoded binary barcode
    int count;                                  // Number of recorded transitions

    bool lastState;                             // Previous sensor state
    unsigned long lastTime;                     // Time of last transition
    int newCount;

    bool complete;                              // Flag to indicate barcode read is done

    // Helper: detect transition (e.g., black to white)
    bool detectTransition(bool currentState);

    int perpendicularLineCount;
    bool barcodeReachedFirstTime;
};

#endif