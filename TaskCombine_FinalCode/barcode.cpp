#include "barcode.h"
#include "robot.h"

BARCODEreader::BARCODEreader(Robot *robotPtr)
: count(0), lastState(false), lastTime(0), complete(false)
{
    for (int i = 0; i < MAX_TRANSITIONS; i++) {
        timestamps[i] = 0;
        barcodeBits[i] = false;
    }

    this -> robot = robotPtr;
}

void BARCODEreader::init() {
    // ir = new IRarray();   // Create IRarray instance
    // ir->init();           // Initialize sensor pins
    perpendicularLineCount = 0;
    barcodeReachedFirstTime;

    robot -> detectPerpendicularLine = false;
    // robot -> _baseSpeed = 100;

     //////////////////////// Set PID parameters ///////////////
    robot -> _pid.Kp = 55.5f;   // Initial value 55.5f
    robot -> _pid.Ki = 0.0f;
    robot -> _pid.Kd = 0.03f;    ///       0.03

    
    // robot -> _GAP_TIMEOUT = 1800;
    robot -> _baseSpeed = 100;    //// 80
    robot -> _isFirstCall = false;
    // arrowCount = 1;

    // previousDirection = 1;
    // Set the last time
    robot -> _completeLineLost = false;
    robot -> _lastTime = millis();

    
    lastState = false;    // Assume starting on black
    count = 0;
    complete = false;
    

    ////// Initial moving ///////////////////////////////////////////////////////////////////////////
    robot -> _steppers.runForward(-(robot -> _baseSpeed), 400);
    robot -> turnBack();

    lastTime = millis();  // Initialize timing

    ////////////// Line follow until the perpendicular line is found
    bool doneTurning = false;

    while(!doneTurning){
        doneTurning =  robot ->lineFollowEndsWithTurn();
            // delay(5);
    }
    

    lastTime = millis();

    /////////////////////////////////////////////////////////////////////////////////////
    
}

bool BARCODEreader::isOnWhite() {
    robot -> _irsensors.read();
    return robot -> _irsensors.values[4];   // corner
}

bool BARCODEreader::detectTransition(bool currentState) {
    return currentState != lastState;
}

void BARCODEreader::update() {
    if (complete || count >= MAX_TRANSITIONS) return;

    robot -> _irsensors.read();
    bool currentState = robot -> _irsensors.values[4];  // Use corner sensor

    if (detectTransition(currentState)) {
        unsigned long now = millis();
        unsigned long duration = now - lastTime;

        if(lastState){
            timestamps[count] = duration;
        }
        lastTime = now;
        lastState = currentState;
        count++;
        
        if (count >= MAX_TRANSITIONS) {
            complete = true;
        }
    }
}

bool BARCODEreader::isComplete() {   /// Not needed
    return complete;
}



// This convert the timestamp array to bit array.

void BARCODEreader::decodeBarcode() {
    if (!complete) return;
    //keep only odd-indexed timestamps (1, 3, 5, ...)
    newCount = 0;

    for (int i = 1; i < count; i += 2) {
        new_timestamps[newCount] = timestamps[i];
        newCount++;
    }

    // Compute average duration of new timestamps
    unsigned long sum = 0;
    for (int i = 0; i < newCount; i++) {
        sum += new_timestamps[i];
    }
    unsigned long avg = (newCount > 0) ? (sum / newCount) : 0;

    // Threshold: longer than average → 1, shorter → 0
    for (int i = 0; i < newCount; i++) {
        barcodeBits[newCount - i - 1] = (new_timestamps[i] > avg);
    }

    // return barcodeBits;
}



//Calculate the barcode value.

int BARCODEreader::barcode_Val() {
    int sum = 0;
    // Loop backwards through the filtered bits
    for (int i = newCount - 1; i >= 0; i--) {
        sum = (sum << 1) | (barcodeBits[i] ? 1 : 0);
    }
    return sum;
}


int BARCODEreader::getCount() {
    return count;
}


/*

//  For debugging purposes

void BARCODEreader::printTimings() {
    Serial.println("Raw timing data:");
    for (int i = 0; i < count; i++) {
        Serial.print(timestamps[i]);
        Serial.print(" ");
    }
    Serial.println();
}

//For printing purpose
void BARCODEreader::printBarcode() {
    Serial.println("Decoded barcode:");
    for (int i = 0; i < newCount; i++) {
        Serial.print(barcodeBits[i] ? "1" : "0");
    }
    Serial.println();

    Serial.println("Decoded barcode (reversed):");
    for (int i = newCount - 1; i >= 0; i--) {
        Serial.print(barcodeBits[i] ? "1" : "0");
    }
    Serial.println();

}
*/

////////////////////////////////////////////
bool BARCODEreader::run(){
    /////////////////// Line follow ////////////////
    if (perpendicularLineCount == 3){
        ///////////// End of the task
        robot -> stopTheRobot(false);  // Reset for the next;

           robot ->clearDisplay();
            robot -> writeToDisplay(2, 20, 10, "Task done");

        return true;
    
    } else if (perpendicularLineCount == 1){
        //// Combe back for barcode
        robot -> _steppers.setSpeed(robot -> _baseSpeed, robot -> _baseSpeed);


        if (barcodeReachedFirstTime) {
            while (!isOnWhite()) {
                robot -> _steppers.runMotors();
                delay(10);  // poll every 10 ms
            }
            barcodeReachedFirstTime = false;
        }

        // robot -> _steppers.runForward(-baseSpeed, )
        
        robot -> _steppers.runMotors();

        // Continuously update barcode reader
        update();

        if (complete) {
            robot -> _steppers.stop();
            decodeBarcode();

            // Add display barcode value functionality
            robot -> barcodeOutput = (barcode_Val() % 3) + 2;

            //// Write to display
            char output[1];
            sprintf(output, "%d", robot -> barcodeOutput);
            robot ->clearDisplay();
            robot -> writeToDisplay(2, 20, 10, output);

            

            delay(100);


            /// Testing
            robot -> detectPerpendicularLine = false;
            complete = false;
            perpendicularLineCount = 2;

            robot -> _lastTime = millis();
            return false;

        }

        delay(5);  /// motor step speed
        
    } else if(perpendicularLineCount == 2){
            robot -> lineFollow();
            if (robot -> detectPerpendicularLine) perpendicularLineCount++;

    } else {
        ///////// Line follow at the other places
        robot -> lineFollow();
        if (robot -> detectPerpendicularLine) perpendicularLineCount++;

        if (perpendicularLineCount == 1) {
            // When the barcode is reached, go back few steps and come again..
            delay(100);
            robot -> _steppers.runForward(-(robot -> _baseSpeed), 100);
            delay(100);

            barcodeReachedFirstTime = true;
        }
    }

    return false;
}
