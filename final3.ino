// --- CONSTANTS AND GLOBAL VARIABLES ---
const int sensors[8] = {32, 33, 34, 35, 36, 39, 25, 26};
const int THRESHOLD_WHITE = 600;

const int AIN1 = 23;
const int AIN2 = 22;
const int PWMA = 21;

const int BIN1 = 18;
const int BIN2 = 5;
const int PWMB = 17;

const int STBY = 19;

const int ENCODER_LEFT_A = 12;
const int ENCODER_LEFT_B = 13;
const int ENCODER_RIGHT_A = 14;
const int ENCODER_RIGHT_B = 27;

const int LED_PIN = 15;

const int NORMAL_SPEED = 250;
const int FAST_SPEED = 245;
const int TURN_SPEED_DRY = 125;
const int TURN_SPEED_FINAL = 120;
const int SLOW_SPEED = 100;
const int STOP_DELAY_MS = 100;
const int POST_TURN_STABILIZE_SPEED = 240;
const float POST_TURN_STABILIZE_DISTANCE = 30.0;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

const int ENCODER_PPR = 35;
const float WHEEL_DIAMETER = 4.4;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;
const float CM_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_PPR;

const float PRE_TURN_DISTANCE = 30.0;
const float PRE_TURN_DISTANCE_SECOND_RUN = 40.0;
const float END_DETECT_DISTANCE = 22.0;
const float PRE_TURN_ENDPOINT_CONFIRM = 33.0;

const int TURN_90_TICKS = 1;

unsigned long whiteDetectStartTime = 0;
bool whiteDetectStarted = false;
float whiteDetectDistanceTraveled = 0;
bool endpointHitInMove = false;

struct PathSegment {
    char move;
    char junctionType;
};

PathSegment path[100];
int pathLength = 0;

PathSegment finalPath[100];
int finalPathLength = 0;

long lastJunctionEncoderCount = 0;
const float MIN_JUNCTION_DISTANCE = 2.0;
bool junctionProcessed = false;
unsigned long lastJunctionTime = 0;
const unsigned long JUNCTION_DEBOUNCE_MS = 100;

bool sensorState[8];
bool sensorStateFiltered[8];
int sensorReadingBuffer[8][3];
int bufferIndex = 0;

int lastError = 0;
bool isTurning = false;

enum RunState {
    FIRST_RUN,
    WAITING,
    SECOND_RUN,
    COMPLETED
};

RunState currentRunState = FIRST_RUN;
unsigned long waitStartTime = 0;
int secondRunPathIndex = 0;
bool justCompletedTurn = false;
long postTurnEncoderStart = 0;



int junctionConfirmCount = 0;
const int JUNCTION_CONFIRM_THRESHOLD = 3;
const int SECOND_RUN_CONFIRM_THRESHOLD = 2;
char lastDetectedJunction = 'S';

// NEW: Lock for forcing left turn at cross/wide junctions
bool crossJunctionLeftLock = false;
unsigned long crossJunctionLockTime = 0;
const unsigned long CROSS_JUNCTION_LOCK_TIMEOUT = 2000;

// Function declarations
void readSensors();
bool isEndpointConditionMet();
bool checkEndpoint();
char detectJunctionType();
char checkJunctionWithConfirmation();
void handleFirstRunComplete();
void startSecondRun();
void handleSecondRunComplete();
void createOptimizedPath();
bool simplifyFinalPath();
void followOptimizedPath();
void executeOptimizedTurn(char move);
void makeLeftTurnOptimized(int speed, float preTurnDist);
void makeRightTurnDirectOptimized(int speed, float preTurnDist);
void makeUTurnOptimized(int speed, float preTurnDist);
void turnLeftByEncoder(long targetTicks, int speed);
void turnRightByEncoder(long targetTicks, int speed);
void handleTurn(char turn, char confirmedJunctionType);
void makeLeftTurn(int speed);
void makeRightTurn(int speed);
void makeUTurn(int speed);
void moveForwardDistance(float distanceCM);
void moveBackwardDistance(float distanceCM);
void alignToLine();
void followLine();
bool simplifyPath();
void resumeMovement();
char confirmJunctionType(char initialDetection);
bool isSecondRunEndpointReached();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void adjustLeft(int baseSpeed, int adjustment);
void adjustRight(int baseSpeed, int adjustment);
void stopMotors();

void IRAM_ATTR encoderLeftISR() {
    if (digitalRead(ENCODER_LEFT_B) == digitalRead(ENCODER_LEFT_A)) {
        encoderLeftCount++;
    } else {
        encoderLeftCount--;
    }
}

void IRAM_ATTR encoderRightISR() {
    if (digitalRead(ENCODER_RIGHT_B) == digitalRead(ENCODER_RIGHT_A)) {
        encoderRightCount++;
    } else {
        encoderRightCount--;
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, CHANGE);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    for (int i = 0; i < 8; i++) {
        pinMode(sensors[i], INPUT);
    }
    
    digitalWrite(STBY, HIGH);
    
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            sensorReadingBuffer[i][j] = 0;
        }
        sensorStateFiltered[i] = false;
    }
    
    delay(2000);
    Serial.println("Starting Line Maze Solver (Auto-Start Final Run after 7 seconds)");
    Serial.println("Initial State: FIRST_RUN - Learning the maze...");
    lastJunctionEncoderCount = (encoderLeftCount + encoderRightCount) / 2;
}

void loop() {
    readSensors();
    
    if (currentRunState == WAITING || currentRunState == COMPLETED) {
        stopMotors();
        if (currentRunState == WAITING && millis() - waitStartTime >= 7000) {
            Serial.println("===========================================");
            Serial.println("7 SECONDS ELAPSED - STARTING FINAL RUN NOW!");
            Serial.println("===========================================");
            startSecondRun();
        }
        return;
    }
    
    if (endpointHitInMove) {
        return;
    }
    
    if (checkEndpoint()) {
        return;
    }
    
    if (!isTurning) {
        if (currentRunState == FIRST_RUN) {
            char turn = checkJunctionWithConfirmation();
            
            if (turn != 'S') {
                long currentEncoderAvg = (encoderLeftCount + encoderRightCount) / 2;
                float distanceSinceLastJunction = abs(currentEncoderAvg - lastJunctionEncoderCount) * CM_PER_TICK;
                
                if (distanceSinceLastJunction >= MIN_JUNCTION_DISTANCE) {
                    lastJunctionEncoderCount = currentEncoderAvg;
                    char confirmedJunctionType = confirmJunctionType(turn);
                    
                    if (confirmedJunctionType == 'O' || confirmedJunctionType == 'T') {
                        if (detectJunctionType() == 'R') {
                            turn = 'R';
                        } else if (detectJunctionType() == 'L') {
                            turn = 'L';
                        } else {
                            turn = 'L';
                        }
                    } else if (confirmedJunctionType == 'C' || confirmedJunctionType == 'W') {
                        turn = 'L';  // This will be overridden by the lock anyway
                    }
                    
                    handleTurn(turn, confirmedJunctionType);
                    junctionConfirmCount = 0;
                } else {
                    Serial.print("Junction too close (");
                    Serial.print(distanceSinceLastJunction);
                    Serial.println("cm) - ignoring to prevent false detection");
                    followLine();
                }
            } else {
                followLine();
            }
        } else if (currentRunState == SECOND_RUN) {
            followOptimizedPath();
        }
    }
}

void readSensors() {
    for (int i = 0; i < 8; i++) {
        int value = analogRead(sensors[i]);
        sensorState[i] = (value < THRESHOLD_WHITE);
        sensorReadingBuffer[i][bufferIndex] = sensorState[i] ? 1 : 0;
    }
    
    for (int i = 0; i < 8; i++) {
        int sum = 0;
        for (int j = 0; j < 3; j++) {
            sum += sensorReadingBuffer[i][j];
        }
        sensorStateFiltered[i] = (sum >= 2);
    }
    
    bufferIndex = (bufferIndex + 1) % 3;
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        Serial.print("Raw: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(sensorState[i] ? "W" : "B");
        }
        Serial.print(" | Filtered: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(sensorStateFiltered[i] ? "W" : "B");
        }
        Serial.print(" | L_Enc: ");
        Serial.print(encoderLeftCount);
        Serial.print(" | R_Enc: ");
        Serial.println(encoderRightCount);
        lastPrint = millis();
    }
}

bool isEndpointConditionMet() {
    int whiteCount = 0;
    for (int i = 0; i < 8; i++) {
        if (sensorStateFiltered[i]) whiteCount++;
    }
    return (whiteCount >= 7);
}

bool isSecondRunEndpointReached() {
    // Skip endpoint detection at the very start
    if (secondRunPathIndex == 0) {
        long currentEncoderAvg = (encoderLeftCount + encoderRightCount) / 2;
        if (abs(currentEncoderAvg) < 100) {
            return false;
        }
    }
    
    // Also skip if we haven't completed all planned turns yet
    // Only check for endpoint after all path moves are done
    if (secondRunPathIndex < finalPathLength) {
        return false;  // Still have turns to make, don't check endpoint yet
    }
    
    if (isEndpointConditionMet()) {
        if (!whiteDetectStarted) {
            whiteDetectStarted = true;
            whiteDetectStartTime = millis();
            lastEncoderLeftCount = encoderLeftCount;
            lastEncoderRightCount = encoderRightCount;
            Serial.println("=== SECOND RUN: All sensors WHITE - checking endpoint ===");
            moveForward(TURN_SPEED_FINAL);
        } else {
            long leftTicks = abs(encoderLeftCount - lastEncoderLeftCount);
            long rightTicks = abs(encoderRightCount - lastEncoderRightCount);
            float avgTicks = (leftTicks + rightTicks) / 2.0;
            whiteDetectDistanceTraveled = avgTicks * CM_PER_TICK;
            
            // Use same distance as first run for consistency
            if (whiteDetectDistanceTraveled >= END_DETECT_DISTANCE) {
                Serial.print("=== SECOND RUN ENDPOINT CONFIRMED! Distance: ");
                Serial.print(whiteDetectDistanceTraveled);
                Serial.println(" cm ===");
                stopMotors();
                whiteDetectStarted = false;
                whiteDetectDistanceTraveled = 0;
                return true;
            }
        }
    } else {
        if (whiteDetectStarted) {
            Serial.print("White detection lost at ");
            Serial.print(whiteDetectDistanceTraveled);
            Serial.println(" cm - was a junction, not endpoint");
        }
        whiteDetectStarted = false;
        whiteDetectDistanceTraveled = 0;
    }
    return false;
}

bool checkEndpoint() {
    if (isTurning) {
        return false;
    }
    
    if (currentRunState == SECOND_RUN) {
        return isSecondRunEndpointReached();
    }
    
    if (isEndpointConditionMet()) {
        if (!whiteDetectStarted) {
            whiteDetectStarted = true;
            whiteDetectStartTime = millis();
            lastEncoderLeftCount = encoderLeftCount;
            lastEncoderRightCount = encoderRightCount;
            Serial.println("All white detected - checking if endpoint (60cm)...");
            int speed = (currentRunState == SECOND_RUN) ? FAST_SPEED : NORMAL_SPEED;
            moveForward(speed);
        } else {
            long leftTicks = abs(encoderLeftCount - lastEncoderLeftCount);
            long rightTicks = abs(encoderRightCount - lastEncoderRightCount);
            float avgTicks = (leftTicks + rightTicks) / 2.0;
            whiteDetectDistanceTraveled = avgTicks * CM_PER_TICK;
            
            if (whiteDetectDistanceTraveled >= END_DETECT_DISTANCE) {
                Serial.print("ENDPOINT CONFIRMED! Distance traveled: ");
                Serial.print(whiteDetectDistanceTraveled);
                Serial.println(" cm");
                stopMotors();
                whiteDetectStarted = false;
                whiteDetectDistanceTraveled = 0;
                return true;
            }
        }
    } else {
        whiteDetectStarted = false;
        whiteDetectDistanceTraveled = 0;
    }
    return false;
}

char checkJunctionWithConfirmation() {
    char currentJunction = detectJunctionType();
    int threshold = (currentRunState == SECOND_RUN) ? SECOND_RUN_CONFIRM_THRESHOLD : JUNCTION_CONFIRM_THRESHOLD;
    
    if (currentJunction == lastDetectedJunction && currentJunction != 'S') {
        junctionConfirmCount++;
    } else {
        junctionConfirmCount = 0;
        lastDetectedJunction = currentJunction;
    }
    
    if (junctionConfirmCount >= threshold) {
        return currentJunction;
    }
    return 'S';
}

char detectJunctionType() {
    int whiteCount = 0;
    for (int i = 0; i < 8; i++) {
        if (sensorStateFiltered[i]) whiteCount++;
    }
    
    if (currentRunState == SECOND_RUN && secondRunPathIndex == 0) {
        long currentEncoderAvg = (encoderLeftCount + encoderRightCount) / 2;
        if (abs(currentEncoderAvg) < 50) {
            return 'S';
        }
    }
    
    if (whiteCount >= 7) {
        return 'L';
    }
    
    if (whiteCount == 0) {
        return 'U';
    }
    
    if (sensorStateFiltered[0] && sensorStateFiltered[1] && sensorStateFiltered[2] &&
        (sensorStateFiltered[3] || sensorStateFiltered[4])) {
        return 'L';
    }
    
    if (sensorStateFiltered[5] && sensorStateFiltered[6] && sensorStateFiltered[7] &&
        (sensorStateFiltered[3] || sensorStateFiltered[4])) {
        return 'R';
    }
    
    if (sensorStateFiltered[0] && !sensorStateFiltered[3] && !sensorStateFiltered[4]) {
        return 'L';
    }
    
    if (sensorStateFiltered[7] && !sensorStateFiltered[3] && !sensorStateFiltered[4]) {
        return 'R';
    }
    
    return 'S';
}

// MODIFIED: confirmJunctionType now sets the lock when all-white is detected
char confirmJunctionType(char initialDetection) {
    const float JUNCTION_CONFIRM_DISTANCE = 1.0;
    
    // When all white detected initially, set the lock for left turn IMMEDIATELY
    if (initialDetection == 'L' && isEndpointConditionMet()) {
        Serial.println("=== ALL WHITE DETECTED - SETTING LEFT TURN LOCK ===");
        crossJunctionLeftLock = true;  // LOCK: Bot will take left after pre-forward
        
        Serial.println("Potential Cross Junction Detected (All White)");
        moveForwardDistance(JUNCTION_CONFIRM_DISTANCE);
        readSensors();
        
        if (isEndpointConditionMet()) {
            Serial.println("JUNCTION TYPE: CROSS (C) - LEFT TURN LOCKED");
            moveBackwardDistance(JUNCTION_CONFIRM_DISTANCE);
            return 'C';
        } else {
            Serial.println("JUNCTION TYPE: WIDE (W) - LEFT TURN LOCKED");
            moveBackwardDistance(JUNCTION_CONFIRM_DISTANCE);
            return 'W';
        }
    }
    
    if ((sensorStateFiltered[3] || sensorStateFiltered[4]) &&
        (sensorStateFiltered[5] || sensorStateFiltered[6] || sensorStateFiltered[7]) &&
        !(sensorStateFiltered[0] || sensorStateFiltered[1] || sensorStateFiltered[2])) {
        Serial.println("Potential 2-Way Explicit Right (T)");
        moveForwardDistance(JUNCTION_CONFIRM_DISTANCE);
        readSensors();
        if (sensorStateFiltered[3] || sensorStateFiltered[4]) {
            Serial.println("JUNCTION TYPE: TWO-WAY EXPLICIT RIGHT (T)");
            moveBackwardDistance(JUNCTION_CONFIRM_DISTANCE);
            return 'T';
        } else {
            moveBackwardDistance(JUNCTION_CONFIRM_DISTANCE);
            return 'T';
        }
    }
    
    if ((initialDetection == 'L' && (sensorStateFiltered[0] || sensorStateFiltered[1])) ||
        (initialDetection == 'R' && (sensorStateFiltered[6] || sensorStateFiltered[7]))) {
        Serial.println("JUNCTION TYPE: ONE-WAY (O)");
        return 'O';
    }
    
    return 'S';
}

void handleFirstRunComplete() {
    Serial.println("===========================================");
    Serial.println("FIRST RUN COMPLETE! Endpoint reached!");
    Serial.println("===========================================");
    
    stopMotors();
    createOptimizedPath();
    
    Serial.println("===========================================");
    Serial.print("Optimized path for second run: ");
    for (int i = 0; i < finalPathLength; i++) {
        Serial.print(finalPath[i].move);
    }
    Serial.println();
    Serial.println("===========================================");
    
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(300);
        digitalWrite(LED_PIN, LOW);
        delay(300);
    }
    
    Serial.println("===========================================");
    Serial.println("WAITING 7 SECONDS BEFORE AUTOMATIC START...");
    Serial.println("Final run will begin automatically!");
    Serial.println("===========================================");
    
    currentRunState = WAITING;
    waitStartTime = millis();
}

void startSecondRun() {
    Serial.println("===========================================");
    Serial.println("STARTING SECOND RUN - SPEED RUN!");
    Serial.println("Following optimized path...");
    Serial.println("===========================================");
    
    whiteDetectStarted = false;
    whiteDetectDistanceTraveled = 0;
    endpointHitInMove = false;
    isTurning = false;
    junctionConfirmCount = 0;
    justCompletedTurn = false;
    crossJunctionLeftLock = false;
    crossJunctionLockTime = 0;
    
    currentRunState = SECOND_RUN;
    secondRunPathIndex = 0;
    lastError = 0;
    
    encoderLeftCount = 0;
    encoderRightCount = 0;
    lastJunctionEncoderCount = 0;
    
    delay(1000);
    Serial.println("Second run initialized - Starting optimized path!");
}

void handleSecondRunComplete() {
    Serial.println("===========================================");
    Serial.println("SECOND RUN COMPLETE! Maze solved!");
    Serial.println("===========================================");
    
    stopMotors();
    currentRunState = COMPLETED;
    
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
}

void createOptimizedPath() {
    finalPathLength = 0;
    
    for (int i = 0; i < pathLength; i++) {
        finalPath[finalPathLength++] = path[i];
    }
    
    Serial.println("Starting path optimization...");
    Serial.print("Initial path: ");
    for (int i = 0; i < finalPathLength; i++) {
        Serial.print(finalPath[i].move);
    }
    Serial.println();
    
    bool simplified = true;
    int passCount = 0;
    while (simplified && passCount < 10) {
        simplified = simplifyFinalPath();
        if (simplified) {
            passCount++;
            Serial.print("Pass ");
            Serial.print(passCount);
            Serial.print(": ");
            for (int i = 0; i < finalPathLength; i++) {
                Serial.print(finalPath[i].move);
            }
            Serial.println();
        }
    }
    
    while (finalPathLength > 0 && finalPath[finalPathLength - 1].move == 'E') {
        finalPathLength--;
    }
    
    Serial.print("Final optimized path: ");
    for (int i = 0; i < finalPathLength; i++) {
        Serial.print(finalPath[i].move);
    }
    Serial.println();
}

bool simplifyFinalPath() {
    if (finalPathLength < 3) return false;
    
    bool madeChange = false;
    
    for (int i = 0; i < finalPathLength - 2; i++) {
        char before = finalPath[i].move;
        char middle = finalPath[i+1].move;
        char after = finalPath[i+2].move;
        char replacement = '\0';
        
        if (middle == 'U') {
            if (before == 'L' && after == 'R') replacement = 'U';
            else if (before == 'L' && after == 'S') replacement = 'R';
            else if (before == 'L' && after == 'L') replacement = 'S';
            else if (before == 'R' && after == 'L') replacement = 'U';
            else if (before == 'R' && after == 'S') replacement = 'L';
            else if (before == 'R' && after == 'R') replacement = 'S';
            else if (before == 'S' && after == 'L') replacement = 'R';
            else if (before == 'S' && after == 'R') replacement = 'L';
            else if (before == 'S' && after == 'S') replacement = 'U';
        }
        
        if (replacement != '\0') {
            finalPath[i].move = replacement;
            for (int j = i + 1; j < finalPathLength - 2; j++) {
                finalPath[j] = finalPath[j + 2];
            }
            finalPathLength -= 2;
            madeChange = true;
            Serial.print("Simplified: ");
            Serial.print(before);
            Serial.print(middle);
            Serial.print(after);
            Serial.print(" -> ");
            Serial.println(replacement);
            break;
        }
    }
    return madeChange;
}

void followOptimizedPath() {
    if (secondRunPathIndex >= finalPathLength) {
        Serial.println("Path exhausted, seeking endpoint...");
        
        // Check for endpoint when path is complete
        if (isEndpointConditionMet()) {
            if (!whiteDetectStarted) {
                whiteDetectStarted = true;
                lastEncoderLeftCount = encoderLeftCount;
                lastEncoderRightCount = encoderRightCount;
                Serial.println("=== PATH COMPLETE: All sensors WHITE - confirming endpoint ===");
            }
            
            long leftTicks = abs(encoderLeftCount - lastEncoderLeftCount);
            long rightTicks = abs(encoderRightCount - lastEncoderRightCount);
            float avgTicks = (leftTicks + rightTicks) / 2.0;
            whiteDetectDistanceTraveled = avgTicks * CM_PER_TICK;
            
            if (whiteDetectDistanceTraveled >= END_DETECT_DISTANCE) {
                Serial.println("=== ENDPOINT REACHED - SECOND RUN COMPLETE! ===");
                handleSecondRunComplete();
                return;
            }
        } else {
            whiteDetectStarted = false;
            whiteDetectDistanceTraveled = 0;
        }
        
        followLine();
        return;
    }
    
    char detectedJunction = checkJunctionWithConfirmation();
    
    if (detectedJunction == 'S') {
        char rawDetection = detectJunctionType();
        long currentEncoderAvg = (encoderLeftCount + encoderRightCount) / 2;
        float distanceSinceLastJunction = abs(currentEncoderAvg - lastJunctionEncoderCount) * CM_PER_TICK;
        
        if ((rawDetection == 'L' || rawDetection == 'R' || rawDetection == 'U') &&
            distanceSinceLastJunction >= (MIN_JUNCTION_DISTANCE * 0.8)) {
            detectedJunction = rawDetection;
            Serial.println("EARLY DETECTION! Forcing turn based on raw sensors to prevent overshoot.");
        }
    }
    
    if (detectedJunction != 'S') {
        long currentEncoderAvg = (encoderLeftCount + encoderRightCount) / 2;
        float distanceSinceLastJunction = abs(currentEncoderAvg - lastJunctionEncoderCount) * CM_PER_TICK;
        
        if (distanceSinceLastJunction >= MIN_JUNCTION_DISTANCE) {
            char nextTurn = finalPath[secondRunPathIndex].move;
            
            Serial.print("Junction detected (");
            Serial.print(detectedJunction);
            Serial.print(") - Executing PLANNED turn: ");
            Serial.print(nextTurn);
            Serial.print(" (Index: ");
            Serial.print(secondRunPathIndex);
            Serial.println(")");
            
            lastJunctionEncoderCount = currentEncoderAvg;
            executeOptimizedTurn(nextTurn);
            secondRunPathIndex++;
            junctionConfirmCount = 0;
        } else {
            followLine();
        }
    } else {
        followLine();
    }
}

void executeOptimizedTurn(char turn) {
    isTurning = true;
    stopMotors();
    delay(STOP_DELAY_MS);
    
    int currentTurnSpeed = TURN_SPEED_FINAL;
    float preTurnDist = PRE_TURN_DISTANCE_SECOND_RUN;
    
    switch(turn) {
        case 'L':
            makeLeftTurnOptimized(currentTurnSpeed, preTurnDist);
            break;
        case 'R':
            makeRightTurnDirectOptimized(currentTurnSpeed, preTurnDist);
            break;
        case 'S':
            Serial.println("Going straight through junction (Optimized)");
            moveForwardDistance(preTurnDist);
            justCompletedTurn = true;
            postTurnEncoderStart = (encoderLeftCount + encoderRightCount) / 2;
            break;
        case 'U':
            makeUTurnOptimized(currentTurnSpeed, preTurnDist);
            break;
        case 'E':
            Serial.println("Reached Final Path End.");
            stopMotors();
            break;
    }
    
    resumeMovement();
}

void makeLeftTurnOptimized(int speed, float preTurnDist) {
    Serial.println("LEFT turn (Optimized)");
    moveForwardDistance(preTurnDist);
    if (endpointHitInMove) return;
    
    stopMotors();
    delay(50);
    turnLeftByEncoder(TURN_90_TICKS, speed);
}

void makeRightTurnDirectOptimized(int speed, float preTurnDist) {
    Serial.println("Executing RIGHT turn (optimized)");
    moveForwardDistance(preTurnDist);
    if (endpointHitInMove) return;
    
    stopMotors();
    delay(50);
    turnRightByEncoder(TURN_90_TICKS, speed);
}

void makeUTurnOptimized(int speed, float preTurnDist) {
    Serial.println("U-TURN (Optimized, Left Rotation)");
    moveForwardDistance(preTurnDist);
    if (endpointHitInMove) return;
    
    stopMotors();
    delay(50);
    turnLeftByEncoder(TURN_90_TICKS * 2, speed);
}

void turnLeftByEncoder(long targetTicks, int speed) {
    Serial.print("Turning left ");
    Serial.print(targetTicks);
    Serial.println(" ticks");
    
    turnLeft(speed);
    delay(300);
    
    bool lineLost = false;
    unsigned long phase2Start = millis();
    while (!lineLost && millis() - phase2Start < 1200) {
        readSensors();
        if (!sensorStateFiltered[0] && !sensorStateFiltered[1] && !sensorStateFiltered[2] &&
            !sensorStateFiltered[3] && !sensorStateFiltered[4] && !sensorStateFiltered[5] &&
            !sensorStateFiltered[6] && !sensorStateFiltered[7]) {
            lineLost = true;
            delay(150);
        }
        delay(5);
    }
    
    bool lineFound = false;
    while (!lineFound) {
        readSensors();
        if (sensorStateFiltered[3] || sensorStateFiltered[4]) {
            lineFound = true;
            delay(50);
        }
        delay(5);
    }
}

void turnRightByEncoder(long targetTicks, int speed) {
    Serial.print("Turning right ");
    Serial.print(targetTicks);
    Serial.println(" ticks");
    
    turnRight(speed);
    delay(300);
    
    bool lineLost = false;
    unsigned long phase2Start = millis();
    while (!lineLost && millis() - phase2Start < 1200) {
        readSensors();
        if (!sensorStateFiltered[0] && !sensorStateFiltered[1] && !sensorStateFiltered[2] &&
            !sensorStateFiltered[3] && !sensorStateFiltered[4] && !sensorStateFiltered[5] &&
            !sensorStateFiltered[6] && !sensorStateFiltered[7]) {
            lineLost = true;
            delay(150);
        }
        delay(5);
    }
    
    bool lineFound = false;
    while (!lineFound) {
        readSensors();
        if (sensorStateFiltered[3] || sensorStateFiltered[4]) {
            lineFound = true;
            delay(50);
        }
        delay(5);
    }
}

void resumeMovement() {
    if (endpointHitInMove) {
        if (currentRunState == FIRST_RUN) {
            path[pathLength++] = {'E', 'E'};
            handleFirstRunComplete();
        } else if (currentRunState == SECOND_RUN) {
            handleSecondRunComplete();
        }
        endpointHitInMove = false;
        return;
    }
    isTurning = false;
}

// MODIFIED: handleTurn now checks the crossJunctionLeftLock first
void handleTurn(char turn, char confirmedJunctionType) {
    isTurning = true;
    stopMotors();
    delay(STOP_DELAY_MS);
    
    int currentTurnSpeed = TURN_SPEED_DRY;
    
    // Check if lock has expired (safety timeout)
    if (crossJunctionLeftLock && (millis() - crossJunctionLockTime > CROSS_JUNCTION_LOCK_TIMEOUT)) {
        Serial.println("Cross junction lock EXPIRED - resetting");
        crossJunctionLeftLock = false;
    }
    
    // If cross junction lock is active, force left turn regardless of 'turn' parameter
    if (crossJunctionLeftLock) {
        Serial.println("=== CROSS/WIDE JUNCTION LOCK ACTIVE - FORCING LEFT TURN ===");
        makeLeftTurn(currentTurnSpeed);
        path[pathLength++] = {'L', confirmedJunctionType};
        crossJunctionLeftLock = false;  // Reset the lock after executing
        resumeMovement();
        return;
    }
    
    // Also check current sensor state - if still seeing all white, force left
    int currentWhiteCount = 0;
    for (int i = 0; i < 8; i++) {
        if (sensorStateFiltered[i]) currentWhiteCount++;
    }
    if (currentWhiteCount >= 6 && (confirmedJunctionType == 'C' || confirmedJunctionType == 'W')) {
        Serial.println("=== STILL SEEING ALL WHITE AT JUNCTION - FORCING LEFT TURN ===");
        makeLeftTurn(currentTurnSpeed);
        path[pathLength++] = {'L', confirmedJunctionType};
        resumeMovement();
        return;
    }
    
    switch(turn) {
        case 'L':
            makeLeftTurn(currentTurnSpeed);
            path[pathLength++] = {'L', confirmedJunctionType};
            break;
        case 'R':
            makeRightTurn(currentTurnSpeed);
            break;
        case 'U':
            makeUTurn(currentTurnSpeed);
            path[pathLength++] = {'U', confirmedJunctionType};
            simplifyPath();
            break;
    }
    resumeMovement();
}

void makeLeftTurn(int speed) {
    Serial.println("LEFT turn (Dry Run)");
    moveForwardDistance(PRE_TURN_DISTANCE);
    if (endpointHitInMove) return;
    
    stopMotors();
    delay(150);
    turnLeftByEncoder(TURN_90_TICKS, speed);
    stopMotors();
    delay(150);
    alignToLine();
}

void makeRightTurn(int speed) {
    Serial.println("RIGHT turn (Dry Run)");
    moveForwardDistance(PRE_TURN_DISTANCE);
    if (endpointHitInMove) return;
    
    stopMotors();
    delay(150);
    readSensors();
    
    bool straightAvailable = (sensorStateFiltered[3] && sensorStateFiltered[4]) ||
                             (sensorStateFiltered[3] || sensorStateFiltered[4]);
    
    if (straightAvailable) {
        Serial.println("Going STRAIGHT (Overriding R)");
        path[pathLength++] = {'S', 'S'};
        return;
    }
    
    Serial.println("Turning RIGHT");
    turnRightByEncoder(TURN_90_TICKS, speed);
    stopMotors();
    delay(150);
    alignToLine();
    path[pathLength++] = {'R', 'O'};
}

void makeUTurn(int speed) {
    Serial.println("U-TURN (Dry Run) - Left Rotation, Aligned");
    moveForwardDistance(PRE_TURN_DISTANCE);
    if (endpointHitInMove) return;
    
    stopMotors();
    delay(150);
    turnLeftByEncoder(TURN_90_TICKS * 2, speed);
    stopMotors();
    delay(150);
    alignToLine();
}

void moveForwardDistance(float distanceCM) {
    long targetTicks = distanceCM / CM_PER_TICK;
    long startLeftCount = encoderLeftCount;
    long startRightCount = encoderRightCount;
    
    Serial.print("Moving forward ");
    Serial.print(distanceCM);
    Serial.print(" cm (");
    Serial.print(targetTicks);
    Serial.println(" ticks target)");
    
    int speed = (currentRunState == SECOND_RUN) ? FAST_SPEED : NORMAL_SPEED;
    moveForward(speed);
    
    while (true) {
        readSensors();
        
        if (isEndpointConditionMet()) {
            if (currentRunState == SECOND_RUN) {
                if (!whiteDetectStarted) {
                    whiteDetectStarted = true;
                    lastEncoderLeftCount = encoderLeftCount;
                    lastEncoderRightCount = encoderRightCount;
                    Serial.println("All 8 sensors WHITE during pre-turn - checking 30cm...");
                }
                
                long leftTicks = abs(encoderLeftCount - lastEncoderLeftCount);
                long rightTicks = abs(encoderRightCount - lastEncoderRightCount);
                float avgTicks = (leftTicks + rightTicks) / 2.0;
                float whiteDistance = avgTicks * CM_PER_TICK;
                
                if (whiteDistance >= PRE_TURN_ENDPOINT_CONFIRM) {
                    Serial.print("ENDPOINT CONFIRMED DURING PRE-TURN! All 8 sensors WHITE for ");
                    Serial.print(whiteDistance);
                    Serial.println(" cm");
                    stopMotors();
                    endpointHitInMove = true;
                    whiteDetectStarted = false;
                    
                    for (int i = 0; i < 3; i++) {
                        digitalWrite(LED_PIN, HIGH);
                        delay(100);
                        digitalWrite(LED_PIN, LOW);
                        delay(100);
                    }
                    break;
                }
            } else if (currentRunState == FIRST_RUN) {
                if (!whiteDetectStarted) {
                    whiteDetectStarted = true;
                    lastEncoderLeftCount = encoderLeftCount;
                    lastEncoderRightCount = encoderRightCount;
                    Serial.println("White detected during pre-turn movement - checking distance...");
                }
                
                long leftTicks = abs(encoderLeftCount - lastEncoderLeftCount);
                long rightTicks = abs(encoderRightCount - lastEncoderRightCount);
                float avgTicks = (leftTicks + rightTicks) / 2.0;
                float whiteDistance = avgTicks * CM_PER_TICK;
                
                if (whiteDistance >= END_DETECT_DISTANCE) {
                    Serial.println("ENDPOINT CONFIRMED DURING PRE-TURN MOVEMENT!");
                    stopMotors();
                    endpointHitInMove = true;
                    whiteDetectStarted = false;
                    
                    for (int i = 0; i < 3; i++) {
                        digitalWrite(LED_PIN, HIGH);
                        delay(100);
                        digitalWrite(LED_PIN, LOW);
                        delay(100);
                    }
                    break;
                }
            }
        } else {
            if (whiteDetectStarted) {
                Serial.println("White lost - was just a wide junction, continuing...");
                whiteDetectStarted = false;
            }
        }
        
        long leftTicks = abs(encoderLeftCount - startLeftCount);
        long rightTicks = abs(encoderRightCount - startRightCount);
        long avgTicks = (leftTicks + rightTicks) / 2;
        
        if (avgTicks >= targetTicks) {
            Serial.print("Target reached! Left: ");
            Serial.print(leftTicks);
            Serial.print(" Right: ");
            Serial.print(rightTicks);
            Serial.print(" Avg: ");
            Serial.println(avgTicks);
            stopMotors();
            whiteDetectStarted = false;
            break;
        }
        
        delay(5);
    }
}

void moveBackwardDistance(float distanceCM) {
    long targetTicks = distanceCM / CM_PER_TICK;
    long startLeftCount = encoderLeftCount;
    long startRightCount = encoderRightCount;
    
    Serial.print("Moving backward ");
    Serial.print(distanceCM);
    Serial.print(" cm (");
    Serial.print(targetTicks);
    Serial.println(" ticks target)");
    
    int speed = SLOW_SPEED;
    moveBackward(speed);
    
    while (true) {
        long leftTicks = abs(encoderLeftCount - startLeftCount);
        long rightTicks = abs(encoderRightCount - startRightCount);
        long avgTicks = (leftTicks + rightTicks) / 2;
        
        if (avgTicks >= targetTicks) {
            Serial.println("Backward movement complete.");
            stopMotors();
            break;
        }
        delay(5);
    }
}

void alignToLine() {
    int speed = (currentRunState == SECOND_RUN) ? TURN_SPEED_FINAL : SLOW_SPEED;
    
    for (int i = 0; i < 30; i++) {
        readSensors();
        
        if ((sensorStateFiltered[3] && sensorStateFiltered[4])) {
            break;
        }
        
        if ((sensorStateFiltered[3] && !sensorStateFiltered[1] && !sensorStateFiltered[6]) ||
            (sensorStateFiltered[4] && !sensorStateFiltered[1] && !sensorStateFiltered[6])) {
            break;
        }
        
        if (sensorStateFiltered[2] || sensorStateFiltered[1] || sensorStateFiltered[0]) {
            turnLeft(speed);
            delay(40);
            stopMotors();
            delay(30);
        } else if (sensorStateFiltered[5] || sensorStateFiltered[6] || sensorStateFiltered[7]) {
            turnRight(speed);
            delay(40);
            stopMotors();
            delay(30);
        } else {
            break;
        }
    }
    stopMotors();
    delay(100);
}

void followLine() {
    int baseSpeed = (currentRunState == SECOND_RUN) ? FAST_SPEED : NORMAL_SPEED;
    int slowSpeed = (currentRunState == SECOND_RUN) ? TURN_SPEED_FINAL : SLOW_SPEED;
    
    if (currentRunState == SECOND_RUN && justCompletedTurn) {
        long currentEncoderAvg = (encoderLeftCount + encoderRightCount) / 2;
        float distanceSinceTurn = abs(currentEncoderAvg - postTurnEncoderStart) * CM_PER_TICK;
        
        if (distanceSinceTurn < POST_TURN_STABILIZE_DISTANCE) {
            baseSpeed = POST_TURN_STABILIZE_SPEED;
            Serial.print("Post-turn stabilization: ");
            Serial.print(distanceSinceTurn);
            Serial.println(" cm");
        } else {
            justCompletedTurn = false;
            Serial.println("Post-turn stabilization COMPLETE - resuming full speed");
        }
    }
    
    if (sensorStateFiltered[3] && sensorStateFiltered[4]) {
        moveForward(baseSpeed);
        lastError = 0;
    } else if (sensorStateFiltered[3] && sensorStateFiltered[2]) {
        adjustLeft(baseSpeed, 30);
        lastError = -1;
    } else if (sensorStateFiltered[4] && sensorStateFiltered[5]) {
        adjustRight(baseSpeed, 30);
        lastError = 1;
    } else if (sensorStateFiltered[2] || sensorStateFiltered[1]) {
        adjustLeft(baseSpeed, 50);
        lastError = -2;
    } else if (sensorStateFiltered[5] || sensorStateFiltered[6]) {
        adjustRight(baseSpeed, 50);
        lastError = 2;
    } else if (sensorStateFiltered[0]) {
        adjustLeft(baseSpeed, 70);
        lastError = -3;
    } else if (sensorStateFiltered[7]) {
        adjustRight(baseSpeed, 70);
        lastError = 3;
    } else if (sensorStateFiltered[3] || sensorStateFiltered[4]) {
        moveForward(baseSpeed);
        lastError = 0;
    } else {
        if (lastError < 0) {
            adjustLeft(slowSpeed, 80);
        } else if (lastError > 0) {
            adjustRight(slowSpeed, 80);
        } else {
            moveForward(slowSpeed);
        }
    }
}

bool simplifyPath() {
    if (pathLength < 3) return false;
    
    bool madeChange = false;
    
    for (int i = pathLength - 3; i >= 0; i--) {
        char before = path[i].move;
        char middle = path[i+1].move;
        char after = path[i+2].move;
        char replacement = '\0';
        
        if (middle == 'U') {
            if (before == 'L' && after == 'R') replacement = 'U';
            else if (before == 'L' && after == 'S') replacement = 'R';
            else if (before == 'L' && after == 'L') replacement = 'S';
            else if (before == 'R' && after == 'L') replacement = 'U';
            else if (before == 'R' && after == 'S') replacement = 'L';
            else if (before == 'R' && after == 'R') replacement = 'S';
            else if (before == 'S' && after == 'L') replacement = 'R';
            else if (before == 'S' && after == 'R') replacement = 'L';
            else if (before == 'S' && after == 'S') replacement = 'U';
        }
        
        if (replacement != '\0') {
            path[i].move = replacement;
            for (int j = i + 1; j < pathLength - 2; j++) {
                path[j] = path[j + 2];
            }
            pathLength -= 2;
            madeChange = true;
            break;
        }
    }
    return madeChange;
}

void moveForward(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
    
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
}

void moveBackward(int speed) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
    
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed);
}

void turnLeft(int speed) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
    
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
}

void turnRight(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
    
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed);
}

void adjustLeft(int baseSpeed, int adjustment) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, max(0, baseSpeed - adjustment));
    
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, min(255, baseSpeed + adjustment));
}

void adjustRight(int baseSpeed, int adjustment) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, min(255, baseSpeed + adjustment));
    
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, max(0, baseSpeed - adjustment));
}

void stopMotors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
    
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
}
