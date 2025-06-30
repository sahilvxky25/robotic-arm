#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "esp_camera.h"

// =================================================================
// ============== CRITICAL: USER CONFIGURATION =====================
// =================================================================

// -- WIFI CREDENTIALS
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// -- ROBOT ARM DIMENSIONS (in mm) - **MEASURE YOUR ROBOT ARM!**
// These are placeholder values. You MUST replace them with your arm's actual link lengths.
const float L1 = 100.0; // Height from base to shoulder joint
const float L2 = 150.0; // Length of the upper arm (shoulder to elbow)
const float L3 = 180.0; // Length of the forearm (elbow to wrist)
// Add other link lengths as needed for your specific 6-DOF arm geometry.

// -- SERVO PINS
#define SERVO_PIN_BASE    12
#define SERVO_PIN_SHOULDER 13
#define SERVO_PIN_ELBOW   14
#define SERVO_PIN_WRIST_PITCH 15
#define SERVO_PIN_WRIST_ROLL  27
#define SERVO_PIN_GRIPPER     26

// -- SERVO LIMITS & OFFSETS (in degrees)
// Calibrate these to prevent your robot from damaging itself.
// [MIN, MAX]
const int servoLimits[6][2] = {
  {0, 180},   // Base
  {15, 165},  // Shoulder
  {0, 180},   // Elbow
  {0, 180},   // Wrist Pitch
  {0, 180},   // Wrist Roll
  {10, 80}    // Gripper (10=closed, 80=open)
};

// -- CAMERA MODEL - **UNCOMMENT THE ONE YOU ARE USING**
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
#define CAMERA_MODEL_AI_THINKER

// Include the correct camera pin definition file
#if defined(CAMERA_MODEL_AI_THINKER)
  #include "camera_pins.h"
#else
  #error "Camera model not selected"
#endif

// =================================================================
// =================== GLOBAL VARIABLES ============================
// =================================================================

WebServer server(80);

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWristPitch;
Servo servoWristRoll;
Servo servoGripper;
Servo servos[] = {servoBase, servoShoulder, servoElbow, servoWristPitch, servoWristRoll, servoGripper};

// Target coordinates received from the computer
float targetX = 150.0, targetY = 0.0, targetZ = 150.0;
bool newTargetReceived = false;

// =================================================================
// =================== INVERSE KINEMATICS (IK) =====================
// =================================================================

/**
 * @brief Calculates the joint angles for a target (X, Y, Z).
 * @param x The target X coordinate.
 * @param y The target Y coordinate.
 * @param z The target Z coordinate.
 * @param angles An array to store the calculated angles [base, shoulder, elbow, ...].
 * @return True if a solution is found, false otherwise.
 *
 * NOTE: This is a simplified 3-DOF IK for positioning. A full 6-DOF solution
 * also requires orientation angles (roll, pitch, yaw) for the gripper.
 * The mathematics here are HIGHLY dependent on your robot's specific geometry.
 * You will likely need to derive or find the correct equations for your arm.
 */
bool calculateIK(float x, float y, float z, float angles[6]) {
    // --- Base Angle (Theta 1) ---
    angles[0] = atan2(y, x) * (180.0 / PI);

    // --- Arm Angles (Theta 2 and 3) ---
    // Calculations for a 2-link planar arm in the vertical plane
    float r = sqrt(x*x + y*y); // Distance from base to wrist in the XY plane
    float d = sqrt(pow(r, 2) + pow(z - L1, 2));

    // Check if the target is reachable
    if (d > L2 + L3) {
        Serial.println("Error: Target is unreachable.");
        return false;
    }

    // Cosine rule to find angles
    float alpha = acos((L2*L2 + L3*L3 - d*d) / (2 * L2 * L3));
    float beta = acos((L2*L2 + d*d - L3*L3) / (2 * L2 * d));
    
    // Calculate final angles for shoulder and elbow
    angles[2] = 180.0 - (alpha * (180.0 / PI)); // Elbow angle
    angles[1] = (atan2(z - L1, r) + beta) * (180.0 / PI); // Shoulder angle

    // --- Wrist and Gripper Angles (Placeholder) ---
    // These would be determined by the required gripper orientation
    angles[3] = 90; // Wrist Pitch - keep it level for now
    angles[4] = 90; // Wrist Roll
    angles[5] = servoLimits[5][1]; // Gripper open

    Serial.printf("IK Solution: Base=%.2f, Shoulder=%.2f, Elbow=%.2f\n", angles[0], angles[1], angles[2]);
    return true;
}


// =================================================================
// ================== MOTION CONTROL ===============================
// =================================================================

/**
 * @brief Moves all servos to the specified angles.
 * @param angles The array of target angles for each servo.
 * @param speedDelay Delay between steps for smoother, slower movement.
 */
void moveServos(float angles[6], int speedDelay = 15) {
    // This function can be improved with gradual stepping to prevent jerky movements
    for (int i = 0; i < 6; i++) {
        float targetAngle = constrain(angles[i], servoLimits[i][0], servoLimits[i][1]);
        servos[i].write(targetAngle);
        delay(speedDelay); // Small delay for coordination
    }
}

void gripperOpen() {
    servoGripper.write(servoLimits[5][1]); // Assumes max value is open
    delay(500);
}

void gripperClose() {
    servoGripper.write(servoLimits[5][0]); // Assumes min value is closed
    delay(500);
}

// =================================================================
// ================ OBJECT DETECTION (Placeholder) =================
// =================================================================

/**
 * @brief Finds an object using the camera and returns its 3D coordinates.
 * @param outX Pointer to store the object's X coordinate.
 * @param outY Pointer to store the object's Y coordinate.
 * @param outZ Pointer to store the object's Z coordinate.
 * @return True if an object is found, false otherwise.
 *
 * INNOVATION OPPORTUNITY: This is where you can innovate.
 * - Simple: Detect the largest blob of a specific color (e.g., bright red).
 * - Advanced: Run a lightweight TensorFlow Lite model to recognize specific objects.
 *
 * The translation from 2D pixel to 3D world coordinate is non-trivial.
 * A common simplification is to assume the object is on a flat surface (Z=0)
 * and use the camera's known height and angle to calculate X and Y.
 */
bool findObject(float* outX, float* outY, float* outZ) {
    Serial.println("Attempting to find object...");
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    // ============ Placeholder Vision Logic =============
    // This is a highly simplified placeholder. You need to implement
    // real image processing here (e.g., using OpenCV on a PC, or a
    // simplified blob detection on the ESP32).
    // For now, it just returns a fixed "detected" position.
    
    *outX = 180.0; // Placeholder X (mm)
    *outY = 20.0;  // Placeholder Y (mm)
    *outZ = 0.0;   // Placeholder Z (mm), assuming on the table
    
    Serial.printf("Placeholder: Object 'found' at (%.2f, %.2f, %.2f)\n", *outX, *outY, *outZ);
    // ===================================================

    esp_camera_fb_return(fb); // IMPORTANT: return the frame buffer
    return true;
}

// =================================================================
// ================= WEB SERVER HANDLERS ===========================
// =================================================================

void handleRoot() {
    String html = "<h1>ESP32 Robotic Arm Controller</h1>";
    html += "<p>Send target coordinates via /set?x=VAL&y=VAL&z=VAL</p>";
    server.send(200, "text/html", html);
}

void handleSetCoords() {
    if (server.hasArg("x") && server.hasArg("y") && server.hasArg("z")) {
        targetX = server.arg("x").toFloat();
        targetY = server.arg("y").toFloat();
        targetZ = server.arg("z").toFloat();
        newTargetReceived = true;
        String response = "Received new target: X=" + String(targetX) + ", Y=" + String(targetY) + ", Z=" + String(targetZ);
        server.send(200, "text/plain", response);
        Serial.println(response);
    } else {
        server.send(400, "text/plain", "Bad Request: Missing coordinates");
    }
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}


// =================================================================
// ======================= SETUP & LOOP ============================
// =================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\nBooting Robotic Arm Controller...");

    // -- Initialize Camera
    // (Camera initialization code is lengthy, place it here)
    camera_config_t config;
    // ... Full camera config ...
    config.pixel_format = PIXFORMAT_JPEG;
    // ...
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }


    // -- Initialize Servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servos[0].attach(SERVO_PIN_BASE);
    servos[1].attach(SERVO_PIN_SHOULDER);
    servos[2].attach(SERVO_PIN_ELBOW);
    servos[3].attach(SERVO_PIN_WRIST_PITCH);
    servos[4].attach(SERVO_PIN_WRIST_ROLL);
    servos[5].attach(SERVO_PIN_GRIPPER);

    // -- Initialize WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // -- Initialize Web Server
    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_GET, handleSetCoords);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started.");

    // -- Go to Home Position
    Serial.println("Moving to Home Position.");
    float homeAngles[6] = {90, 150, 30, 90, 90, servoLimits[5][1]}; // A safe, upright position
    moveServos(homeAngles);
    gripperOpen();
    Serial.println("System Ready.");
}

void loop() {
    server.handleClient(); // Handle incoming HTTP requests

    if (newTargetReceived) {
        newTargetReceived = false; // Reset the flag

        Serial.println("\n--- Starting Pick and Place Sequence ---");

        // 1. Find the object
        float objX, objY, objZ;
        if (!findObject(&objX, &objY, &objZ)) {
            Serial.println("Failed to find object. Aborting sequence.");
            return;
        }

        // 2. Move above the object
        Serial.println("Moving to pre-pick position...");
        float angles[6];
        if (calculateIK(objX, objY, objZ + 40.0, angles)) { // 40mm above
            moveServos(angles);
            delay(1000);
        } else { return; }

        // 3. Descend to pick the object
        Serial.println("Moving to pick position...");
        if (calculateIK(objX, objY, objZ, angles)) {
            moveServos(angles);
            delay(1000);
        } else { return; }

        // 4. Pick up
        Serial.println("Closing gripper...");
        gripperClose();

        // 5. Lift the object
        Serial.println("Lifting object...");
        if (calculateIK(objX, objY, objZ + 50.0, angles)) {
            moveServos(angles);
            delay(1000);
        } else { return; }

        // 6. Move to the target drop-off location
        Serial.println("Moving to drop-off position...");
        if (calculateIK(targetX, targetY, targetZ, angles)) {
            moveServos(angles);
            delay(2000);
        } else { return; }

        // 7. Place the object
        Serial.println("Opening gripper...");
        gripperOpen();
        
        // 8. Retreat from drop-off point
        Serial.println("Retreating...");
        if (calculateIK(targetX, targetY, targetZ + 50.0, angles)) {
            moveServos(angles);
            delay(1000);
        } else { return; }

        // 9. Return to Home Position
        Serial.println("Returning home...");
        float homeAngles[6] = {90, 150, 30, 90, 90, servoLimits[5][1]};
        moveServos(homeAngles, 20);
        
        Serial.println("--- Sequence Complete ---");
    }
}