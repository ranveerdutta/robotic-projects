#include <WiFiS3.h>
#include <WiFiServer.h>
#include <IRremote.h>

// Wi-Fi Access Point credentials
char ssid[] = "ArduinoCar";     
char pass[] = "12345678";       

WiFiServer server(80);

const char htmlPage[] = R"rawliteral(
    <!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
    body { font-family: Arial, sans-serif; background: #111; color: white; margin: 0; text-align: center; }
    h1 { background: #222; padding: 15px; margin: 0; }
    .controller { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-gap: 10px; padding: 20px; max-width: 400px; margin: auto; }
    .row { display: flex; justify-content: center; margin-bottom: 10px; }
    .btn { width: 70px; height: 70px; font-size: 20px; font-weight: bold; border: none; border-radius: 15px; margin: 5px; cursor: pointer; transition: 0.2s; }
    .btn:active { transform: scale(0.95); }
    .forward { background: #4CAF50; }
    .backward { background: #555; }
    .left { background: #2196F3; }
    .right { background: #2196F3; }
    .stop { background: #f44336; }
    .rotate { background: #FF9800; }
    .sweep { background: #9C27B0; }
    </style></head><body>
    <h1>WIFI RC Car Controller</h1>
    <div class='controller'>
    <div></div>
    <button class='btn forward' onclick="sendCommand('F')">Fwd</button>
    <div></div>

    <button class='btn left' onclick="sendCommand('L')">Left</button>
    <button class='btn stop' onclick="sendCommand('S')">Stop</button>
    <button class='btn right' onclick="sendCommand('R')">Right</button>

    <div></div>
    <button class='btn backward' onclick="sendCommand('B')">Back</button>
    <div></div>
    </div>

    <div class='row'>
    <button class='btn rotate' onclick="sendCommand('LR')">Left Rotate</button>
    <button class='btn rotate' onclick="sendCommand('RR')">Right Rotate</button>
    </div>

    <div class='row'>
    <button class='btn stop' onclick="sendCommand('FD')">Fwd Drop</button>
    <button class='btn stop' onclick="sendCommand('BD')">Back Drop</button>
    </div>

    <script>
      function sendCommand(cmd) {
        fetch(`/${cmd}`).catch(err => console.log("Error sending command:", err));
      }
    </script>

    </body></html>

    )rawliteral";

// Motor driver pins (adjust as per your wiring)
const int ENA = 5;   // PWM - Left wheels
const int ENB = 6;   // PWM - Right wheels
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 10;

//mode pins
const int MODE_PIN_1 = A4;
const int MODE_PIN_2 = A5;


//IR Pins
const int FWD_IR = 12;
const int BACK_IR = 13;


//FW Ultrasonic sensor pins
const int fwdTrigPin = 3;
const int fwdEchoPin = 4;
unsigned long lastDistanceCheckFwd = 0;
const unsigned long distanceInterval = 500; // check every 500 ms
const int obstacleThresholdFwd = 30; // in cm

//IR Receiver
const int IR_RECV_PIN = 2;

//BW Ultrasonic sensor pins
const int bwdTrigPin = 11;
const int bwdEchoPin = A3;
unsigned long lastDistanceCheckBwd = 0;
const int obstacleThresholdBwd = 40; // in cm


//Buzzer pin
const int buzzerPin = A2;

//speed
const int MAX_SPEED = 255;
const int BASE_SPEED = 180;
const int SLOW_SPEED = 80;

enum CarState {
  NOT_MOVING,
  MOVING_FORWARD,
  MOVING_BACKWARD
};
CarState currentState = NOT_MOVING;

enum InputMode {
  WIFI,
  IR_REMOTE,
  NONE
};

InputMode currMode = NONE;
unsigned long lastModeCheck = 0;


void setup() {

  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(fwdTrigPin, OUTPUT);
  pinMode(fwdEchoPin, INPUT);

  pinMode(bwdTrigPin, OUTPUT);
  pinMode(bwdEchoPin, INPUT);

  pinMode(FWD_IR, INPUT);
  pinMode(BACK_IR, INPUT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH); // Assume active-low

  //mode pins
  pinMode(MODE_PIN_1, INPUT);
  pinMode(MODE_PIN_2, INPUT);

  // Start the receiver
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("ir initialized");

}

void setupServer(){
  // Start Wi-Fi Access Point
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    Serial.println("Failed to start AP");
    while (true);
  }

  IPAddress ip = WiFi.localIP();
  server.begin();
}

void loop() {
  
  checkAndSetupInputMode();

  if(currentState != NOT_MOVING) checkFloorDrop();
    
  if(currentState == MOVING_FORWARD) checkObstacle(obstacleThresholdFwd, lastDistanceCheckFwd, fwdTrigPin, fwdEchoPin);
  else if(currentState == MOVING_BACKWARD) checkObstacle(obstacleThresholdBwd, lastDistanceCheckBwd, bwdTrigPin, bwdEchoPin);
  
    
  if(currMode == WIFI) handleClient();
  else if(currMode == IR_REMOTE) handleIRRemote();
}

void checkAndSetupInputMode(){

  unsigned long now = millis();
  //check mode every 10 seconds
  if (lastModeCheck > 0 && now - lastModeCheck < 10000) return;
  lastModeCheck = now;
  //Serial.println(currMode);
  //Serial.println(isWifiMode());
  //Serial.println(isRemoteMode());
  //Serial.println("done");
  if(isWifiMode() && currMode != WIFI){
    //Serial.println("setting up wifi");
    currMode = WIFI;
    setupServer();
  }else if(isRemoteMode() && currMode != IR_REMOTE){
    Serial.println("setting up remote");
    if(currMode == WIFI){
      server.end();
      WiFi.end();
      //Serial.println("cloosed wifi");
    }
    currMode = IR_REMOTE;
  }else if(isNoneMode() && currMode != NONE){
    if(currMode == WIFI){
      server.end();
      WiFi.end();
      //Serial.println("cloosed wifi");
    }
    currMode = NONE;
  }
}

bool isWifiMode(){
  return digitalRead(MODE_PIN_1) == HIGH;
}

bool isNoneMode(){
  return digitalRead(MODE_PIN_2) == HIGH;
}

bool isRemoteMode(){
  return digitalRead(MODE_PIN_1) == LOW && digitalRead(MODE_PIN_2) == LOW;
}

void handleClient() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    client.println("HTTP/1.1 200 OK");

    if (request.indexOf("GET /FD") >= 0) moveForwardDetect();
    else if (request.indexOf("GET /BD") >= 0) moveBackwardDetect();
    else if (request.indexOf("GET /LR") >= 0) rotateLeft();
    else if (request.indexOf("GET /RR") >= 0) rotateRight();
    else if (request.indexOf("GET /F") >= 0) moveForward();
    else if (request.indexOf("GET /B") >= 0) moveBackward();
    else if (request.indexOf("GET /L") >= 0) turnLeft();
    else if (request.indexOf("GET /R") >= 0) turnRight();
    else if (request.indexOf("GET /S") >= 0) stopMotors();
    else {
      client.println("Content-Type: text/html");
      client.println();
      client.println(htmlPage);
    }

    client.stop();
   
  }
}

void handleIRRemote() {
  if (IrReceiver.decode()) {
    uint8_t command = IrReceiver.decodedIRData.command;
    //Serial.print("IR Command: 0x");
    //Serial.println(command, HEX);

    switch (command) {
      case 0x40: moveForward(); break;     // Up arrow
      case 0x41: moveBackward(); break;    // Down arrow
      case 0x7: turnLeft(); break;        // Left arrow
      case 0x6: turnRight(); break;       // Right arrow
      case 0x44: stopMotors(); break;      // OK button
      case 0x3: rotateLeft(); break;      // - button
      case 0x2: rotateRight(); break;     // + button
      default: break;
    }

    IrReceiver.resume();
}
}


void checkObstacle(int obstacleThreshold, unsigned long &lastDistanceCheck, int trigPin, int echoPin) {
  unsigned long now = millis();
  if (now - lastDistanceCheck < distanceInterval) return;
  lastDistanceCheck = now;

  long distance = getDistance(trigPin, echoPin);

  // Consider -1 as invalid reading (timeout)
  if (distance != -1 && distance <= obstacleThreshold) {
    stopMotors();
    triggerBuzzer();
  }
}

void triggerBuzzer(){
  digitalWrite(buzzerPin, LOW);  // ON (for active-low)
  delay(500);
  digitalWrite(buzzerPin, HIGH); // OFF
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout = ~3.4m
  if (duration == 0) return -1; // timeout or no echo

  long distance = duration * 0.034 / 2;
  return distance;
}

void checkFloorDrop() {

  int fwdVal = digitalRead(FWD_IR);
  int backtVal = digitalRead(BACK_IR);

  if(currentState == MOVING_FORWARD && fwdVal == HIGH){
    stopMotors();
    triggerBuzzer();
  }else if(currentState == MOVING_BACKWARD && backtVal == HIGH){
    stopMotors();
    triggerBuzzer();
  }
}

// Movement functions
void moveBackward() {

  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);

  currentState = MOVING_BACKWARD;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void moveForward() {

  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);

  currentState = MOVING_FORWARD;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
}

void turnLeft() {


  //read current value
  int valIN1 = digitalRead(IN1);
  int valIN2 = digitalRead(IN2);
  int valIN3 = digitalRead(IN3);
  int valIN4 = digitalRead(IN4);

  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(500);

  //resume previous state of motors
  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  digitalWrite(IN1, valIN1);
  digitalWrite(IN2, valIN2);
  digitalWrite(IN3, valIN3);
  digitalWrite(IN4, valIN4);

}

void turnRight() {

  //read current value
  int valIN1 = digitalRead(IN1);
  int valIN2 = digitalRead(IN2);
  int valIN3 = digitalRead(IN3);
  int valIN4 = digitalRead(IN4);
  
  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(500);

  //resume previous state of motors
  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  digitalWrite(IN1, valIN1);
  digitalWrite(IN2, valIN2);
  digitalWrite(IN3, valIN3);
  digitalWrite(IN4, valIN4);

}

void rotateLeft() {

  currentState = NOT_MOVING;

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void rotateRight() {

  currentState = NOT_MOVING;

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

}

void stopMotors() {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  currentState = NOT_MOVING;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
}

// Movement functions
void moveBackwardDetect() {

  analogWrite(ENA, SLOW_SPEED);
  analogWrite(ENB, SLOW_SPEED);

  currentState = MOVING_BACKWARD;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void moveForwardDetect() {

  analogWrite(ENA, SLOW_SPEED);
  analogWrite(ENB, SLOW_SPEED);

  currentState = MOVING_FORWARD;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
}
