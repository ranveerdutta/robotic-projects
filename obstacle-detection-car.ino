#include <WiFiS3.h>
#include <WiFiServer.h>

// Wi-Fi Access Point credentials
char ssid[] = "ArduinoCar";     
char pass[] = "12345678";       

WiFiServer server(80);

// Motor driver pins (adjust as per your wiring)
const int ENA = 5;   // PWM - Left wheels
const int ENB = 6;   // PWM - Right wheels
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 10;


//IR
const int FWD_IR = A0;
const int BACK_IR = A1;
const int IR_VAL_THRESHOLD = 500;


// Ultrasonic sensor pins
const int trigPin = 2;
const int echoPin = 3;
unsigned long lastDistanceCheck = 0;
const unsigned long distanceInterval = 100; // check every 100 ms
const int obstacleThreshold = 20; // in cm

enum CarState {
  NOT_MOVING,
  MOVING_FORWARD,
  MOVING_BACKWARD
};

CarState currentState = NOT_MOVING;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(FWD_IR, INPUT);
  pinMode(BACK_IR, INPUT);


  // Start Wi-Fi Access Point
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    Serial.println("Failed to start AP");
    while (true);
  }

  IPAddress ip = WiFi.localIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  server.begin();

}

void loop() {
  if(currentState != NOT_MOVING){
    checkFloorDrop();
    checkObstacle();
  }
    
  handleClient();
}

void handleClient() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    String request = client.readStringUntil('\r');
    client.flush();
    Serial.println(request);

    if (request.indexOf("GET /F") >= 0) moveForward();
    else if (request.indexOf("GET /B") >= 0) moveBackward();
    else if (request.indexOf("GET /LR") >= 0) rotateLeft();
    else if (request.indexOf("GET /RR") >= 0) rotateRight();
    else if (request.indexOf("GET /L") >= 0) turnLeft();
    else if (request.indexOf("GET /R") >= 0) turnRight();
    else if (request.indexOf("GET /S") >= 0) stopMotors();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>");
    client.println("<style>");
    client.println("body { font-family: Arial, sans-serif; background: #111; color: white; margin: 0; text-align: center; }");
    client.println("h1 { background: #222; padding: 15px; margin: 0; }");
    client.println(".controller { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-gap: 10px; padding: 20px; max-width: 400px; margin: auto; }");
    client.println(".row { display: flex; justify-content: center; margin-bottom: 10px; }");
    client.println(".btn { width: 70px; height: 70px; font-size: 20px; font-weight: bold; border: none; border-radius: 15px; margin: 5px; cursor: pointer; transition: 0.2s; }");
    client.println(".btn:active { transform: scale(0.95); }");
    client.println(".forward { background: #4CAF50; }");
    client.println(".backward { background: #555; }");
    client.println(".left { background: #2196F3; }");
    client.println(".right { background: #2196F3; }");
    client.println(".stop { background: #f44336; }");
    client.println(".rotate { background: #FF9800; }");
    client.println(".sweep { background: #9C27B0; }");
    client.println("</style></head><body>");
    client.println("<h1>WIFI RC Car Controller</h1>");

    client.println("<div class='controller'>");
    // Top row - Forward
    client.println("<div></div>");
    client.println("<a href='/F'><button class='btn forward'>Fwd</button></a>");
    client.println("<div></div>");

    // Middle row - Left, Stop, Right
    client.println("<a href='/L'><button class='btn left'>Left</button></a>");
    client.println("<a href='/S'><button class='btn stop'>Stop</button></a>");
    client.println("<a href='/R'><button class='btn right'>Right</button></a>");

    // Bottom row - Backward
    client.println("<div></div>");
    client.println("<a href='/B'><button class='btn backward'>Back</button></a>");
    client.println("<div></div>");
    client.println("</div>");

    // Action buttons row
    client.println("<div class='row'>");
    client.println("<a href='/LR'><button class='btn rotate'>Left Rotate</button></a>");
    client.println("<a href='/RR'><button class='btn rotate'>Right Rotate</button></a>");
    client.println("</div>");

    client.println("</body></html>");


    //delay(1);
    client.stop();
    Serial.println("Client disconnected");
  }
}

void checkObstacle() {
  unsigned long now = millis();
  if (now - lastDistanceCheck < distanceInterval) return;
  lastDistanceCheck = now;

  long distance = getDistance();

  // Consider -1 as invalid reading (timeout)
  if (distance != -1 && distance <= obstacleThreshold) {
    stopMotors();
  }
}

long getDistance() {
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

  int fwdVal = analogRead(FWD_IR);
  int backtVal = analogRead(BACK_IR);

  if(currentState == MOVING_FORWARD && fwdVal > IR_VAL_THRESHOLD){
    stopMotors();
  }else if(currentState == MOVING_BACKWARD && backtVal > IR_VAL_THRESHOLD){
    stopMotors();
  }
}

// Movement functions
void moveBackward() {

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  currentState = MOVING_BACKWARD;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void moveForward() {

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

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

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(500);

  //resume previous state of motors
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
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
  
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(500);

  //resume previous state of motors
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, valIN1);
  digitalWrite(IN2, valIN2);
  digitalWrite(IN3, valIN3);
  digitalWrite(IN4, valIN4);

}

void rotateLeft() {

  currentState = NOT_MOVING;

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void rotateRight() {

  currentState = NOT_MOVING;

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
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
