#include <ArduinoBLE.h>
#include <EduIntro.h>

const int BUZZER = 6; // The pin of the buzzer
const int DHT11_PIN = 8; // The pin of the DHT11 temperature and humidity sensor
const int PUMP = 5; // The pin of the transistor that will let the current (the ground) pass through the pump
const int TILT_BALL = 10; // The pin of the tilt ball switch that detects the orientation of the robot
const int MOTOR_LEFT = 13; // The pin of the left motor that is driven by a transistor
const int MOTOR_RIGHT = 12;
const int ECHO = 4;     // Echo input pin
const int TRIGGER = 2;  // Trigger output pin

// Required variables are defined
int maximumRange = 300; // cm
int minimumRange = 2; // cm
long distance;
long duration;
int time_turn_left = 2000; // Second to activate the righ motor in order to turn left for the robot (estimation)
bool fall = false; // Boolean value to know if the robot has fallen


DHT11 dht11(DHT11_PIN); // Init the DHT11 sensor read


// Define BLE service and characteristics
BLEService controlService("12345678-1234-5678-1234-567812345678");
BLEStringCharacteristic commandCharacteristic("87654321-4321-6789-4321-678987654321", BLERead | BLEWrite, 20);
BLEStringCharacteristic statusCharacteristic("ABCD1234-5678-4321-6789-1234567890AB", BLERead | BLENotify, 50);


void setup() {
  pinMode(BUZZER, OUTPUT); // Init the BUZZER pin as an output
  pinMode(PUMP, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TILT_BALL, INPUT);
  digitalWrite(TILT_BALL, HIGH); // Turn on the built-in pull-up resistor
  Serial.begin(9600); // Init data rate in bits per second (baud) for serial data transmission for communicating with Serial Monitor

  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set BLE local name and advertise service
  BLE.setLocalName("FirefighterRobot");
  BLE.setAdvertisedService(controlService);
  
  // Add the characteristics to the service
  controlService.addCharacteristic(commandCharacteristic);
  controlService.addCharacteristic(statusCharacteristic);
  
  // Add the service
  BLE.addService(controlService);
  
  // Start advertising
  BLE.advertise();

  Serial.println("BLE Firefighter Robot Service");
  
}

void loop() {
  Bluetooth_management();
  Read_Ultrasonic(0);
  if (!fall){
    Read_Humidity_And_Temperature(0); // 0 Because we just want to read in the serial monitor, not send to bluetooth connection
  }
  Read_Orientation(0);
  //Go_forward();
}

void Move_Forward(){
  digitalWrite(MOTOR_RIGHT, HIGH);
  digitalWrite(MOTOR_LEFT, HIGH);
}

void Move_Left(){
  digitalWrite(MOTOR_RIGHT, HIGH);
  digitalWrite(MOTOR_LEFT, LOW);
  delay(time_turn_left); // Time estimation to go left
  Move_Stop();
}

void Move_Turn_Around(){
  digitalWrite(MOTOR_RIGHT, HIGH);
  digitalWrite(MOTOR_LEFT, LOW);
  delay(2*time_turn_left); // Double the time to go left
  Move_Stop();
}

void Move_Stop(){
  digitalWrite(MOTOR_RIGHT, LOW);
  digitalWrite(MOTOR_LEFT, LOW);
}



void Bluetooth_management(){
  // Listen for BLE peripherals to connect
  BLEDevice central = BLE.central();

  // If a central is connected to peripheral
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // Check if there's a command received
    if (commandCharacteristic.written()) {
      String command = commandCharacteristic.value();
      Serial.print("Received command: ");
      Serial.println(command);

      // Process the command
      if (command == "START_SIREN") {
        Firefighter_Siren();
      } else if (command == "STOP_SIREN") {
        noTone(BUZZER);
      } else if (command == "START_PUMP") {
        digitalWrite(PUMP, HIGH);
      } else if (command == "STOP_PUMP") {
        digitalWrite(PUMP, LOW);
      } else if (command == "READ_SENSOR") {
        Read_Humidity_And_Temperature(1);
      } else if (command == "READ_DISTANCE") {
        Read_Ultrasonic(1);
      } else if (command == "CHECK_ORIENTATION") {
        Read_Orientation(1);
      } else {
        statusCharacteristic.writeValue("Error 400 Bad Request");
      }
    }

    delay(100);
  }
}

void Police_Siren() {
  for (int hz = 440; hz < 1000; hz++) { // Increase the frequency 
    tone(BUZZER, hz, 50);
    delay(5);
  }

  for (int hz = 1000; hz > 440; hz--) { // Decrease the frequency 
    tone(BUZZER, hz, 50);
    delay(5);
  }
}

void Firefighter_Siren() {
  int length = 1000; // Length of one tone in milliseconds
  tone(BUZZER, 430);
  delay(length);
  tone(BUZZER, 490);
  delay(length);
}

void Read_Humidity_And_Temperature(bool send) {
  dht11.update();

  float h = dht11.readHumidity();
  float t = dht11.readCelsius();

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %");
  
  Serial.print("\t\tTemperature: ");
  Serial.print(t);
  Serial.println(" °C");

  // Send the sensor values over BLE
  String statusMessage = " Humidity: " + String(h) + " %, Temperature: " + String(t) + " °C";
  if (send){statusCharacteristic.writeValue(statusMessage);}
  
  if (t > 40) {
    Firefighter_Siren();
    digitalWrite(PUMP, HIGH);
    Move_Stop();
    Serial.println("Fire detected !");
  } else {
    digitalWrite(PUMP, LOW);
    noTone(BUZZER); // Reset the siren
  }

  delay(1000);
}


void Read_Ultrasonic(bool send){
  digitalWrite(TRIGGER, HIGH); // Send signal to trigger pin
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH); // 
  distance = duration / 58.2;
  String statusMessage = "";

  if (fall){
    Move_Stop();
  }
  // Check whether the measured value is within the permissible distance
  else if (distance <= minimumRange){
      statusMessage = "Distance outside the measuring range, too close to obstacle";
      Move_Turn_Around();
  }
  else if (distance >= maximumRange) {
    statusMessage = "Distance outside the measuring range, nothing in front";
    Move_Forward();
  }

  else {
    // The calculated distance is output in the serial output
    statusMessage = "The distance to obsctacle is: " + String(distance) + " cm";

    if (distance < 20.0){
      statusMessage = statusMessage + "; Too close to obsctacle, moving to the left !";
      Move_Left();
    }
    else {
      Move_Forward();
    }
  }

  Serial.println(statusMessage);
  if (send){statusCharacteristic.writeValue(" " + statusMessage);}
  // Pause between the individual measurements
  delay(500);
}


void Read_Orientation(bool send) {
  String statusMessage = "";
  if (digitalRead(TILT_BALL)) { // It means that the robot has fallen 
    fall = true;
  } 

  if (fall){
    statusMessage = "Orientation: Robot fell";
  } else {
    statusMessage = "Orientation: Normal";
  }

  Serial.println(statusMessage);
  Serial.println("-----------------------------------");

  if (send){statusCharacteristic.writeValue(" " + statusMessage);}
}

