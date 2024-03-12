#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>


// Define sensor pins
#define IR_SENSOR_PIN 2
#define MOISTURE_SENSOR_PIN A0
#define CAPACITIVE_SENSOR_PIN A1
#define INDUCTIVE_SENSOR_PIN A2
#define ULTRASONIC_WET_PIN_TRIG 3
#define ULTRASONIC_WET_PIN_ECHO 4
#define ULTRASONIC_NONMETAL_PIN_TRIG 5 
#define ULTRASONIC_NONMETAL_PIN_ECHO 6
#define ULTRASONIC_METAL_PIN_TRIG 7
#define ULTRASONIC_METAL_PIN_ECHO 8
#define LATCH_SERVO_PIN 9     
#define CYLINDER_SERVO_PIN 10
#define BT_RX_PIN 11  // Connect Bluetooth RX to pin 11
#define BT_TX_PIN 12  // Connect Bluetooth TX to pin 12

SoftwareSerial bluetoothSerial(BT_RX_PIN, BT_TX_PIN);
// Define servo motor angles
#define ANGLE_WET 0
#define ANGLE_NONMETAL 90
#define ANGLE_METAL 180

// Defining servo motors
Servo cylinderServo;
Servo latchServo;

// Define LCD module parameters
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);     

// Define ultrasonic sensor variables
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters)
#define TIMEOUT_US 5000   // Timeout in microseconds

// Define maximum and minimum distances for each compartment
#define WET_MAX_DISTANCE 40 // Updated maximum distance for wet waste compartment
#define WET_MIN_DISTANCE 0  // Minimum distance for wet waste compartment
#define NONMETAL_MAX_DISTANCE 40 // Updated maximum distance for non-metal waste compartment
#define NONMETAL_MIN_DISTANCE 0  // Minimum distance for non-metal waste compartment
#define METAL_MAX_DISTANCE 40 // Updated maximum distance for metal waste compartment
#define METAL_MIN_DISTANCE 0  // Minimum distance for metal waste compartment

// Ethernet shield MAC address
/*byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// IP address of your Express.js server
IPAddress server(192, 168, 1, 5);
*/
//DEFINE FUNCTIONS

int mapToPercentage(long value, long max, long min) {
  return map(value, min, max, 100, 0);
}

unsigned long lastScrollTime = 0;
unsigned long scrollInterval = 200;  // Adjust the interval based on your desired scrolling speed





void scrollDisplay() {
  // Scroll the lines to the left
  for (int i = 0; i < LCD_COLUMNS; i++) {
    lcd.scrollDisplayLeft();
    delay(230);  // Adjust the delay to control scrolling speed
  }
}

void openLatch() {
  latchServo.write(180); // Assuming 180 is open position
  delay(1000); // Wait for the latch to open
}

void closeLatch() {
  latchServo.write(0); // Assuming 0 is closed position
  delay(1000); // Wait for the latch to close
}

void sortWetWaste() {
  cylinderServo.write(ANGLE_WET);
  delay(1000); // Wait for the servo to move to position
  openLatch();
  delay(2000); // Wait for the waste to go into the compartment
  closeLatch();
}

void sortNonMetalWaste() {
  cylinderServo.write(ANGLE_NONMETAL);
  delay(1000); // Wait for the servo to move to position
  openLatch();
  delay(2000); // Wait for the waste to go into the compartment
  closeLatch();
}

void sortMetalWaste() {
  cylinderServo.write(ANGLE_METAL);
  delay(1000); // Wait for the servo to move to position
  openLatch();
  delay(2000); // Wait for the waste to go into the compartment
  closeLatch();
}

void sendNonMetalPercentage(int percentage) {
  bluetoothSerial.print("NonMetalPercentage:");
  bluetoothSerial.println(percentage);
}

long readUltrasonicSensor(int trigPin, int echoPin) {
  // Send one pulse to the Trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pulse
  long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);

  // Calculate distance in cm
  long distance = duration * 0.034 / 2;

  return distance;
}




void setup() {
  // Initialize servo motors
  cylinderServo.attach(CYLINDER_SERVO_PIN);
  latchServo.attach(LATCH_SERVO_PIN);

  // Initialize sensor pins
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(MOISTURE_SENSOR_PIN, INPUT);
  pinMode(CAPACITIVE_SENSOR_PIN, INPUT);
  pinMode(INDUCTIVE_SENSOR_PIN, INPUT);
  
  //initialize ultrasonic pins
  pinMode(ULTRASONIC_WET_PIN_TRIG, OUTPUT);
  pinMode(ULTRASONIC_WET_PIN_ECHO, INPUT);
  pinMode(ULTRASONIC_NONMETAL_PIN_TRIG, OUTPUT);
  pinMode(ULTRASONIC_NONMETAL_PIN_ECHO, INPUT);
  pinMode(ULTRASONIC_METAL_PIN_TRIG, OUTPUT);
  pinMode(ULTRASONIC_METAL_PIN_ECHO, INPUT);

  // Initialize LCD module
  lcd.init();
  lcd.backlight();

  // Initialize serial communication
  bluetoothSerial.begin(9600);
  Serial.begin(9600);
}

void sendBluetoothData(const char* sensorType, int percentage) {
  // Send data over Bluetooth to the paired device
  bluetoothSerial.print(sensorType);
  bluetoothSerial.print(":");
  bluetoothSerial.println(percentage);
}


void loop() {
  // Read sensor values
  int irValue = digitalRead(IR_SENSOR_PIN);
  int moistureValue = analogRead(MOISTURE_SENSOR_PIN);
  int capacitiveValue = analogRead(CAPACITIVE_SENSOR_PIN);
  int inductiveValue = analogRead(INDUCTIVE_SENSOR_PIN);

  // Check if an object is detected by the IR sensor
  if (irValue == LOW) {
      // Sort the waste based on sensor readings
      
      if (moistureValue < 500) {
          delay(3000);
          sortWetWaste();
      } else if (capacitiveValue > 550 && inductiveValue < 350) {
          sortMetalWaste();
          Serial.println("it is metal");
          delay(1000);
      } else {  // Corrected the syntax here
          sortNonMetalWaste();
      } 
  } else {
      // If no object is detected, do not move the latch servo
      closeLatch(); // Ensure the latch is closed
  }

  // Read ultrasonic sensor distances
  long wetDistance = readUltrasonicSensor(ULTRASONIC_WET_PIN_TRIG, ULTRASONIC_WET_PIN_ECHO);
  long nonMetalDistance = readUltrasonicSensor(ULTRASONIC_NONMETAL_PIN_TRIG, ULTRASONIC_NONMETAL_PIN_ECHO);
  long metalDistance = readUltrasonicSensor(ULTRASONIC_METAL_PIN_TRIG, ULTRASONIC_METAL_PIN_ECHO);

  // Convert distances to percentage values
  if (wetDistance >= 45 || nonMetalDistance >= 45 || metalDistance >= 45) {
    Serial.println("No object detected");
  } else {
    // Convert distances to percentage values
    int wetPercentage = mapToPercentage(wetDistance, WET_MAX_DISTANCE, WET_MIN_DISTANCE);
    int nonMetalPercentage = mapToPercentage(nonMetalDistance, NONMETAL_MAX_DISTANCE, NONMETAL_MIN_DISTANCE);
    int metalPercentage = mapToPercentage(metalDistance, METAL_MAX_DISTANCE, METAL_MIN_DISTANCE);



  // Display trash levels on LCD
  // Scroll the text to the left
  String line1 = "Wet: " + String(wetPercentage) + "% ";
  String line3 = "Metal: " + String(metalPercentage) + "% ";
  String line2 = "Non-Metal: " + String(nonMetalPercentage) + "% ";
  

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  lcd.setCursor(0, 2);
  lcd.print(line3);

  // Check if it's time to scroll
  if (millis() - lastScrollTime >= scrollInterval) {
    scrollDisplay();
    lastScrollTime = millis();  // Reset the timer
  }
  sendNonMetalPercentage(nonMetalPercentage);
  


  Serial.print("Ir value:");
  Serial.println(irValue);
  Serial.print("Moisture: ");
  Serial.println(moistureValue);
  Serial.print("Capacitive: ");
  Serial.println(capacitiveValue);
  Serial.print("Inductive: ");
  Serial.println(inductiveValue);

  // Print ultrasonic sensor percentage waste levels
  Serial.print("Ultrasonic Wet: ");
  Serial.print(wetPercentage);
  Serial.println("%");

  Serial.print("Ultrasonic Non-Metal: ");
  Serial.print(nonMetalPercentage);
  Serial.println("%");

  Serial.print("Ultrasonic Metal: ");
  Serial.print(metalPercentage);
  Serial.println("%");

  sendBluetoothData("wet", wetPercentage);
  sendBluetoothData("nonMetal", nonMetalPercentage);
  sendBluetoothData("metal", metalPercentage);

  delay(10); 

  }
 
}

//i added something here