#include <Wire.h>
#include <FastLED.h>
#include "SoftwareWire.h"

#define NUM_LEDS 144
#define CLOCK_PIN 13
#define DATA_PIN 12

#define NUM_SENSORS 5

#define MIN_GRIP_VALUE 22
#define MAX_GRIP_VALUE 130

#define MIN_CLOSE_VALUE 22
#define MAX_CLOSE_VALUE 250

#define MIN_CUBE_DISTANCE 30
#define NEAR_CUBE_DISTANCE 250
#define MAX_CUBE_DISTANCE 330

#define GOOD_SENSOR_DELAY 5

template< typename T, size_t N > size_t ArraySize (T (&) [N]){ return N; }

String goodValues[] = {"12","2","3","23","13","24","34","124","134"};
String actionableValues[] = {"1","4","14"};
String wallValues[] = {"123","234","1234"};

CRGB leds[NUM_LEDS] = {CRGB::Blue};

int qq = 0;
int ledDirection = 1;

// Create an array of Software I2C interfaces, one for each sensor.
SoftwareWire wires[NUM_SENSORS] = {
  SoftwareWire(2,3),
  SoftwareWire(4,5),
  SoftwareWire(6,7),
  SoftwareWire(8,9),
  SoftwareWire(10,11)
};

short distances[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte  readIndex = 0;
int currentGoodSensorCount = GOOD_SENSOR_DELAY;

void setup() {

  // Wait for serial port to come online.
  while(!Serial);
  Serial.print("Number of good items: ");Serial.println(ArraySize(goodValues));

  // Start the i2c interface as slave at address 8.
  Wire.begin(8);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(115200);

  // Initialize the range finders...
  for (int i = 0; i < NUM_SENSORS; i++) {
    initSensor(wires[i]);
  }

  // Set up analog pins as outputs to send signals to RoboRIO.
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
//  pinMode(A4, OUTPUT);
//  pinMode(A5, OUTPUT);

  // LEDs
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR, DATA_RATE_MHZ(12)>(leds, NUM_LEDS);
  // The Brightness maxium is 255
  FastLED.setBrightness(10);
  for (int i = 0; i < NUM_LEDS; i++) {
   // leds[i] = CRGB::Blue;
  }

}


/**
 * Hardware I2C slave function to send data to the I2C master on request.
 */
void requestEvent() {
  short data = distances[readIndex];
  Wire.write((data >> 8) & 0xff);
  Wire.write(data        & 0xff);
}

/**
 * Hardware I2C slave function to receive data from the I2C master.
 */
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
  }
  readIndex = Wire.read();    // receive byte as an integer
}


/**
 * Main loop.
 */
void loop() {

  // read inputs
  for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = processSensor(wires[i], distances[i]);
  }

  // Look for good states
  // Signal cube in gripper if we have a good value between 1.5 and 6 inches.
  bool hasCubeHigh = (distances[4] > MIN_GRIP_VALUE) && (distances[4] < MAX_GRIP_VALUE);
  bool hasCubeLow = (distances[4] > MIN_CLOSE_VALUE) && (distances[4] < MAX_CLOSE_VALUE);
  
  bool cubeInPosition = checkForGoodCubes(distances, NEAR_CUBE_DISTANCE) && !checkForActionableCubes(distances, MAX_CUBE_DISTANCE);
  bool cubeActionable = checkForActionableCubes(distances, NEAR_CUBE_DISTANCE);
  bool againstWall = checkForWall(distances, NEAR_CUBE_DISTANCE);

  // Add delay to prevent false positives.
  if (!cubeInPosition) {
    currentGoodSensorCount = GOOD_SENSOR_DELAY;
  } else {
    currentGoodSensorCount = max(0, currentGoodSensorCount - 1);
  }

  // Set outputs
  // A0 = active-low gripper switch.
  digitalWrite(A0, !hasCubeHigh);

  // A1 --> Cube in good position
  digitalWrite(A1, cubeInPosition && (currentGoodSensorCount == 0));

  // A2 --> We think we're up against a wall.
  digitalWrite(A2, againstWall);

  // A3 --> Cube in poor position but driver may act if desired
  digitalWrite(A3, hasCubeLow);

// Debugging output
  if (true) {
  
    for (int i = 0; i < 8; i++) {
      Serial.print(distances[i]);
      Serial.print("\t");
      //Serial.print(distances[i] / 25.4);
    }
  
    Serial.print(hasCubeHigh ?      "Got Cube         " : "No Cube          ");
    Serial.print(hasCubeLow ?       "Cube Close       " : "Cube Not Close   ");
    Serial.print(cubeInPosition ?   "Ready            " : "Not Ready        ");
    Serial.println(cubeActionable ? "Strafe" : "");
  }

/*
  int q = distances[4] / 20; //(distances[4] * NUM_LEDS) / MIN_GRIP_VALUE;
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = i > q ? CRGB::Blue : CRGB::Red;
  }
  */

  leds[0] = distances[0] > MIN_CUBE_DISTANCE && distances[0] < NEAR_CUBE_DISTANCE ? CRGB::Blue : CRGB::Red;
  leds[1] = distances[1] > MIN_CUBE_DISTANCE && distances[1] < NEAR_CUBE_DISTANCE ? CRGB::Blue : CRGB::Red;
  leds[2] = distances[2] > MIN_CUBE_DISTANCE && distances[2] < NEAR_CUBE_DISTANCE ? CRGB::Blue : CRGB::Red;
  leds[3] = distances[3] > MIN_CUBE_DISTANCE && distances[3] < NEAR_CUBE_DISTANCE ? CRGB::Blue : CRGB::Red;

  leds[5] = cubeInPosition ? CRGB::Blue : CRGB::Red;
  leds[6] = cubeActionable ? CRGB::Blue : CRGB::Red;

  leds[8] = hasCubeLow ? CRGB::Blue : CRGB::Red;
  leds[9] = hasCubeHigh ? CRGB::Blue : CRGB::Red;


/*
  qq = max(14, (qq + ledDirection) % NUM_LEDS);
  if (qq == 14) {
    ledDirection = 1;
  }
  if (qq == NUM_LEDS - 1) {
    ledDirection = -1;
  }
  FastLED.setBrightness(qq / 3);
  */

  int strandColor = CRGB::Blue;
  int topMark = 11;
  if (hasCubeHigh) {
    topMark = NUM_LEDS;
  } else if (hasCubeLow) {
    topMark = 2 * NUM_LEDS / 3;
  } else if (cubeInPosition) {
    topMark = NUM_LEDS / 2;
  } else if (cubeActionable) {
    topMark = NUM_LEDS / 4;
  } else {
    topMark = NUM_LEDS;
    strandColor = CRGB::Green;
  }
  for (int i = 11; i < NUM_LEDS; i++) {
    leds[i] = (i < topMark) ? strandColor : CRGB::Black;
  }
  FastLED.show();
  
  delay(50);
}

int processSensor(SoftwareWire &wire, int oldValue) {
  int range_mm = read_range_mm(wire);
  if (range_mm == 0) {
    initSensor(wire);
  }
  return range_mm;
}

bool checkForWall(short distances[], int threshold) {
  String currentState = String("");
  for (int i = 0; i < 4; i++) {
    if (distances[i] > MIN_CUBE_DISTANCE && distances[i] < threshold) {
      currentState += (char)('1' + i);
    }
  }
  for (int i = 0; i < ArraySize(wallValues); i++) {
    if (currentState.equals(wallValues[i])) {
      return true;
    }
  }
  return false;
   
}

bool checkForGoodCubes(short distances[], int threshold) {
  String currentState = String("");
  for (int i = 0; i < 4; i++) {
    if (distances[i] > MIN_CUBE_DISTANCE && distances[i] < threshold) {
      currentState += (char)('1' + i);
    }
  }
  for (int i = 0; i < ArraySize(goodValues); i++) {
    if (currentState.equals(goodValues[i])) {
      return true;
    }
  }
  return false;
}

bool checkForActionableCubes(short distances[], int threshold) {
  String currentState = String("");
  for (int i = 0; i < 4; i++) {
    if (distances[i] > MIN_CUBE_DISTANCE && distances[i] < threshold) {
      currentState += (char)('1' + i);
    }
  }
  for (int i = 0; i < ArraySize(actionableValues); i++) {
    if (currentState.equals(actionableValues[i])) {
      return true;
    }
  }
  return false;
}


