#include <Wire.h>
#include "SoftwareWire.h"

#define   VL53L0X_ADDR                      0b0101001
#define   VL53L0X_RANGE_START               0x00
#define   VL53L0X_SYS_INTR_CLR              0x0B
#define   VL53L0X_RESULT_INTR_STATUS        0x13
#define   VL53L0X_RESULT_RANGE_STATUS       0x14
#define   VL53L0X_MODEL_ID                  0xC0
#define   VL53L0X_REVISION_ID               0xC2
#define   VL53L0X_MSRC_CONFIG_CONTROL       0x60
#define   VL53L0X_SYSTEM_SEQUENCE_CONFIG    0x01

#define NUM_SENSORS 5

#define MIN_GRIP_VALUE 35
#define MAX_GRIP_VALUE 130

#define MIN_CLOSE_VALUE 35
#define MAX_CLOSE_VALUE 250

#define MIN_CUBE_DISTANCE 35
#define NEAR_CUBE_DISTANCE 250
#define MAX_CUBE_DISTANCE 330

char* goodValues[] = {"12","2","3","23","13","24","34","124","134"};
char* actionableValues[] = {"123","234","1234","1","4","14"};

// Create an array of Software I2C interfaces, one for each sensor.
SoftwareWire wires[NUM_SENSORS] = {
  SoftwareWire(2,3),
  SoftwareWire(4,5),
  SoftwareWire(6,7),
  SoftwareWire(8,9),
  SoftwareWire(10,11)
};

#define numGood (sizeof(goodValues)/sizeof(char *))
#define numActionable (sizeof(actionableValues)/sizeof(char *))

short distances[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte  readIndex = 0;

void setup() {
  // put your setup code here, to run once:

  // Wait for serial port to come online.
  while(!Serial);

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
}

void initSensor(SoftwareWire &wire) {

  // Start up Virtual I2C port.
  wire.begin();
  
  // Set 2.8V mode
  byte workingVoltage = I2C_read_byte(wire, VL53L0X_ADDR, 0x89);
  I2C_write_byte(wire, VL53L0X_ADDR, 0x89, workingVoltage | 0x01);

  // Set I2C Standard Mode
  I2C_write_byte(wire, VL53L0X_ADDR, 0x88, 0);

  // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  I2C_write_byte(wire, VL53L0X_ADDR, VL53L0X_MSRC_CONFIG_CONTROL, 
                 I2C_read_byte(wire, VL53L0X_ADDR, VL53L0X_MSRC_CONFIG_CONTROL) | 0x12);
  setSignalRateLimit(wire, 0.25);
  I2C_write_byte(wire, VL53L0X_ADDR, VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    
  byte model_num = I2C_read_byte(wire, VL53L0X_ADDR, VL53L0X_MODEL_ID);
  byte revision_id = I2C_read_byte(wire, VL53L0X_ADDR, VL53L0X_REVISION_ID);

  Serial.print("found device model id: "); Serial.println(model_num);
  Serial.print("found device revision id: "); Serial.println(revision_id);

  // continuous back-to-back mode
  I2C_write_byte(wire, VL53L0X_ADDR, VL53L0X_RANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  
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

  // Set outputs
  // A0 = active-low gripper switch.
  digitalWrite(A0, !hasCubeHigh);

  // A1 --> Cube in good position
  digitalWrite(A1, cubeInPosition);

  // A2 --> Cube in poor position but driver may act if desired
  digitalWrite(A2, cubeActionable);

  // A3 --> Cube in poor position but driver may act if desired
  digitalWrite(A3, hasCubeLow);

// Debugging output
  if (false) {
  
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

  delay(80);
}

int processSensor(SoftwareWire &wire, int oldValue) {
  //if (has_value(wire)) {
    int range_mm = read_range_mm(wire);
    return range_mm;
  //}
  return oldValue;
}

bool checkForGoodCubes(short distances[], int threshold) {
  String currentState = String("");
  for (int i = 0; i < 4; i++) {
    if (distances[i] > MIN_CUBE_DISTANCE && distances[i] < threshold) {
      currentState += (char)('1' + i);
    }
  }
  for (int i = 0; i < numGood; i++) {
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
  for (int i = 0; i < numActionable; i++) {
    if (currentState.equals(actionableValues[i])) {
      return true;
    }
  }
  return false;
}


/**
 * Ask the sensor if it has a good reading.  Not really used in continuous measurement mode.
 */
bool has_value(SoftwareWire &wire) {
  return 0 != (I2C_read_byte(wire, VL53L0X_ADDR, VL53L0X_RESULT_INTR_STATUS) & 0x07);
}

/**
 * Read the output from a sensor attached to the specified SoftwareWire software i2c port.
 * 
 * Returns the distance in mm.
 */
int read_range_mm(SoftwareWire &wire) {
  int range_mm = 0;

  // Read data.
  range_mm = I2C_read_word(wire, VL53L0X_ADDR, (VL53L0X_RESULT_RANGE_STATUS + 10));
  I2C_write_byte(wire, VL53L0X_ADDR, VL53L0X_SYS_INTR_CLR, 0x01);  // clear any interrupt state
  return range_mm;
}

/**
 * ==============================================
 * ==   This code is used to manage the sensors 
 * ==   and should not be changed unless you 
 * ==   REALLY know what you are doing.
 * ==============================================
 */

/**
 * Tell the sensor how often to have a reading ready.
 */
bool setSignalRateLimit(SoftwareWire &wire, float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  I2C_write_word(wire, VL53L0X_ADDR, 0x44, limit_Mcps * (1 << 7));
  return true;
}

int I2C_write_multi(SoftwareWire &wire, int address, int regstr, byte* data, int count) {
  wire.beginTransmission(address);
  wire.write(regstr);
  for (int i = 0; i < count; i++) {
    wire.write((uint8_t)data[i]);
  }
  wire.endTransmission();
  return 0;
}

int I2C_write_byte(SoftwareWire &wire, int address, int regstr, byte data) {
  return I2C_write_multi(wire, address,regstr, &data, 1);
}

int I2C_write_word(SoftwareWire &wire, int address, int regstr, unsigned int data) {
  wire.beginTransmission(address);
  wire.write(regstr);
  wire.write((data >> 8) & 0xff);
  wire.write(data & 0xff);
  wire.endTransmission();
  return 0;
}

int I2C_write_dword(SoftwareWire &wire, int address, int regstr, unsigned long data) {
  wire.beginTransmission(address);
  wire.write(regstr);
  wire.write((data >> 24) & 0xff);
  wire.write((data >> 16) & 0xff);
  wire.write((data >>  8) & 0xff);
  wire.write(data         & 0xff);
  wire.endTransmission();
  return 0;
}

int I2C_read_multi(SoftwareWire &wire, int address, int regstr, byte* data, int count) {
  wire.beginTransmission(address);
  wire.write(regstr);
  wire.endTransmission();
  wire.requestFrom(address, count);

  //Serial.print("Reading from device: "); Serial.println(address);

  for (int i = 0; i < count; i++) {
    data[i] = wire.read();
    //Serial.print("   "); Serial.println(data[i]);
  }
  return 0;
}

byte I2C_read_byte(SoftwareWire &wire, int address, int regstr) {
  byte data;
  if (I2C_read_multi(wire, address,regstr, &data, 1) == 0) {
    return data;
  }
  return -1;
}

int I2C_read_word(SoftwareWire &wire, int address, int regstr) {
  byte data[2];
  if (I2C_read_multi(wire, address,regstr, data, 2) == 0) {
    return data[0] << 8 | data[1];
  }
  return -1;
}

