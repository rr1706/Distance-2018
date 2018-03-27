/**
 * ==============================================
 * ==   This code is used to manage the sensors 
 * ==   and should not be changed unless you 
 * ==   REALLY know what you are doing.
 * ==============================================
 */

#define   VL53L0X_ADDR                      0b0101001
#define   VL53L0X_RANGE_START               0x00
#define   VL53L0X_SYS_INTR_CLR              0x0B
#define   VL53L0X_RESULT_INTR_STATUS        0x13
#define   VL53L0X_RESULT_RANGE_STATUS       0x14
#define   VL53L0X_MODEL_ID                  0xC0
#define   VL53L0X_REVISION_ID               0xC2
#define   VL53L0X_MSRC_CONFIG_CONTROL       0x60
#define   VL53L0X_SYSTEM_SEQUENCE_CONFIG    0x01


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
  //I2C_write_byte(wire, VL53L0X_ADDR, VL53L0X_SYS_INTR_CLR, 0x01);  // clear any interrupt state
  return range_mm;
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

