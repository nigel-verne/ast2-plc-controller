#include <SPI.h>
#include <SD.h>
#include <P1AM.h>

/***************************************************************************************************
 *  Define all constant system parameters
 **************************************************************************************************/
// Declare PLC module slot numbers
const int P1_16CDR_MODULE_1 = 1;   // P1-16CDR digital input/output module
const int P1_04DAL_1_MODULE_1 = 2; // P1-04DAL-1 analog output current module
const int P1_08ADL_1_MODULE_1 = 3; // P1-08ADL-1   analog current module
const int P1_04RTD_MODULE_1 = 4;   // P1-04RTD module 1
const int P1_04RTD_MODULE_2 = 5;   // P1-04RTD module 2
const int P1_04RTD_MODULE_3 = 6;   // P1-04RTD module 3

// Declare ADC bit counts for the modules
const int THIRTEEN_BIT_ADC_COUNT = 8191;

// Declare RTD module configuration parameters
// const char P1_04RTD_CONFIG1[] = {0x40, 0x02, 0x60, 0x01, 0x20, 0x01, 0x80, 0x00};
// const char P1_04RTD_CONFIG2[] = {0x40, 0x02, 0x60, 0x01, 0x20, 0x01, 0x80, 0x00};
// const char P1_04RTD_CONFIG3[] = {0x40, 0x02, 0x60, 0x01, 0x20, 0x01, 0x80, 0x00}; // leave this one as is for now

// Configure RTD modules to read resistance directly
const char P1_04RTD_CONFIG1[] = {0x40, 0x02, 0x60, 0x01, 0x20, 0x06, 0x80, 0x00};
const char P1_04RTD_CONFIG2[] = {0x40, 0x02, 0x60, 0x01, 0x20, 0x06, 0x80, 0x00};
const char P1_04RTD_CONFIG3[] = {0x40, 0x02, 0x60, 0x01, 0x20, 0x06, 0x80, 0x01};

// Declare SD card parameters
const String LOG_FILE_NAME = "datalog.csv";
const int CHIP_SELECT_PIN = SDCARD_SS_PIN; // Pin used for selecting the SD card
const int SD_CARD_WRITE_INTERVAL = 1000;   // Time interval in milliseconds between SD card writes
const int SD_CARD_RECONNECT_INTERVAL = 10; // Time in seconds before attempting to reconnect to the SD card

// Declare solenoid vent parameters
const int XV762_VENT_PRESSURE = 400;   // Pressure at which XV762 opens (vent)
const int XV762_RESEAL_PRESSURE = 395; // Pressure at which XV762 closes (reseal)

const unsigned long VENTING_DURATION = 5000; // duration before venting is activated

// Declare PLC timing parameters
const unsigned long ACQUISITION_INTERVAL = 150; // Desired PLC loop time in milliseconds, minimum value for consistent loop timing is 40

const float TANK_VOLUME_L = 445.0; // tank volume in liters

// Define hydrogen lookup table properties
/*
const String LOOKUP_TABLE_FILE_NAME = "hydrogen.txt";
const int doubleSizeInBytes = 8; // Width of each ascii character in bytes
const int minimumTemperature = 30;
const int maximumTemperature = 400;
const int minimumPressure = 0;
const int maximumPressure = 500;
*/

// Define nitrogen lookup table properties
const String LOOKUP_TABLE_FILE_NAME = "nitrogen.txt";
const int minimumTemperature = 100;
const int maximumTemperature = 400;
const int minimumPressure = 0;
const int maximumPressure = 20;

const int numRows = maximumTemperature - minimumTemperature + 1;
const int numColumns = maximumPressure - minimumPressure + 1;

const int doubleSizeInBytes = 8; // size of a double (in Bytes)
/****************************************************************
 *  Set RTD Lookup Table Parameters
 ****************************************************************/
const String RTD_LOOKUP_TABLE = "PT111.bin";
const float minimumResistance = 3.5;
const float maximumResistance = 193.3;
const float resistanceStep = 0.1;
// double temperature;

/***************************************************************************************************
 *  Define all global variables
 **************************************************************************************************/
// Define global variables for all sensor values
float PT711;
float PT712;
double TT711;
double TT712;
double TT741;
double TT742;
double TT743;
double TT744;
float TT791;
float TT792;
float TT793;
float VG782;

// Declare Averages
const int averageItemCount = 25;

// declare variables for averaged pressure
float PTRUNNINGAVERAGE;
float PTArray[averageItemCount];
int PTArrayIndex = 0;

// declare variables for averaged temperature
float TTRUNNINGAVERAGE;
float TTArray[averageItemCount];
int TTArrayIndex = 0;

// Define global variables for visualizer commands
int solenoidCommand = 0;         // 0 or 1, controls XV762 valve
float parkerHanbayCommand = 0.0; // 0 to 100, controls Parker hanbay valve
int dragonHanbayCommand = 0;     // 0 to 100, controls Dragon hanbay valve
int ventEnableCommand = 0;       // 0 or 1, enables venting
float ventRateCommand = 0.0;     // 0.0 to 100.0 desired vent rate in g/s (0 to 15 kg/hr)

// Define global variables for various timing processes
unsigned long previousMillis = millis();         // Timestamp of the previous loop iteration
unsigned long sdCardPreviousMillis = millis();   // Timestamp for the previous SD card write operation
unsigned long startTimestampMillis = millis();   // Initial timestamp for measuring elapsed time
unsigned long elapsedTimestampMillis = millis(); // Elapsed time since startTimestampMillis
unsigned long MOPVentingStartTime = 0;           // Time that hard-coded MOP venting is activated
unsigned long MOPResealingStartTime = 0;         // Time that hard-coded MOP resealing is activated

// Declare remaining global variables
int sdCardReconnectAttemptsCounter = 0; // Counter for tracking SD card reconnection attempts
bool hardCodedVentingActivated = false; // Flag indicating whether hard-coded solenoid venting is active
int XV762Status = 0;                    // Flag indicating status of solenoid valve (0 is closed, 1 is open)

// declare controlled vent rate variables
float desiredHydrogenStored, actualHydrogenStored, initialHydrogenStored;
float desiredVentRate;
unsigned long ventInitiationTime;
const float ventCutoff = 6.0; // point at which the hanbay valve actually starts to vent

// Calculate valve step - valve takes 180000 ms to go from fully open to fully closed
float valveStep = 30.0 / (180000 / static_cast<float>(ACQUISITION_INTERVAL));

/***************************************************************************************************
 * @description Sets up the initial configuration of the PLC when it is turned on.
 *              Initializes Serial communication, PLC modules, and the SD card.
 *              Configures RTD modules and initializes the start time for SD card timestamps.
 **************************************************************************************************/
void setup()
{
  // Initialize Serial with a baud rate of 115200
  Serial.begin(115200);

  // Attempt to initialize modules, report timeout error if initialization fails
  if (!P1.init())
  {
    Serial.println("ERROR: Failure when initializing PLC modules. Check module connections and power cycle PLC.");
  }

  if (!initialize_sd_card())
  {
    Serial.println("ERROR: Failure initializing SD card. Insert or reformat SD card and power cycle PLC.");
  }

  // Configure RTD Modules
  bool RTD1ModStatus = P1.configureModule(P1_04RTD_CONFIG1, P1_04RTD_MODULE_1);
  bool RTD2ModStatus = P1.configureModule(P1_04RTD_CONFIG2, P1_04RTD_MODULE_2);
  bool RTD3ModStatus = P1.configureModule(P1_04RTD_CONFIG3, P1_04RTD_MODULE_3);

  // Check if module initialization failed
  if (!RTD1ModStatus || !RTD2ModStatus || !RTD3ModStatus)
  {
    Serial.println("ERROR: Failure configuring RTD modules. Check module connections and power cycle PLC.");
  }

  // Initialize start time for SD Card Timestamps
  startTimestampMillis = millis();
}

/***************************************************************************************************
 * @description Main loop of the PLC program. Reads sensor values, actuates valves,
 *              communicates with the visualizer, and writes data to the SD card.
 **************************************************************************************************/
void loop()
{
  // Get the current time in milliseconds
  unsigned long currentMillis = millis();

  // If at least ACQUISITION_INTERVAL milliseconds have elapsed, run code
  if (currentMillis - previousMillis >= ACQUISITION_INTERVAL)
  {
    // Reset previousMillis to the current time
    previousMillis = currentMillis;

    // Read the serial commands, if any are available
    read_serial_commands();

    // Simulate random sensor values - uncomment for testing

    // simulate_random_sensor_values();

    // Read actual sensor values - comment out for testing
    read_sensor_values();

    // Actuate parker valve, including logic for controlled vent rate
    actuate_parker_valve();

    // Actuate dragon valve (visualizer control only)
    actuate_dragon_valve();

    // Actuate solenoid valve (including logic for MOP venting)
    actuate_solenoid_valve();

    // Send system info to the visualizer via the serial
    write_to_serial();

    // If at least SD_CARD_WRITE_INTERVAL milliseconds have elapsed, write to the SD card
    if (currentMillis - sdCardPreviousMillis >= SD_CARD_WRITE_INTERVAL)
    {
      // write_to_sd_card();
      sdCardPreviousMillis = currentMillis;
    }

    /*
    // Print loop timings
    Serial.print("Loop execution time: ");
    Serial.print(millis() - previousMillis);
    Serial.println(" ms");
    */
  }
}

/***************************************************************************************************
 * @description Checks for messages sent from the visualizer and reads them if any are present
 **************************************************************************************************/
void read_serial_commands()
{
  if (Serial.available() > 0)
  {
    // Read the command
    String input = Serial.readStringUntil('\n');

    // Remove leading or trailing whitespaces
    input.trim();

    // Create an empty array to hold the values received by the PLC
    String values[5];

    // Initialize variables for while loop
    int i = 0;
    int lastIndex = -1;

    // Parse the input string and extract visualizer commands
    while (i < 5)
    {
      int index = input.indexOf(',', lastIndex + 1);
      values[i] = input.substring(lastIndex + 1, index);
      lastIndex = index;
      i++;
    }

    // If a solenoid command was received, set solenoidControl accordingly
    if (values[0].length() > 0)
    {
      int temp = values[0].toInt();
      if (temp == 0 || temp == 1)
      {
        solenoidCommand = temp;
      }
    }

    // If a vent enabling command was received, set ventEnableCommand accordingly
    if (values[1].length() > 0)
    {
      int temp = values[1].toInt();
      // Check to make sure hanbay command is a valid percentage before setting it
      if (temp >= 0 && temp <= 1)
      {
        ventEnableCommand = temp;
      }
    }

    // If a parker hanbay command was received, set parkerHanbayControl accordingly
    if (values[2].length() > 0)
    {
      int temp = values[2].toInt();
      // Check to make sure hanbay command is a valid percentage before setting it
      if (temp >= 0 && temp <= 100)
      {
        parkerHanbayCommand = temp;
      }
    }

    // If a dragon hanbay command was received, set dragonHanbayControl accordingly
    if (values[3].length() > 0)
    {
      int temp = values[3].toInt();
      // Check to make sure hanbay command is a valid percentage before setting it
      if (temp >= 0 && temp <= 100)
      {
        dragonHanbayCommand = temp;
      }
    }

    // If a vent rate command was received, set ventRateCommand accordingly
    if (values[4].length() > 0)
    {
      float temp = values[4].toFloat();
      // Check to make sure hanbay command is a valid percentage before setting it
      if (temp >= 0 && temp <= 100)
      {
        ventRateCommand = temp;
      }
    }
  }
}

/***************************************************************************************************
 * @description Maps a float value from one range to another.
 *
 * @param x Input value to be mapped.
 * @param inMin Minimum value of the input range.
 * @param inMax Maximum value of the input range.
 * @param outMin Minimum value of the output range.
 * @param outMax Maximum value of the output range.
 * @return Mapped float value.
 **************************************************************************************************/
float map_float(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/***************************************************************************************************
 * @description interpolates a float value from one range to another.
 *
 * @param temperature temperature value to be interpolated.
 * @param pressure pressure value to be interpolated.
 **************************************************************************************************/
float interpolate(float temperature, float pressure)
{
  // convert gauge pressure to absolute pressure
  pressure += 1.01325;
  // Check to make sure temperature and pressure are within the bounds of the lookup table
  if (temperature < minimumTemperature || temperature > maximumTemperature || pressure < minimumPressure || pressure > maximumPressure)
  {
    Serial.println("Temperature and/or pressure values out of range.");
    return NAN;
  }

  int lowerTempIndex = int(temperature);        // Floor value for temperature
  int lowerPressureIndex = int(pressure);       // Floor value for pressure
  float deltaT = temperature - lowerTempIndex;  // Fractional part of temperature
  float deltaP = pressure - lowerPressureIndex; // Fractional part of pressure

  // Perform bilinear interpolation
  float interpolatedValue =
      (1 - deltaT) * (1 - deltaP) * get_density_value(lowerTempIndex, lowerPressureIndex) + deltaT * (1 - deltaP) * get_density_value(lowerTempIndex + 1, lowerPressureIndex) + (1 - deltaT) * deltaP * get_density_value(lowerTempIndex, lowerPressureIndex + 1) + deltaT * deltaP * get_density_value(lowerTempIndex + 1, lowerPressureIndex + 1);

  return interpolatedValue;
}

/***************************************************************************************************
 * @description Retrieves the temperature value by interpolating from a lookup table based on the
 * measured resistance of an RTD sensor.
 *
 * @param measuredResistance - The resistance value measured by the RTD sensor.
 *
 * @return The interpolated temperature value corresponding to the given resistance.
 **************************************************************************************************/
double get_rtd_temperature(float measuredResistance)
{
  // Check if resistance is within bounds
  if (measuredResistance < minimumResistance || measuredResistance > maximumResistance)
  {
    Serial.print("Resistance value out of lookup table bounds: ");
    Serial.println(measuredResistance);
    return NAN; // Return NaN if resistance is outside the specified bounds
  }

  // Floor and ceiling values for resistance
  float measuredResistanceFloor = floor(measuredResistance * 10) / 10.0;
  float measuredResistanceCeiling = measuredResistanceFloor + resistanceStep;

  // Open the lookup table file
  File lookup_file = SD.open(RTD_LOOKUP_TABLE, FILE_READ);

  // Check if the file is successfully opened
  if (!lookup_file)
  {
    Serial.println("Error opening the lookup table file.");
    return NAN; // Return NaN if the file couldn't be opened
  }
  else
  {
    // Find bounding y-values (Y1 & Y2) via lookup table
    float measuredResistanceFloorIndex = (measuredResistanceFloor - minimumResistance) / resistanceStep;

    // Calculate byte indices for floor and ceiling values
    unsigned long measuredResistanceFloorByteIndex = static_cast<unsigned long>(round(measuredResistanceFloorIndex * doubleSizeInBytes));
    unsigned long measuredResistanceCeilingByteIndex = measuredResistanceFloorByteIndex + doubleSizeInBytes;

    // Get temperature values from the lookup table
    double lowerTemp = get_double_from_lookup_table(lookup_file, measuredResistanceFloorByteIndex, measuredResistance);
    double higherTemp = get_double_from_lookup_table(lookup_file, measuredResistanceCeilingByteIndex, measuredResistance);

    // Perform linear interpolation for the final temperature value
    double interpolatedTemperature = interpolate(measuredResistance, measuredResistanceFloor, measuredResistanceCeiling, lowerTemp, higherTemp);
    lookup_file.close();
    return interpolatedTemperature; // Return the interpolated temperature
  }
}

/***************************************************************************************************
 * @description Retrieves a double value from a lookup table file at a specified position.
 *
 * @param file - The File object representing the lookup table file.
 * @param positionToSeek - The position in bytes to seek in the lookup table file.
 *
 * @return The retrieved double value from the lookup table.
 **************************************************************************************************/
double get_double_from_lookup_table(File lookup_file, unsigned long positionToSeek, double measuredResistance)
{
  // Seek to the calculated position in the file
  if (!lookup_file.seek(positionToSeek))
  {
    Serial.print("Error searching lookup table: value out of range. ");
    Serial.print(measuredResistance);
    Serial.print(" Position: ");
    Serial.println(positionToSeek);
    return NAN; // Return NaN if seeking to the position fails
  }
  else
  {
    // Read the double value from the file
    double lookupValue;
    lookup_file.read((uint8_t *)&lookupValue, doubleSizeInBytes);

    // Print the value from the lookup table
    /*
    Serial.print("Value from lookup Table: ");
    Serial.println(lookupValue, 8);
    */
    return lookupValue; // Return the retrieved double value
  }
}

/***************************************************************************************************
 * @description interpolates a float value from one range to another.
 *
 * @param file Hydrogen lookup file to open
 * @param temperature temperature value to be interpolated.
 * @param pressure pressure value to be interpolated.
 **************************************************************************************************/
float get_density_value(float temperature, float pressure)
{
  File lookupFile = SD.open(LOOKUP_TABLE_FILE_NAME, FILE_READ);
  // calculate the position to seek to based on row and column indices
  unsigned long positionToSeek = (temperature - minimumTemperature) * (numColumns * doubleSizeInBytes) + (pressure - minimumPressure) * doubleSizeInBytes;

  if (temperature == NAN || pressure == NAN)
  {
    return NAN;
  }
  // seek to the calculated position, throw error if position doesn't exist
  if (!lookupFile.seek(positionToSeek))
  {
    Serial.println("Error seeking to position.");
    lookupFile.close();
    return NAN;
  }

  char asciiString[doubleSizeInBytes + 1]; // +1 for null terminator
  if (lookupFile.readBytes(asciiString, doubleSizeInBytes) != doubleSizeInBytes)
  {
    Serial.println("Error reading data from lookupFile.");
    lookupFile.close();
    return NAN;
  }
  asciiString[doubleSizeInBytes] = '\0'; // Null-terminate the string

  float value = atof(asciiString); // convert ASCII string to float
  lookupFile.close();              // Close the lookup file
  return value;
}

/***************************************************************************************************
 * @description Performs a linear 1D interpolation.
 *
 * @param x - The x-coordinate to interpolate.
 * @param x0 - The lower x-coordinate of the interpolation range.
 * @param x1 - The upper x-coordinate of the interpolation range.
 * @param y0 - The corresponding y-coordinate at x0.
 * @param y1 - The corresponding y-coordinate at x1.
 *
 * @return The interpolated y-coordinate at the given x-coordinate.
 **************************************************************************************************/
double interpolate(double x, double x0, double x1, double y0, double y1)
{
  // Check if x is outside the range [x0, x1]
  if (x < x0 || x > x1)
  {
    return NAN; // Return NaN (Not a Number) if x is outside the range
  }

  // Perform linear interpolation
  double y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);

  return y; // Return the interpolated value
}

/***************************************************************************************************
 * @description Reads sensor values from pressure sensors, vacuum gauge, and temperature sensors.
 *              Maps analog input values to corresponding physical quantities.
 **************************************************************************************************/
void read_sensor_values()
{
  // Read PT711 Pressure Sensor
  float PT711Current = map_float(P1.readAnalog(P1_08ADL_1_MODULE_1, 1), 0, THIRTEEN_BIT_ADC_COUNT, 0, 20); // map ADC count to current'
  if (PT711Current < 3.996 || PT711Current > 20)
  {
    PT711 = NAN;
    Serial.println("Error: PT711 Reporting Incorrect Values or Disconnected.");
  }
  else
  {
    PT711 = map_float(PT711Current, 3.996, 20, 0, 482.63301); // map current to pressure (min = 0 bar, max=482.63301bar (7000 PSI))
  }

  // Read PT712 Pressure Sensor
  float PT712Current = map_float(P1.readAnalog(P1_08ADL_1_MODULE_1, 2), 0, THIRTEEN_BIT_ADC_COUNT, 0, 20); // map ADC count to current
  if (PT712Current < 3.995 || PT712Current > 19.992)
  {
    PT712 = NAN;
    Serial.println("Error: PT712 Reporting Incorrect Values or Disconnected.");
  }
  else
  {
    PT712 = map_float(PT712Current, 3.995, 19.992, 0, 482.63301); // map current to pressure (min = 0 bar, max=482.63301bar (7000 PSI))
  }

  // Update the array with the new measurements
  if (PT711 != NAN && PT712 != NAN)
  {
    PTArray[PTArrayIndex] = (PT711 + PT712) / 2;
  }
  else if (PT711 != NAN)
  {
    PTArray[PTArrayIndex] = PT711;
  }
  else if (PT712 != NAN)
  {
    PTArray[PTArrayIndex] = PT712;
  }
  PTArrayIndex = (PTArrayIndex + 1) % averageItemCount;

  // Calculate the running average
  float sum = 0;
  for (int i = 0; i < averageItemCount; i++)
  {
    sum += PTArray[i];
  }
  PTRUNNINGAVERAGE = sum / averageItemCount;
  /*
  Serial.print("PT711: ");
  Serial.print(PT711);
  Serial.print(" PT712: ");
  Serial.print(PT712);
  Serial.print(" PTRUNNINGAVERAGE: ");
  Serial.println(PTRUNNINGAVERAGE);
  */

  // Read VG782 Vacuum Gauge
  float VG782Current = map_float(P1.readAnalog(P1_08ADL_1_MODULE_1, 3), 0, THIRTEEN_BIT_ADC_COUNT, 0, 20); // map ADC count to current
  if (VG782Current < 4 || VG782Current > 20)
  {
    VG782 = NAN;
    Serial.println("Error: VG782 Reporting Incorrect Values or Disconnected.");
  }
  else
  {
    VG782 = map_float(VG782Current, 4, 20, 0.01, 100.0); // map current to vacuum pressure (min 0.01, max 100)
  }

  // Read Temperature Sensors
  float TT711Resistance = P1.readTemperature(P1_04RTD_MODULE_1, 1); // Read resistances from 0 to 193.3125 ohms
  float TT712Resistance = P1.readTemperature(P1_04RTD_MODULE_1, 2); // Read resistances from 0 to 193.3125 ohms
  float TT741Resistance = P1.readTemperature(P1_04RTD_MODULE_1, 3); // Read resistances from 0 to 193.3125 ohms
  float TT742Resistance = P1.readTemperature(P1_04RTD_MODULE_2, 1); // Read resistances from 0 to 193.3125 ohms
  float TT743Resistance = P1.readTemperature(P1_04RTD_MODULE_2, 2); // Read resistances from 0 to 193.3125 ohms
  float TT744Resistance = P1.readTemperature(P1_04RTD_MODULE_2, 3); // Read resistances from 0 to 193.3125 ohms
  float TT791Resistance = P1.readTemperature(P1_04RTD_MODULE_3, 1); // Read resistances from 0 to 193.3125 ohms
  float TT792Resistance = P1.readTemperature(P1_04RTD_MODULE_3, 2); // Read resistances from 0 to 193.3125 ohms
  float TT793Resistance = P1.readTemperature(P1_04RTD_MODULE_3, 3); // Read resistances from 0 to 193.3125 ohms

  TT711 = get_rtd_temperature(TT711Resistance);
  TT712 = get_rtd_temperature(TT712Resistance);
  TT741 = get_rtd_temperature(TT741Resistance);
  TT742 = get_rtd_temperature(TT742Resistance);
  TT743 = get_rtd_temperature(TT743Resistance);
  TT744 = get_rtd_temperature(TT744Resistance);
  TT791 = get_rtd_temperature(TT791Resistance);
  TT792 = get_rtd_temperature(TT792Resistance);
  TT793 = get_rtd_temperature(TT793Resistance);

  TTArray[TTArrayIndex] = (TT741 + TT742 + TT743) / 3.0;
  TTArrayIndex = (TTArrayIndex + 1) % averageItemCount;

  // Calculate the running average
  sum = 0;
  for (int i = 0; i < averageItemCount; i++)
  {
    sum += TTArray[i];
  }
  TTRUNNINGAVERAGE = sum / averageItemCount;
}

/***************************************************************************************************
 * @description Actuates parker hanbay valve based on sensor readings and visualizer commands.
 *              Implements logic for venting at a specified vent rate
 **************************************************************************************************/
void actuate_parker_valve()
{
  // declare variables
  float elapsedVentTimeSeconds;
  static int parkerHanbayBits;

  // Define and initialize an enum for structuring states of sstate machine
  enum controlledVentingState
  {
    manually_control_valve,
    initialize_venting,
    venting,
    terminate_venting
  };
  static controlledVentingState currentState = manually_control_valve;

  // Switch statement to handle different states
  switch (currentState)
  {
  case manually_control_valve:
    desiredHydrogenStored = NAN;
    // Manually acutate Parker Hanbay Valve
    parkerHanbayBits = map_float(parkerHanbayCommand, 0, 100, 0, 4095); // set valve to parkerHanbayCommand position (from visualizer)
    P1.writeAnalog(parkerHanbayBits, P1_04DAL_1_MODULE_1, 1);

    /*
  Serial.print("TTRUNNINGAVERAGE: ");
  Serial.print(TTRUNNINGAVERAGE, 3);
  Serial.print(" PTRUNNINGAVERAGE: ");
  Serial.println(PTRUNNINGAVERAGE, 3);
  */

    // calculates H2 Density (in g/L) and multiplies by tank volume to find total H2 in grams
    actualHydrogenStored = interpolate(TTRUNNINGAVERAGE, PTRUNNINGAVERAGE) * TANK_VOLUME_L / 1000;

    // Initialize Venting if Vent Conrol is enabled
    if (ventEnableCommand == 1)
    {
      ventInitiationTime = millis();
      currentState = initialize_venting;
    }
    break;

  case initialize_venting:
    parkerHanbayCommand = ventCutoff;                                   // point at which it starts to vent, more or less
    parkerHanbayBits = map_float(parkerHanbayCommand, 0, 100, 0, 4095); // set valve to cutoff value, on verge of opening
    P1.writeAnalog(parkerHanbayBits, P1_04DAL_1_MODULE_1, 1);

    if (millis() - ventInitiationTime >= 1800 * ventCutoff) // wait for valve to almost begin to open, then calculate initial parameters
    {
      // Convert vent rate from kg/hr to kg/s
      desiredVentRate = ventRateCommand / 3600;

      // Calculate H2 Density (in g/L) and multiply by tank volume (in L) then divide by 1000 to find total H2 storage in kg
      initialHydrogenStored = interpolate(TTRUNNINGAVERAGE, PTRUNNINGAVERAGE) * TANK_VOLUME_L / 1000;

      // Record venting parameters
      Serial.print("initialHydrogenStored: ");
      Serial.print(initialHydrogenStored);
      Serial.print(" desiredVentRate: ");
      Serial.println(desiredVentRate, 6);

      // Set ElapsedVentTime to millis()
      ventInitiationTime = millis();

      // Switch to venting state
      currentState = venting;
    }
    break;

  case venting:
    // Calculate desired H2 storage in grams, per the specified desiredVentRate
    elapsedVentTimeSeconds = static_cast<float>(millis() - ventInitiationTime) / 1000.0;

    // Serial.print("elapsedVentTimeSeconds: ");
    // Serial.print(elapsedVentTimeSeconds, 3);
    // Serial.print(" initialHydrogenStored: ");
    // Serial.print(initialHydrogenStored, 6);
    // Serial.print(" desiredVentRate: ");
    // Serial.println(desiredVentRate, 6);
    desiredHydrogenStored = initialHydrogenStored - (desiredVentRate * elapsedVentTimeSeconds);

    // calculates H2 Density (in g/L) and multiplies by tank volume to find total H2 in grams
    actualHydrogenStored = interpolate(TTRUNNINGAVERAGE, PTRUNNINGAVERAGE) * TANK_VOLUME_L / 1000;

    /*
    Serial.print("Desired Hydrogen Stored: ");
    Serial.print(desiredHydrogenStored, 6);
    Serial.print(" Actual Hydrogen Stored: ");
    Serial.println(actualHydrogenStored, 6);
    */

    if (actualHydrogenStored < desiredHydrogenStored) // flow too fast, close valve
    {
      if (parkerHanbayCommand - valveStep >= 0)
      {
        parkerHanbayCommand -= valveStep;
      }
      parkerHanbayBits = map_float(parkerHanbayCommand, 0, 100, 0, 4095); // set valve to 0 (totally closed)
      P1.writeAnalog(parkerHanbayBits, P1_04DAL_1_MODULE_1, 1);
    }
    else if (actualHydrogenStored >= desiredHydrogenStored) // flow too slow, open valve
    {
      if (parkerHanbayCommand + valveStep <= 100)
      {
        parkerHanbayCommand += valveStep;
      }
      parkerHanbayBits = map_float(parkerHanbayCommand, 0, 100, 0, 4095); // set valve to 100 (totally open)
      P1.writeAnalog(parkerHanbayBits, P1_04DAL_1_MODULE_1, 1);
    }

    if (ventEnableCommand == 0)
    {
      currentState = terminate_venting;
    }
    break;

  case terminate_venting:
    // Set Hanbay Valve to 0 (totally closed)
    parkerHanbayCommand = 0.0;
    parkerHanbayBits = map_float(parkerHanbayCommand, 0, 100, 0, 4095);
    P1.writeAnalog(parkerHanbayBits, P1_04DAL_1_MODULE_1, 1);

    desiredHydrogenStored = NAN;
    currentState = manually_control_valve;
    break;
  }
}

/***************************************************************************************************
 * @description Actuates Dragon Hanbay valve based on visualizer commands
 **************************************************************************************************/
void actuate_dragon_valve()
{
  // Control the dragon hanbay valve
  int dragonHanbayBits = map_float(dragonHanbayCommand, 0, 100, 0, 4095);
  P1.writeAnalog(dragonHanbayBits, P1_04DAL_1_MODULE_1, 2);
}

/***************************************************************************************************
 * @description Actuates valves based on sensor readings and visualizer commands.
 *              Implements logic for hard-coded MOP venting.
 **************************************************************************************************/
void actuate_solenoid_valve()
{
  // State Machine logic for hard-coded MOP venting
  if ((PT711 > XV762_VENT_PRESSURE || PT712 > XV762_VENT_PRESSURE) && !hardCodedVentingActivated)
  {
    if (MOPVentingStartTime == 0)
    {
      MOPVentingStartTime = millis(); // Start the venting timer
    }

    if (millis() - MOPVentingStartTime >= VENTING_DURATION)
    {
      // Activate hard-coded venting if PT711 or PT712 pressure has been above vent threshold for 5 seconds
      hardCodedVentingActivated = true;

      // Print status to serial
      Serial.print("INFO: Solenoid venting, PT711 (");
      Serial.print(PT711);
      Serial.print(") or PT712 (");
      Serial.print(PT712);
      Serial.println(") pressure greater than 400 bar for over 5 seconds.");
    }
  }
  else
  {
    // Reset the venting timer if pressure is below vent threshold
    MOPVentingStartTime = 0;

    if (hardCodedVentingActivated)
    {
      // Check if both PT711 and PT712 are below reseal pressure for at least VENTING_DURATION milliseconds
      if (PT711 < XV762_RESEAL_PRESSURE && PT712 < XV762_RESEAL_PRESSURE)
      {
        if (MOPResealingStartTime == 0)
        {
          MOPResealingStartTime = millis(); // Start the resealing timer
        }

        if (millis() - MOPResealingStartTime >= VENTING_DURATION)
        {
          // Deactivate hard-coded venting and reset if both PT711 and PT712 have been below reseal pressure for 5 seconds
          hardCodedVentingActivated = false;
          MOPResealingStartTime = 0;

          // Print status to serial
          Serial.print("INFO: Solenoid resealing, PT711 (");
          Serial.print(PT711);
          Serial.print(") or PT712 (");
          Serial.print(PT712);
          Serial.println(") pressure lower than 395 bar for over 5 seconds.");
        }
      }
      else
      {
        // Reset the resealing timer if either PT711 or PT712 is above reseal pressure
        MOPResealingStartTime = 0;
      }
    }
  }

  // Activate solenoid valve
  if (solenoidCommand == 1 || hardCodedVentingActivated == true)
  {
    XV762Status = 1;                              // Indicate that XV762 solenoid is enabled
    P1.writeDiscrete(HIGH, P1_16CDR_MODULE_1, 2); // Turn slot 1 channel 1 on
  }
  else if (solenoidCommand == 0 || hardCodedVentingActivated == false)
  {
    XV762Status = 0;                             // Indicate that XV762 solenoid is disabled
    P1.writeDiscrete(LOW, P1_16CDR_MODULE_1, 2); // Turn slot 1 channel 1 off
  }
}

/***************************************************************************************************
 * @description Writes sensor values, valve status, and visualizer commands to Serial for monitoring.
 **************************************************************************************************/
void write_to_serial()
{
  Serial.print(PT711);
  Serial.print(',');
  Serial.print(PT712);
  Serial.print(',');
  Serial.print(PTRUNNINGAVERAGE);
  Serial.print(',');
  Serial.print(TT711);
  Serial.print(',');
  Serial.print(TT712);
  Serial.print(',');
  Serial.print(TT741);
  Serial.print(',');
  Serial.print(TT742);
  Serial.print(',');
  Serial.print(TT743);
  Serial.print(',');
  Serial.print(TT744);
  Serial.print(',');
  Serial.print(TT791);
  Serial.print(',');
  Serial.print(TT792);
  Serial.print(',');
  Serial.print(TT793);
  Serial.print(',');
  Serial.print(VG782);
  Serial.print(',');
  Serial.print(XV762Status);
  Serial.print(',');
  Serial.print(parkerHanbayCommand);
  Serial.print(',');
  Serial.print(dragonHanbayCommand);
  Serial.print(',');
  Serial.print(ventEnableCommand);
  Serial.print(',');
  Serial.print(ventRateCommand);
  Serial.print(',');
  Serial.print(desiredHydrogenStored, 3);
  Serial.print(',');
  Serial.println(actualHydrogenStored, 3);
}

/***************************************************************************************************
 * @description Generates a timestamp indicating the elapsed time since the PLC started.
 *
 * @return Timestamp as a formatted string.
 **************************************************************************************************/
String generate_timestamp()
{
  // Calculate the elapsed time in milliseconds
  elapsedTimestampMillis = millis() - startTimestampMillis;

  // Calculate the total numbers of seconds, minutes, hours, and days
  unsigned long int seconds = (elapsedTimestampMillis / 1000);
  unsigned long int minutes = (seconds / 60);
  int hours = (minutes / 60);

  // Calculate modulus for timestamp generation
  int milliseconds = elapsedTimestampMillis % 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 10000; // make sure no hours overflow issue

  // Define an empty char array to store the timestamp
  char buffer[14];

  // Fill the array with the timestamp info, and return the array as a String
  sprintf(buffer, "%04d:%02lu:%02lu.%03d", hours, minutes, seconds, milliseconds);
  return String(buffer);
}

/***************************************************************************************************
 * @description Initializes the SD card and prints success or failure message to Serial.
 *
 * @return True if SD card initialization is successful, false otherwise.
 **************************************************************************************************/
bool initialize_sd_card(void)
{
  Serial.print("Attempting to connect to SD card...");
  if (!SD.begin(CHIP_SELECT_PIN))
  {
    Serial.println("SD card connection failed. Insert SD card or check SD card connection.");
    return false;
  }
  else
  {
    Serial.println("SD card connected successfully.");
    return true;
  }
}

/***************************************************************************************************
 * @description Writes sensor data, valve status, and visualizer commands to the SD card.
 *              Handles SD card disconnection and attempts to reconnect after a specified interval.
 **************************************************************************************************/
void write_to_sd_card()
{
  // Generate a timestamp for the data entry
  String dataString = generate_timestamp();

  // Append sensor data to the string
  dataString += ",";
  dataString += String(PT711);
  dataString += ",";
  dataString += String(PT712);
  dataString += ",";
  dataString += String(PTRUNNINGAVERAGE);
  dataString += ",";
  dataString += String(TT711);
  dataString += ",";
  dataString += String(TT712);
  dataString += ",";
  dataString += String(TT741);
  dataString += ",";
  dataString += String(TT742);
  dataString += ",";
  dataString += String(TT743);
  dataString += ",";
  dataString += String(TT744);
  dataString += ",";
  dataString += String(TT791);
  dataString += ",";
  dataString += String(TT792);
  dataString += ",";
  dataString += String(TT793);
  dataString += ",";
  dataString += String(VG782);
  dataString += ",";
  dataString += String(XV762Status);
  dataString += ",";
  dataString += String(parkerHanbayCommand);
  dataString += ",";
  dataString += String(dragonHanbayCommand);
  dataString += ",";
  dataString += String(ventEnableCommand);
  dataString += ",";
  dataString += String(ventRateCommand);
  dataString += ",";
  dataString += String(desiredHydrogenStored);
  dataString += ",";
  dataString += String(actualHydrogenStored);

  // Attempt to open the data file on the SD card
  File dataFile = SD.open(LOG_FILE_NAME, FILE_WRITE);

  // If the data file on the SD card is available, write to it
  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
    sdCardReconnectAttemptsCounter = 0; // Reset the attempts counter
  }
  // If the file isn't open, handle the error
  else
  {
    sdCardReconnectAttemptsCounter += 1;

    // Check if it's time to attempt reconnecting to the SD card
    if (sdCardReconnectAttemptsCounter >= SD_CARD_RECONNECT_INTERVAL)
    {
      initialize_sd_card();               // Reinitialize the SD card
      sdCardReconnectAttemptsCounter = 0; // Reset the attempts counter
    }
    else
    {
      // Print a message indicating SD card disconnection and the time until the next attempt
      Serial.print("SD Card Disconnected. Reconnecting in ");
      Serial.print(SD_CARD_RECONNECT_INTERVAL - sdCardReconnectAttemptsCounter);
      Serial.println(" seconds.");
    }
  }
}

/***************************************************************************************************
 *
 *                        SIMULATION CODE FOR TESTING PURPOSES ONLY
 *
 **************************************************************************************************/

// Declare variables for PT711 simulation testing
unsigned long PT711LastUpdateTime = 0;
float PT711Increment = 0.25;
int PT711IncrementInterval = 250; // increment time in milliseconds
bool isPT711Incrementing = true;

// Declare variables for PT712 simulation testing
unsigned long PT712LastUpdateTime = 0;
float PT712Increment = 0.25;
int PT712IncrementInterval = 250; // increment time in milliseconds
bool isPT712Incrementing = true;

/***************************************************************************************************
 * @description Simulates PT711 pressure decrease due to controlled venting
 **************************************************************************************************/
void simulate_PT711_controlled_venting()
{
  unsigned long currentTime = millis();
  if (ventEnableCommand == 0)
  {
    PT711 = 18;
  }
  else
  {
    // Check if 1 second has passed since the last update
    if (currentTime - PT711LastUpdateTime >= PT711IncrementInterval)
    {
      // Update the last update time
      PT711LastUpdateTime = currentTime;

      if (parkerHanbayCommand > 0 && PT711 >= 0)
      {
        PT711 -= 0.1;
      }
      // Print the current value of PT711 (for testing purposes)
      // Serial.println("PT711: " + String(PT711));
    }
  }
}

/***************************************************************************************************
 * @description Simulates PT712 pressure decrease due to controlled venting
 **************************************************************************************************/
void simulate_PT712_controlled_venting()
{
  unsigned long currentTime = millis();
  if (ventEnableCommand == 0)
  {
    PT712 = 19;
  }
  else
  {
    // Check if 1 second has passed since the last update
    if (currentTime - PT712LastUpdateTime >= PT712IncrementInterval)
    {
      // Update the last update time
      PT712LastUpdateTime = currentTime;

      if (parkerHanbayCommand > 0 && PT712 >= 0)
      {
        PT712 -= 0.1;
      }
      // Print the current value of PT712 (for testing purposes)
      // Serial.println("PT712: " + String(PT712));
    }
  }
}

/***************************************************************************************************
 * @description Simulates random sensor values for testing purposes.
 **************************************************************************************************/
void simulate_random_sensor_values()
{
  // simulate_PT711_MOP_venting()
  // simulate_PT712_MOP_venting()
  simulate_PT711_controlled_venting();
  simulate_PT712_controlled_venting();
  TT711 = float(random(101));
  TT712 = float(random(101));
  TT741 = 110;
  TT742 = 111;
  TT743 = 112;
  TT744 = 113;
  TT791 = float(random(101));
  TT792 = float(random(101));
  TT793 = float(random(101));
  // VG782 = float(random(101));
}