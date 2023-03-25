/*
This is the code that will receive a command from the jetson using UART (Serial1 in this case), take in that string and
then parse it into a command. This command will then be put into a struct (aptly named command), which holds the type of
command (a string, example "pwmFunc") the array of parameters (ints), an int to represent the number of parameters, and
an int to represent the validity of the command (used to issue feedback/debugging for the software team)

To add new commands:

Step 1:
  Create the command as a function within the `commands` section (currently line 45 but will inevitably change)
Step 2:
  Within the `validate` function create a new else if case for your new command, and add in whatever limits it needs
  (needs X amount of params, and param[X] needs to be within a certain range)
Step 3:
  Within the `sendCommand` function create a new else if case for your new command, and convert/cast/whatever needs to
  be done so that the parameter types match what the function takes in
*/

// Libraries Used:
/// https://github.com/d03n3rfr1tz3/HC-SR04
/// https://github.com/adafruit/Adafruit_Sensor - not sure?
/// https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library 

/*
Instructions for installing library:

Step 1: 
  Go to github and download the entire repo as a .zip file
Step 2: 
  Open Arduino IDE and click on Sketch > Include Library > Add .ZIP Library
Step 3: 
  Select downloaded .zip file
Step 4: 
  Profit ?
*/

#define DEBUGSERIAL Serial
#define COMMSERIAL Serial // Change this back to `Serial1` if using Arduino Mega

bool promptSent = false;

#define NO_PARAMS 0
#define VALID 1
#define INVALID_COMMAND 2
#define INVALID_PARAMS 3
#define PARSING_ERROR 4
#define WRONG_NUM_PARAM 5
#define PARAM_RANGE_ERROR 6
#define UNKNOWN_COMMAND 7
#define COMMAND_NOT_VALIDATED 8

const String ErrorCodes[9] = {
  "No parameters",                       // 0
  "Valid",                               // 1
  "Invalid command",                     // 2
  "Invalid paramters",                   // 3
  "Something went wrong during parsing", // 4
  "Wrong number of parameters",          // 5
  "Paramter outside of possible range",  // 6
  "Unknown command",                     // 7
  "Command was not validated first"      // 8
};

/// Struct Variables ///
#define MAXPARAMS 4
#define PLACEHOLDER -420 // Change to a val that is never used (as new functions are added)

struct command {
  String type;
  // If MAXPARAMS is changed this needs to be as well
  int params[MAXPARAMS] = {-420, -420, -420, -420}; 
  uint8_t paramCount;
  uint8_t validity;
  bool validated = false;
};
/// End Struct Variables ///

/// Ultrasonic Sensor Variables ///
#include <HCSR04.h>
#define ULTRASONIC_SENSOR_COUNT 4 - 1 // subtract one because index starts at 0
HCSR04Sensor distanceSensor[ULTRASONIC_SENSOR_COUNT + 1]; // Array of all our Ultrasonic Sensors
/// End Ultrasonic Sensor Variables ///

/// PWM Controller Variables ///
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // called this way, it uses the default address 0x40
uint16_t dutyCyleArray[16] = {0}; // To keep track of duty cycle per channel
/// End PWM Controller Variables ///

/// Commands ///
/**
 * @brief Get distance of one of the ultrasonic sensors
 *
 * @param sensorNum The ultrasonic sensor number to get distance for
 * @return Distance in cm of given sensor
 */
String dist(uint8_t sensorNum){
  // check if number is bigger than the bumber of Ultrasonic Sensors that we use - 1
  if (sensorNum > ULTRASONIC_SENSOR_COUNT) {
    return "-1";
  }

  double *dist_cm;

  dist_cm = distanceSensor[sensorNum].measureDistanceCm();

  return String(dist_cm[0]);
}

/**
 * @brief Set pwm frequency across all channels
 * 
 * @param hz Frequency
 * @return Result string
 */
String pwmFunc(uint16_t hz){
  // check if freq is to low
  if (hz < 40) {
    return String("1,0");
  }
  // check if freq is to high
  if (hz > 1600) {
    return String("1,0");
  }

  pwm.setPWMFreq(hz);

  // hzArray[channel] = hz; //ensure we keep track of each of the HZ

  return String("0," + String(hz, DEC));
}

/**
 * @brief Sets duty cycle for a specific channel
 *
 * @param channel PWM channel to set
 * @param dutyCyle Duty cycle to be set
 * @return Result string
 */
String pwmDuty(uint8_t channel, uint16_t dutyCyle){
  // check to see if its a valid channel
  if (channel > 15) {
    return String("1,0");
  }
  // check if dutyCyle is to high
  if (dutyCyle > 10000) {
    return String("1," + String(dutyCyleArray[channel] / 100.0));
  }

  // keep track of the dutyCyles
  dutyCyleArray[channel] = dutyCyle;

  double d = dutyCyle / 10000.0;

  pwm.setPWM(channel, 0, (int)(4096 * (d)));

  return String("0," + String(dutyCyleArray[channel] / 100.0));

  // pwm.setPWM(15, 0, (int)(4096 * (dudyDouble)));
}

/**
 * @brief Get PWM frequency for specific channel
 *
 * @param channel PWM channel to get pwm value for
 * @return Result string
 */
String pwmGetVal(uint8_t channel){
  // check to see if its a valid channel
  if (channel > 15) {
    return String("1,0");
  }

  return String("0," + String(dutyCyleArray[channel] / 100.0));
}
/// End commands ///

/**
 * @brief Loop that only runs once, used to set up Serial
 */
void setup(){
  /// Serial ///
  DEBUGSERIAL.begin(9600);
  COMMSERIAL.begin(9600);
  /// End Serial ///

  /// PWM ///
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // This is the maximum PWM frequency
  Wire.setClock(400000);
  /// End PWM ///

  /// Distance Sensors ///
  distanceSensor[0].begin(7, 8); // TODO: add sensors 1 to 3
  /// End Distance Sensors ///
}

/// Parsing Logic ///
/**
 * @brief Parses a string into a command struct
 *
 * @param inputString New line terminating string read in from the jetson
 * @return command Struct that contains the command and parameters that were parsed, along with basic validity checks
 */
command parse(String inputString){
  command toReturn;
  toReturn.paramCount = 0;
  toReturn.validity = PARSING_ERROR;

  // Get the command part of the string
  inputString.trim();
  if (inputString.indexOf(' ') == -1) {
    toReturn.type = inputString;
    toReturn.validity = NO_PARAMS;
    return toReturn;
  }
  int spaceIndex = inputString.indexOf(' ');
  toReturn.type = inputString.substring(0, spaceIndex);
  inputString = inputString.substring(spaceIndex + 1);
  inputString.trim();
  spaceIndex = inputString.indexOf(' ');

  // Get the params part of the string
  int paramIndex = 0;
  while (spaceIndex != -1) {
    if (inputString.substring(0, spaceIndex).length() == 0) {
      Serial.print("here");
      toReturn.validity = INVALID_PARAMS;
      return toReturn;
    }
    toReturn.params[paramIndex] = inputString.substring(0, spaceIndex).toInt();
    inputString = inputString.substring(spaceIndex + 1);
    inputString.trim();
    spaceIndex = inputString.indexOf(' ');
    paramIndex++;
  }

  // Get the last parameter since it doesnt have a trailing space
  if (inputString.substring(spaceIndex + 1).length() == 0) {
    toReturn.validity = INVALID_PARAMS;
    return toReturn;
  }
  toReturn.params[paramIndex] = inputString.substring(spaceIndex + 1).toInt();

  // Count number of params
  for (int i = 0; i < MAXPARAMS; i++) {
    if (toReturn.params[i] == PLACEHOLDER) {
      toReturn.paramCount = i;
      break;
    }
  }
  toReturn.validity = VALID;
  return toReturn;
}

/**
 * @brief Checks the cmd.validity state and simplifies it into true/false
 *
 * @param cmd Command to validate
 * @return true Command is valid (`ErrorCodes[cmd.validity]` is either 0 or 1)
 * @return false Command is invalid, use `ErrorCodes[cmd.validity]` to see the error
 */
bool validate(command &cmd){
  if (cmd.type.equals("pwmFunc")) {
    // Needs 1 param
    if (cmd.paramCount != 1) {
      cmd.validity = WRONG_NUM_PARAM;
      return false;
    }
    // Param 1 is between 0 and 1600
    if (cmd.params[0] < 0 || cmd.params[0] > 1600) {
      cmd.validity = PARAM_RANGE_ERROR;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  } else if (cmd.type.equals("pwmDuty")) {
    // Needs 2 params
    if (cmd.paramCount != 2) {
      cmd.validity = WRONG_NUM_PARAM;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15) {
      cmd.validity = PARAM_RANGE_ERROR;
      return false;
    }
    // Param 2 is between 0 and 10000
    if (cmd.params[1] < 0 || cmd.params[1] > 10000) {
      cmd.validity = PARAM_RANGE_ERROR;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  } else if (cmd.type.equals("pwmGetVal")) {
    // Needs 1 param
    if (cmd.paramCount != 1) {
      cmd.validity = WRONG_NUM_PARAM;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15) {
      cmd.validity = PARAM_RANGE_ERROR;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  } else if (cmd.type.equals("dist")) {
    // Needs 1 param
    if (cmd.paramCount != 1) {
      cmd.validity = WRONG_NUM_PARAM;
      return false;
    }
    // Param 1 is between 0 and ULTRASONIC_SENSOR_COUNT
    if (cmd.params[0] < 0 || cmd.params[0] > ULTRASONIC_SENSOR_COUNT) {
      cmd.validity = PARAM_RANGE_ERROR;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  } else {
    cmd.validity = UNKNOWN_COMMAND;
    return false;
  }
}

/**
 * @brief Checks validity of a command
 *
 * @param cmd Command to be checked for validity
 * @return true if valid (cmd.validity == 0 || 1), false if invalid
 */
bool valid(command cmd){
  if (cmd.validity == NO_PARAMS || cmd.validity == VALID) {
    return true;
  } else {
    return false;
  }
}
/// End Parsing Logic ///

/**
 * @brief Sends command to its respective function (`cmd.type`)
 *
 * @param cmd Command to send. Needs to be prevalidated using validate() function
 * @return String Whatever `cmd.type` function returns
 */
String sendCommand(command cmd){
  if (cmd.validated == false) {
    cmd.validity = COMMAND_NOT_VALIDATED;
    return ErrorCodes[cmd.validity];
  }

  if (cmd.type.equals("pwmFunc")) {
    // Convert param 1 to uint16_t
    uint16_t p1 = (uint16_t)cmd.params[1];
    // Send command
    return pwmFunc(p1);
  }
  else if (cmd.type.equals("pwmDuty")) {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Convert param 2 to uint16_t
    uint16_t p2 = (uint16_t)cmd.params[1];
    // Send command
    return pwmDuty(p1, p2);
  }
  else if (cmd.type.equals("pwmGetVal")) {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Send command
    return pwmGetVal(p1);
  }
  else if (cmd.type.equals("dist")) {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Send command
    return dist(p1);
  }

  return "Command not found";
}

/**
 * @brief Main loop that repeats, put code here to run continuously
 */
void loop(){
  // Ask for command
  if (!promptSent)  {
    COMMSERIAL.println("Enter a command:");
    promptSent = true;
  }

  if (COMMSERIAL.available() > 0) {
    // Read in line until selected character
    String inputString = COMMSERIAL.readStringUntil('\n'); // New line
    // String inputString = COMMSERIAL.readStringUntil('\r'); // Carriage return

    command currentCommand = parse(inputString);
    if (!valid(currentCommand)) {
      COMMSERIAL.println(ErrorCodes[currentCommand.validity]);
      promptSent = false;
      return;
    }

    if (!validate(currentCommand)) {
      COMMSERIAL.println(ErrorCodes[currentCommand.validity]);
      promptSent = false;
      return;
    }

    COMMSERIAL.println(sendCommand(currentCommand));
    promptSent = false;
    return;
  }
}
  