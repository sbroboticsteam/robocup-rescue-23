/*
This is the code that will receive a command from the jetson using UART (Serial1 in this case), take in that string and
then parse it into a command. This command will then be put into a struct (aptly named command), which holds the type of
command (a string, example "pwmFunc") the array of parameters (ints), an int to represent the number of parameters, and
an int to represent the validity of the command (used to issue feedback/debugging for the software team)

To add new commands:

Step 1:
  Create the command as a function within the `commands` section (currently line 45 but will inevitably change)
Step 2:
  Within the `parse` function create a new else if case for your new command, and add in whatever limits it needs
  (needs X amount of params, and param[X] needs to be within a certain range)
Step 3:
  Within the `sendCommand` function create a new else if case for your new command, and convert/cast/whatever needs to
  be done so that the parameter types match what the function takes in
*/

#define MAXPARAMS 4
#define PLACEHOLDER -420 // Change to a val that is never used (as new functions are added)
#define DEBUGSERIAL Serial
#define COMMSERIAL Serial // Change this back to `Serial1` when using Arduino Mega

bool promptSent = false;

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

struct command
{
  String type;
  int params[MAXPARAMS] = {-420, -420, -420, -420}; // If MAXPARAMS is changed this needs to be as well
  uint8_t paramCount;
  uint8_t validity;
  bool validated = false;
};

/// Commands ///

String pwmFunc(uint8_t channel, uint16_t hz)
{
  return "good value :)";
}

String pwmDuty(uint8_t channel, uint16_t dutyCyle)
{
  return "good value";
}

String pwmGetVal(uint8_t channel)
{
  return "good value - need one here too :0";
}

/// End commands ///

/**
 * @brief Loop that only runs once, used to set up Serial
 */
void setup()
{
  DEBUGSERIAL.begin(9600);
  COMMSERIAL.begin(9600);
}

/**
 * @brief Parses a string into a command struct
 *
 * @param inputString New line terminating string read in from the jetson
 * @return command Struct that contains the command and parameters that were parsed, along with basic validity checks
 */
command parse(String inputString)
{
  command toReturn;
  toReturn.paramCount = 0;
  toReturn.validity = 4;

  // Get the command part of the string
  inputString.trim();
  if (inputString.indexOf(' ') == -1)
  {
    toReturn.type = inputString;
    toReturn.validity = 0;
    return toReturn;
  }
  int spaceIndex = inputString.indexOf(' ');
  toReturn.type = inputString.substring(0, spaceIndex);
  inputString = inputString.substring(spaceIndex + 1);
  inputString.trim();
  spaceIndex = inputString.indexOf(' ');

  // Get the params part of the string
  int paramIndex = 0;
  while (spaceIndex != -1)
  {
    if (inputString.substring(0, spaceIndex).toInt() == 0)
    {
      toReturn.validity = 3;
      return toReturn;
    }
    toReturn.params[paramIndex] = inputString.substring(0, spaceIndex).toInt();
    inputString = inputString.substring(spaceIndex + 1);
    inputString.trim();
    spaceIndex = inputString.indexOf(' ');
    paramIndex++;
  }

  // Get the last parameter since it doesnt have a trailing space
  if (inputString.substring(spaceIndex + 1).toInt() == 0)
  {
    toReturn.validity = 3;
    return toReturn;
  }
  toReturn.params[paramIndex] = inputString.substring(spaceIndex + 1).toInt();

  // Count number of params
  for (int i = 0; i < MAXPARAMS; i++)
  {
    if (toReturn.params[i] == PLACEHOLDER)
    {
      toReturn.paramCount = i;
      break;
    }
  }
  toReturn.validity = 1;
  return toReturn;
}

/**
 * @brief Checks the cmd.validity state and simplifies it into true/false
 *
 * @param cmd Command to validate
 * @return true Command is valid (`ErrorCodes[cmd.validity]` is either 0 or 1)
 * @return false Command is invalid, use `ErrorCodes[cmd.validity]` to see the error
 */
bool validate(command &cmd)
{
  if (cmd.type.equals("pwmFunc"))
  {
    // Needs 2 params
    if (cmd.paramCount != 2)
    {
      cmd.validity = 5;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15)
    {
      cmd.validity = 6;
      return false;
    }
    // Param 2 is between 0 and 1600
    if (cmd.params[1] < 0 || cmd.params[1] > 1600)
    {
      cmd.validity = 6;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  }
  else if (cmd.type.equals("pwmDuty"))
  {
    // Needs 2 params
    if (cmd.paramCount != 2)
    {
      cmd.validity = 5;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15)
    {
      cmd.validity = 6;
      return false;
    }
    // Param 2 is between 000(.)00 and 100(.)00
    if (cmd.params[1] < 0 || cmd.params[1] > 10000)
    {
      cmd.validity = 6;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  }
  else if (cmd.type.equals("pwmGetVal"))
  {
    // Needs 1 param
    if (cmd.paramCount != 1)
    {
      cmd.validity = 5;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15)
    {
      cmd.validity = 6;
      return false;
    }
    // No issues found, must be valid
    cmd.validated = true;
    return true;
  }
  else
  {
    cmd.validity = 7;
    return false;
  }
}

/**
 * @brief Sends command to its respective function (`cmd.type`)
 *
 * @param cmd Command to send. Needs to be prevalidated using validate() function
 * @return String Whatever `cmd.type` function returns
 */
String sendCommand(command cmd)
{
  if (cmd.validated == false)
  {
    cmd.validity = 8;
    return ErrorCodes[cmd.validity];
  }

  if (cmd.type.equals("pwmFunc"))
  {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Convert param 2 to uint16_t
    uint16_t p2 = (uint16_t)cmd.params[1];
    // Send command
    return pwmFunc(p1, p2);
  }
  else if (cmd.type.equals("pwmDuty"))
  {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Convert param 2 to uint16_t
    uint16_t p2 = (uint16_t)cmd.params[1]; // @Noam, even if 0000 is entered originally, wont this just make it a 0 anyway?
    // Send command
    return pwmDuty(p1, p2);
  }
  else if (cmd.type.equals("pwmGetVal"))
  {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Send command
    return pwmGetVal(p1);
  }
  return "Command not found";
}

bool valid(command cmd)
{
  if (cmd.validity == 0 || cmd.validity == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Main loop that repeats, put code here to run continuously
 */
void loop()
{
  // Ask for command
  if (!promptSent)
  {
    COMMSERIAL.println("Enter a command:");
    promptSent = true;
  }

  if (COMMSERIAL.available() > 0)
  {
    // Read in line (Currently using just newline, no carriage return)
    String inputString = COMMSERIAL.readStringUntil('\n');

    command currentCommand = parse(inputString);
    if (!valid(currentCommand))
    {
      COMMSERIAL.println(ErrorCodes[currentCommand.validity]);
      promptSent = false;
      return;
    }

    if (!validate(currentCommand))
    {
      COMMSERIAL.println(ErrorCodes[currentCommand.validity]);
      promptSent = false;
      return;
    }

    COMMSERIAL.println(sendCommand(currentCommand));
    promptSent = false;
    return;
  }
}