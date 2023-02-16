#define MAXPARAMS 4       // TODO - What value should this actually be?
#define PLACEHOLDER -420  // TODO - need to find a good placeholder value that will never actually be used

bool promptSent = false;


const String ErrorCodes[9] = {};

struct command {
  String type;
  int params[MAXPARAMS] = { -420, -420, -420, -420 }; // If MAXPARAMS is changed this needs to be as well
  uint8_t paramCount;
  uint8_t validity;
};

// Concerns: 
/* uint8_t is too small for hz and dutyCycle
** how to do conversion from XXXxx to XXX.xx and have that still be int, needs float/double? 
** what is the max params going to be AKA is 4 ok? should it be less/more
** placeholder? (for the params array)
** worth saving all the invalid states or should it just be true/false or should there be no states at all? - you should use ErrorCodes const array
** for validate and send i used ignore case, is that ok or should it be changed
** line 58 is preclear params still needed with line 8?
*/

String pwmFunc(uint8_t channel, uint8_t hz) {
  return "good value";
}

String pwmDuty(uint8_t channel, uint8_t dutyCyle) {
  return "good value";
}

String pwmGetVal(uint8_t channel) {
  return "good value";
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); 
   
}

command parse(String inputString) {
  /*
  -1 = No params
  0 = Valid
  1 = Invalid Command
  2 = Invalid Params
  3 = Something else went wrong
  */

  command toReturn;
  // Preclear params (is this still needed?)
  for (int i = 0; i < MAXPARAMS; i++) {
    toReturn.params[i] = -420;
  }
  toReturn.paramCount = 0;
  toReturn.validity = 3;

  // Get the command part of the string
  inputString.trim();
  if (inputString.indexOf(' ') == -1) {
    toReturn.type = inputString;
    toReturn.validity = -1;
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
    if (inputString.substring(0, spaceIndex).toInt() == 0) {
      toReturn.validity = 2;
      return toReturn;
    }
    toReturn.params[paramIndex] = inputString.substring(0, spaceIndex).toInt();
    inputString = inputString.substring(spaceIndex + 1);
    inputString.trim();
    spaceIndex = inputString.indexOf(' ');
    paramIndex++;
  }

  // Get the last parameter since it doesnt have a trailing space
  if (inputString.substring(spaceIndex + 1).toInt() == 0) {
    toReturn.validity = 2;
    return toReturn;
  }
  toReturn.params[paramIndex] = inputString.substring(spaceIndex + 1).toInt();

  // Count number of params // Probably not needed, maybe delete later on
  for (int i = 0; i < MAXPARAMS; i++) {
    if (toReturn.params[i] == PLACEHOLDER) {
      toReturn.paramCount = i;
      break;
    }
  }
  toReturn.validity = 0;
  return toReturn;
}

bool validate(command &cmd) {
  /*
  4 = wrong number of params
  5 = params out of range
  6 = unknown command
  */

  // Used ignore case but that might not be wanted behavior
  if (cmd.type.equals("pwmFunc")) {
    // Needs 2 params
    if (cmd.paramCount != 2) {
      cmd.validity = 4;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15) {
      cmd.validity = 5;
      return false;
    }
    // Param 2 is between 0 and 1600
    if (cmd.params[1] < 0 || cmd.params[1] > 1600) {
      cmd.validity = 5;
      return false;
    }
    // No issues found, must be valid
    return true;
  } else if (cmd.type.equals("pwmDuty")) {
    // Needs 2 params
    if (cmd.paramCount != 2) {
      cmd.validity = 4;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15) {
      cmd.validity = 5;
      return false;
    }
    // Param 2 is between 000(.)00 and 100(.)00
    if (cmd.params[1] < 0 || cmd.params[1] > 10000) {
      cmd.validity = 5;
      return false;
    }
    // No issues found, must be valid
    return true;
  } else if (cmd.type.equals("pwmGetVal")) {
    // Needs 1 param
    if (cmd.paramCount != 1) {
      cmd.validity = 4;
      return false;
    }
    // Param 1 is between 0 and 15
    if (cmd.params[0] < 0 || cmd.params[0] > 15) {
      cmd.validity = 5;
      return false;
    }
    // No issues found, must be valid
    return true;
  } else {
    Serial1.println("Invalid command type");
    cmd.validity = 6;
    return false;
  }
}

String sendCommand(command cmd) {
  // Used ignore case but that might not be wanted behavior
  if (cmd.type.equals("pwmFunc")) {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Convert param 2 to uint16_t
    uint16_t p2 = (uint16_t)cmd.params[1];
    // Send command
    return pwmFunc(p1, p2);
  } else if (cmd.type.equals("pwmDuty")) {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Convert param 2 to uint16_t
    uint16_t p2 = (uint16_t)cmd.params[1];
    // Send command
    return pwmDuty(p1, p2);
  } else if (cmd.type.equals("pwmGetVal")) {
    // Convert param 1 to uint8_t
    uint8_t p1 = (uint8_t)cmd.params[0];
    // Send command
    return pwmGetVal(p1);
  }
  return "Command not found";
}

void loop() {
  // Ask for command
  //Serial1.println("Testing");
  if (!promptSent) {
    Serial1.println("Enter a command:");
    promptSent = true;
  }

  if (Serial1.available() > 0) {

    // Read in line
    String inputString = Serial1.readStringUntil('\n');

    command currentCommand = parse(inputString);

    //------------------------------
    Serial1.print("Type: ");
    Serial1.println(currentCommand.type);
    if (currentCommand.paramCount > 0) {
      Serial1.print("Params: ");
      for (int i = 0; i < currentCommand.paramCount; i++) {
        Serial1.print(currentCommand.params[i]);
        Serial1.print(" ");
      }
      Serial1.println();
    } else {
      Serial1.println("No params");
    }
    //------------------------------

    if ((currentCommand.validity != 0) && (currentCommand.validity != -1)) {
      Serial1.println("Some issue with command parsing");
      //------------------------------
      Serial1.print("Error: ");
      Serial1.println(currentCommand.validity);
      //------------------------------
      promptSent = false;
      return;
    }

    if (validate(currentCommand)) {
      Serial1.println("Validated Successfully");
    } else {
      Serial1.println("Error during validation");
      //------------------------------
      Serial1.print("Error: ");
      Serial1.println(currentCommand.validity);
      //------------------------------
    }

    promptSent = false;
  }
}