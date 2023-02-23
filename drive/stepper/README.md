# Notes

## INITIALIZATION COMMANDS
- ## POR (Poll With Response)
    ### ***Description***
    >This command is the same as POL except that it response always has the same format. This might be easier to parse than the POL command for some host controllers.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | POR<br>SD 05<br>SN na | Immediate<br>Class A<br>27 (0x1B)| NONE | NONE | NONE |

    ### ***Example***
    Poll with command number\
    @16 27 (CR)
    ### ***Response***
    Polling Status Word
    ### ***Response Example***
    Response with status\
    \# *10 0000 2000 (CR)*
- ## CPL (Clear Poll)
    ### ***Description***
    >This is a complement to the Poll (POL) command. This command is used to clear the
    Polling Status Word (PSW) bits (See Polling Status Word (PSW) in User Manual for bit
    definitions). When a status bit is set (“1”) it will remain set until a Clear Poll (CPL)
    command is sent with the same bit set in its Clear Status Word parameter.

    >For example, if a POL command gets back a Polling Status Word(PSW) of “0x2000”, bit
    13 set (Program completed), of the PSW is set. To reset bit 13, the Clear Status Word
    parameter must be set to “0x2000”. This will cause bit 13 to be re-set (“0”). All other bits in the PSW will be left unchanged if the corresponding clear bit is not set. New
    occurrences since the last poll will NOT be cleared (the PSW is double buffered). That
    is, the information must be read before it is cleared.
    See Status Words in User Manual for more details.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | CPL | Immediate<br>Class A<br>1 (0x1)<br>2 words | Clear Status Word  | U16 | 0 to 65535 |

    ### ***Example***
    Clear only Bit #13 set in the Polling Status\
    Word (Decimal 8192 = 0x2000 in Hexadecimal)\
    *@16 1 8192 (CR)*\
    Clear all the bits set in the Polling Status Word.\
    *@16 1 65535 (CR)*
    ### ***Response***
    ACK only
- ## RIS (Read INternal Status Word)
    ### ***Description***
    >The Internal Status Word (ISW) is used in the device to keep track of different conditions that are present in the motor. The Internal Status Word (ISW) can be cleared
    using the Clear Internal Status (CIS) command.
    See Status Words in User Manual for bit definitions.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | RIS | Immediate<br>Class A<br>20 (0x14)<br>1 words | NONE | NONE | NONE |

    ### ***Example***
    Read back the Internal Status Word\
    *@16 20 (CR)*
    ### ***Response***
    Internal Status Word (ISW)
    ### **Response Example**
    Indicates Input #1, 2, 3 “High”, Last
    Calculation was Zero and Index Sensor
    was found.
    \# *10 0014 00F3 (CR)*
- ## AHC (Anti-Hunt Constants)
- ## AHD (Anti-Hunt Delay)
- ## AHM (Anti-Hunt Mode)
- ## BRT (Baud Rate)
    ### ***Description***
    >This command is used to change the devices baud rate.\
    Negative values set the hard divisor for odd baud rates.\
    Divisor = 2.5MHz/(Baud Rate) -1
    If this command is sent in Immediate Mode, the response will be at the new baud rate.\
    See Technical Document QCI-TD053 Serial Communications on our website for details.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | BRT | Program<br>Class D<br>174 (0xAE)<br>2 words<br>Thread 1&2 | Speed – Character bit rate<br> NOTE: Negative, indicates bit period. | S16 | 3 = 300 (baud)<br>12 = 1200<br>    24 = 2400<br>48 = 4800<br>96 = 9600<br>192 = 19200<br>288 = 28800<br>384 = 38400<br>576 = 57600 (Default)<br>1000 = 100000<br>1152 = 115200<br>2304 = 230400*<br>2500 = 250000** <br>or<br>-11 to -32767 |

    ### ***Example***
    Set the baud rate for 57.6K.\
    *@16 174 576 (CR)*
    ### ***Response***
    ACK only
- ## CER(Command Error Recovery)
    ### ***Description***
    >CER sets up options for recovery from a Command Error. Command Errors occur
    when the device is programmed to do something it cannot due. For example, a
    Command Error will occur if the servo is asked to move 1000 revs is 1ms. The
    required velocity is greater than 4000RPM. By default, Command Errors halt the
    program and set bit 12 in the Polling Status Word (PSW). When CER is used, the user
    can, instead, designate a recovery program to load and run anytime a Command Error
    occurs.\
    \
    A value of 0 will disable this function. A value of –1 will run the code from non-volatile
    memory location 0.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | CER<br>SD 08 | Program<br>Class D<br>65 (0x41)<br>2 words<br>Thread 1 | Process | S16 | 0 = Do Nothing<br>-1= Load and Run<br>Program @NV Mem Adr 0<br><br>#### = Load and Run Program @<br>indicated NV Mem Adr |

    ### ***Example***
    Recover command error from nonvolatile memory location 1000
    *@16 65 1000 (CR)*
    ### ***Response***
    ACK only
- ## DIR(Direction)
    ### ***Description***
    >Establishes the direction in which the servo will turn given a motion in a positive
    direction. Normally the device will turn Clockwise (when viewed from the shaft end of
    servo) when a positive distance or velocity number is used. A negative number will
    cause the servo to turn counter clockwise. Using the Direction command, this default
    operation can be reversed.\
    \
    WARNING: DIR can only be used when the device is being initialized and before the
    Go Closed Loop (GCL) command is issued. If DIR is used after GCL the unit will fault
    with a sequence error. Typically this command is only edited within the device
    Initialization Wizard while editing the initialization file "Factory Default Initialization.qcp".
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | DIR | Program<br>Class D<br>184 (0xB8)<br>2 words<br>Thread 1 | Mode | S16 | 0 = Normal (CW) Default<br>1 = Reverse (CLW) |

    ### ***Example***
    Clockwise\
    *@16 184 0 (CR)*
    ### ***Response***
    ACK only
- ## DMD(Disable Motor Driver)
    ### ***Description***
    >Disables the motor driver. The device will be unable to move when attempting any
    motion command. This is a software disable that can be overcome by the Enable Motor
    Driver (EMD) command, or by setting the Motor Constants (MCT).
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | DMD | Program<br>Class D<br>228 (0xE4)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Disable the Motor Driver electronics\
    *@16 228 (CR)*
    ### ***Response***
    ACK only
- ## EMD(Enable Motor Driver)
    ### ***Description***
    >Enables the device motor driver. The driver is by default enabled, this command is only
    required if the driver has been disabled using the Disable Motor Driver (DMD)
    command or disabled by the Kill Motor operation or by an over voltage condition.\
    \
    If the user is enabling the unit after it has been disabled, and any potential exists that
    the motor shaft has been rotated since the motor was disabled, then the user should
    make sure that the motor target and position are made equal before enabling the motor
    so as to prevent the motor from sudden rotations. This may be accomplished using
    either the Zero Target Position (ZTP) or the Set Target Position (STP) commands. The
    ZTP sets both the Target and Position to zero, while the STP maintains the actual
    motor Position information, and merely sets the target to the current position so that no
    error exists when the motor is enabled to prevent unwanted motion. If it is necessary to
    restore the motor to its prior location, then save the target value before doing a STP
    command, and then do an absolute move using to the saved Target position.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | EMD | Program<br>Class D<br>227 (0xE3)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Enable the motor driver\
    *@16 227 (CR)*
    ### ***Response***
    ACK only
- ## ERL(Error Limits)
    ### ***Description***
    >The Error Limits command sets allowable position error before the Holding and/or
    Moving Error bits are set in various status words (see Status Words in User Manual for
    bit definitions). The Delay to Holding parameter specifies the time the device waits
    after a move is completed before it goes from moving torque to holding torque (see
    Torque Limits (TQL) command).\
    \
    A special “Drag” or clutch mode may be implemented by setting the error limits to
    negative values. The absolute value of the limit is still used to generate Holding and
    Moving status conditions, but the target is not allowed to get farther than the respective
    error limit from the servo position. This creates a slip clutch effect. NOTE: Since the
    Holding and Moving status bits are set in the Internal Status Word (ISW), these bits
    should be disabled in the kill motor condition commands KMC and/or KMX. See Error
    Limits and Drag Mode in User Manual for more details.\
    \
    Note: If you are using QuickControl with the Drag Mode box checked, it will
    automatically (internally) negate the error limits for you.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | :---: | --- |
    | ERL | Program<br>Class D<br>151 (0x97)<br>4 words<br>Thread 1&2 | Moving Limit<br><br>Holding Limit<br><br>Delay to<br>Holding (ticks) | S16<br><br>S16<br><br>S16<br><br> | -32768 to 32767, Default = 0<br>QuickControl Default = 20000<br>-32768 to 32767, Default = 0<br>QuickControl Default = 20000<br>0 to 65535 ,Default = 100 ticks<br>QuickControl Default = 120ms |

    ### ***Example***
    Allow 500 counts of error while moving
    and 100 counts of error when holding
    position. Allow 120 milliseconds before
    going into Hold mode with its tighter
    error limit.\
    *@16 151 500 100 1000 (CR)*
    ### ***Response***
    ACK only
- ## GCL(Go Closed Loop)
    ### ***Description***
    >Puts the device into closed loop operation. This is typically only done one time during
    initialization. This command is used to put device into closed loop mode if the unit has
    been placed into open loop mode. This command sets the phase relationship between
    the rotor and the encoder for closed loop operation. (See Initialization in the User
    Manual for more information.)
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | GCL | Program<br>Class D<br>142 (0x8E)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Put device into closed loop mode\
    *@16 142 (CR)*
    ### ***Response***
    ACK only
- ## GOC(Gravity Offset Constant)
    ### ***Description***
    >Establishes a value that compensates for the effects of gravity on the load that the
    servo is driving. This servo control parameter is designed to neutralize the effect of
    gravity on mechanisms that operate in other than horizontal orientation. It enables the
    servo control to operate consistently in both directions of servo rotation by creating a
    torque offset that counters the torque required to hold the load in position. The offset
    value is in torque units the same as the Torque Limits (TQL) command.\
    \
    Depending on the direction of the torque applied to the servo shaft, the value can be
    set to a negative or positive value.\
    \
    For QuickControl, if the Edit GOC dialog box "Normal" option is checked, QuickControl
    will automatically translate the percent torque to the native torque units at time of
    download.\
    \
    Note: The Gravity offset value allows the system to smoothly switch in and out of AntiHunt operation by not requiring the error to build up or the integrator to ramp up to
    provide the torque needed to hold the load when switching from open loop to closed
    loop operation (given an appropriate value for the Gravity offset has been configured.)

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | GOC | Program<br>Class D<br>237 (0xED)<br>2 words<br>Thread 1 | Gravity Offset | U16 | 32767 to 32767<br>Default: 0 |

    ### ***Example***
    Set the Gravity Offset to 35% Torque for a 23-3 servo\
    *@16 237 7000 (CR)*
    ### ***Response***
    ACK only
- ## GOL(Go Open Loop)
    ### ***Description***
    >Puts the device into open loop operation. This is the default servo power up mode. This
    command is used during servo initialization to aid in aligning the rotor to the encoder.\
    \
    The command can also be used to force the servo into open loop mode. This is not
    recommended for normal operation, as the system performance is severely degraded.\
    \
    If the servo is in Dual Loop Control (DLC) operation when this command is
    encountered, it is forced back into Single Loop Control
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | G0L | Program<br>Class D<br>143 (0x8F)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Put device into closed loop mode\
    *@16 143 (CR)*
    ### ***Response***
    ACK only
- ## KDD(Kill Disable Driver)
    ### ***Description***
    >Disables the motor driver, when a Kill Motor Condition is met. If the device is moving, it
    will stop immediately in a rapid manner. The motor will be unable to move until reenabled using the Enable Motor Driver (KMD) command. This is the default setting for the servo.\
    \
    See Technical Document QCI-TD052 Shutdown and Recovery on our website for
    details.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | KDD | Program<br>Class D<br>183 (0xB7)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Disable the Motor Driver electronics when Kill Motor Conditions are met\
    *@16 183 (CR)*
    ### ***Response***
    ACK only
- ## KED(Kill Enable Driver)
    ### ***Description***
    >Causes the device to leave the motor drivers enabled when a Kill Motor Condition is
    met. Normally the motor driver is disabled with a Kill Motor Condition, this command
    can be used to leave the driver enabled if continuing operation is required.\
    \
    In order for this command to function, the device must be set up for multi-tasking
    operation. Without multi-tasking, the driver will be disabled when a Kill Motor Condition
    occurs.\
    \
    See Technical Document QCI-TD052 Shutdown and Recovery on our website for
    details.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | KED | Program<br>Class D<br>182 (0xB6)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Leave the motor driver enabled\
    *@16 182 (CR)*
    ### ***Response***
    ACK only
- ## KMC(Kill Motor Conditions)
    ### ***Description***
    >The Kill Motor Conditions allows the user to select what conditions will allow a
    controlled shutdown of the unit. The Condition Enable word selects which bits in the
    Internal Status Word (ISW) will be evaluated (See Internal Status Word (ISW) in User
    Manual for bit definitions). Conditions are enabled by setting a “1” in the desired bit
    position of the Condition Enable binary word. See KMX for more kill motor conditions.\
    \
    The Condition State word allows the user to specify the state of the selected conditions
    that will cause the device to do a controlled shutdown. Note: Over-voltage is always
    enabled whenever the driver is enabled to protect the drivers from over voltage. An
    over-voltage condition will always disable the drivers regardless of the of Kill Enable
    Drivers state.\
    \
    See Technical Document QCI-TD052 Shutdown and Recovery on our website for
    details.\
    \
    Default has only Over Temperature enabled.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | KMC | Program<br>Class D<br>167 (0xA7)<br>3 Words<br>Thread 1 | Condition Enable<br>Condition State | U16<br>U16 | 0 to 65536<br>0 to 65535 |

    ### ***Example***
    Shut down servo if any of the following conditions are met:\
    I/O#1 LOW (bit 4)\
    Over Temp (bit 7)\
    Moving Error (bit 8)\
    NOTE: Over Temp TRUE = 0.\
    Enable = $2^4 + 2^7 + 2^8 = 400$\
    State = $2^4*0 + 2^7*0 + 2^8*1=256$\
    *@16 167 400 256 (CR)*
    ### ***Response***
    ACK only
- ## KMX(Kill Motor Conditions Extended)
    ### ***Description***
    >The Extended version of Kill Motor Conditions (KMC) provides 3 status and I/O words
    of conditions that may be selected to allow a controlled shutdown of the unit. The three
    Condition Enable words selects which bits in the respective registers will be evaluated;
    a “1” state is set for each bit which is to be evaluated, and a “0” bit for those bits which
    are to be ignored. The three words are ISW, IS2 and XIO. See User Manual for bit
    definition.\
    \
    The Condition State word allows the user to specify the state of the selected conditions
    that will cause the device to do a controlled shutdown. Note: Over-voltage is always
    enabled whenever the driver is enabled to protect the drivers from over voltage. An
    over-voltage condition will always disable the drivers regardless of the of Kill Enable
    Drivers state.\
    \
    See Technical Document QCI-TD052 Shutdown and Recovery on our website for
    details.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | :---: | --- |
    | KMX<br><br>SN n/a<br>SD 05 | Program<br>Class D<br>220 (0xDC)<br>7 words<br>Thread 1 | Condition Enable ISW<br>Condition State ISW<br>Condition Enable IS2<br>Condition State IS2<br>Condition Enable XIO<br>Condition State XIO | U16<br>U16<br>U16<br>U16<br>U16<br>U16 | 0 to 65535<br>0 to 65535<br>0 to 65535<br>0 to 65535<br>0 to 65535<br>0 to 65535 |

    ### ***Example***
    Shut down servo if any of the following conditions are met:\
    I/O#1 LOW (bit 4)\
    Over Temp (bit 7)\
    Moving Error (bit 8)\
    NOTE: Over Temp TRUE = 0.\
    Enable = $2^4 + 2^7 + 2^8 = 400$\
    State = $2^4*0 + 2^7*0 + 2^8*1=256$\
    *@16 167 400 256 0 0 0 0 (CR)*
    ### ***Response***
    ACK only
- ## KMR(Kill Motor Recovery)
    ### ***Description***
    >Kill Motor Recovery sets up options for recovery from a device shut down. The Kill
    Motor Conditions (KMC) establishes conditions that will cause the device to shut down.
    Using Kill Motor Recovery the device can perform a standard or user defined process
    for re-initializing the device. User programs can be executed that have been previously
    stored in the non-volatile memory. (See Kill Motor Conditions for more detail).
    Three options available:
    1. “0” – Default: No recovery program designated. The device drops out of any
    motion or program that is currently executing and goes into an idle state. The
    drivers are disabled. At this point the device will sit with no current to the device.
    2. “-1” – Normal operation: -1 is a special parameter value indicating to run the
    initialization program from non-volatile memory location “0”
    3. “####” – Normal operation: The routine located at #### is loaded and executed.
    NOTE: If QuickControl is polling the device when the shutdown occurs, it will display
    the cause of the fault providing the KMR program does not clear it too quickly.
    Because of this, it is recommended that the KMR program have short delay in it before
    clearing the fault. QCI suggests 100ms/axis.
    >See Technical Document QCI-TD052 Shutdown and Recovery on our website for
    details.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | KMR | Program<br>Class D<br>181 (0xB5)<br>2 words<br>Thread 1 | Process | S16 | 0 = Do Nothing<br>-1 = Load and Run Program<br>@ NV Mem adr 0.<br>#### = Load and Run Program<br>@ indicated NV Mem adr. |

    ### ***Example***
    After motor shutdown load and run "Fault Recovery" program which is stored at 542.\
    \
    NOTE: In QuickControl, the user only needs to specify the program name. The address is calculated automatically.\
    *@16 181 542 (CR)*
    ### ***Response***
    ACK only
- ## LVP(Low Voltage Processor Trip)
    ### ***Description***
    >This command is only usable with units that provide separate power supply inputs for
    the processor and for the driver sections. This command allows the monitoring of the
    processor power supply for low voltages in the same way that a Low Voltage Trip (LVT)
    command monitors the driver (or, for single supply motors, the main power supply).\
    \
    This command sets the input voltage that will trigger a Low Voltage status (Bit #14 in
    the Internal Status Word (ISW)) and subsequently the Power Low Recovery (PLR)
    routine (if configured). When a Low Voltage Processor Trip occurs the low voltage trip
    values, both driver and processor; are overwritten to zero to prevent multiple triggering.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | LVP | Program<br>Class D<br>131 (0x83)<br>2 words<br>Thread 1 | Voltage | U16 | 0 = Don't Check 10 to 48<br>Default: 0 |

    ### ***Example***
    Set LVP to 10 volts\
    *@16 131 10 (CR)*
    ### ***Response***
    ACK only
- ## LVT(Low Voltage Trip)
    ### ***Description***
    >This command sets the input voltage (or driver Input voltage for units that have dual
    input power supplies) that will trigger a Low Voltage status (Bit #14 in the Internal
    Status Word (ISW)) and subsequently the Power Low Recovery (PLR) routine (if
    configured). When a Low Voltage Trip occurs the low voltage trip values associated
    with the Low Voltage Trip and Low Voltage Processor Trip commands are overwritten
    to zero to prevent multiple triggering.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | LVT | Program<br>Class D<br>212 (0xD4)<br>2 words<br>THread 1 | Voltage | U16 | 0 = Don't check<br>10 to 48<br>Default: 10V |

    ### ***Example***
    Set shut down at 10 volts\
    *@16 212 10 (CR)*
    ### ***Response***
    ACK only
- ## MTT(Maximum Temperature Trip)
    ### ***Description***
    >Sets the temperature at which the device will shut down the servo. This is used to
    prevent internal over-heating of the servo electronics. The value is entered in degrees
    Celsius integer units. (Example “70” for 70 degrees Celsius). The maximum
    temperature error condition is OR-ed with the motor driver over temperature condition.
    Either active will cause an Over Temperature status condition in the Internal Status
    Word. The temperature can be read using the ANALOG READ INPUT command.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | MTT | Program<br>Class D<br>214 (0xD6)<br>2 words<br>Thread 1 | Temperature (C) | U16 | 0 = Don't check<br>1 to 80<br>Default: 0 |

    ### ***Example***
    Set Servo to give an error at 70 degrees C\
    *@16 214 70 (CR)*
    ### ***Response***
    ACK only
- ## OVT(Over Voltage Trip)
    ### ***Description***
    >Sets the voltage at which the device will cause a motor shutdown. This command is
    mainly used to prevent over-voltage from the power regenerated during deceleration.
    The voltage value is entered in integer units (example: “48” for 48 volts). If an overvoltage condition is detected, a motor shutdown is executed that disables the motor
    driver to reduce regenerated power flowing into the power supply input which boosts
    the supply voltage.\
    \
    NOTE: The Kill Enable Driver (KED) command does not allow the motor driver to stay
    enabled when an Over Voltage Trip occurs. This condition always disables the motor
    driver.\
    \
    The motor driver is disabled when this condition occurs and must be re-enabled using
    the Enable Motor Driver (EMD) command or by re-writing the Motor Constants (MCT).\
    \
    The factory default is set at 52 volts. A power supply voltage that exceeds 52 volts may
    cause the motor to shutdown at power up. Unregulated power supplies with excessive
    voltage ripple can cause an over voltage trip, even though an average reading meter
    may report the voltage as within specification. The over voltage trip may also activate
    when doing rapid decelerations with large inertias, or using the device as a clutch
    without using a Clamp Module between the device and the power supply. (Note: the IGrade SilverDust units have the clamp built in.)\
    \
    In QuickControl, if Automatic is selected OVT will be set to 4V above the voltage used
    by the most recent MCT command. This is only determined at time of download.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | OVT | Program<br>Class D<br>213 (0xD5)<br>2 words<br>Thread 1 | Voltage | U16 | 1 to 53 (52 = Default) |

    ### ***Example***
    Shut down the Motor if the input voltage  exceeds 52 volts\
    *@16 212 52 (CR)*
    ### ***Response***
    ACK only
- ## PLR(Power Low Recovery)
    ### ***Description***
    >This command designates which program will run if the power supplies voltage drops
    below that specified by the Low Voltage Trip (LVT) command or Low Voltage
    Processor Trip (LVP) commands.\
    The QuickControl edit PLR dialog box has four options:
    1. Load and Run Program - Select a PLR program.
    2. Load and Run Absolute Address - Enter the non-volatile memory address of the
    program you want to load and run for the PLR.
    3. Load and Run Program at NV Memory Address 0 - Load and run the program
    stored at 0. By default this is the initialization program.
    4. Do Nothing – This default state indicates that no recovery program has been
    designated. The device drops out of any motion or program that is currently
    executing and goes into an idle state. Note: Bit 14 of the ISW word is set by the
    low voltage activity, and, if enabled, the Kill Motor Recovery will handle this
    condition.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PLR | Program<br>Class D<br>208 (0xD0)<br>2 words<br>Thread 1 | Process | S16 | 0 = Do Nothing<br>-1 = Load and Run Program<br>@ NV Mem adr 0.<br>#### = Load and Run Program<br>@ indicated NV Mem adr. |

    ### ***Example***
    If power low condition exists load and run
    "Program Low Recovery" program which
    is stored at 568.\
    *@16 208 568 (CR)*

    ### ***Response***
    ACK only
- ## PRO(Protocol)
    ### ***Description***
    >Allows the user to select the desired communications protocol.\
    \
    If this command is sent in Immediate Mode, the response will be in the new protocol.\
    \
    The lower byte of the parameter selects the desired protocol, while the upper byte
    selects the serial configuration. Note, for QCI 9 bit and DMX512 protocols, the serial
    configuration must be 2 stop bits and no parity.\
    \
    See Technical Document "QCI-TD053 Serial Communications" on our website for more
    details on this command.\
    \
    See Application Note "QCI-AN038 Modbus Protocol" for details on communicating with
    a Modbus® device including an example program.\
    \
    See Application Note "QCI-AN045 DMX512 Protocol" for details on communicating
    with a DMX device.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PRO | Program<br>Class D<br>185 (0xB9)<br>2 words<br>Thread 1&2 | Parity & Stop \| Mode | S16 | Mode (lower byte)<br>0 = 9-bit<br>1 = 8-bit(Default) <br>2 = Modbus<br>3 = DMX512<br><br>Parity & Stop<br>Bit 15: Stop bits, 0 =>2bits, 1=>1 bit<br>Bit 14: Enable Parity, 0=none, 1=enabled<br>Bit 13: Odd Parity, 0=even, 1=odd |

    ### ***Example***
    Select the 8-Bit ASCII Protocol
    *@16 185 1 (CR)*
    ### ***Response***
    ACK only
- ## SCF(S-Curve Factor)
    ### ***Description***
    >The shape of motion profile acceleration can be set from linear to full s-curve. This
    command can be set at any time except for during a motion. SCF only affects the
    basic motion commands and their register based deviations (MRT, MRV, …).\
    \
    SCF is not available in the Step & Direction (i.e. SSD) , Profiled Move (i.e. PMC) , Input
    Mode (i.e. PIM) or the Velocity modes (i.e. VMP).\
    \
    See S-Curve in User Manual for more information.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | SCF | Program<br>Class D<br>195 (0xC3)<br>2 words<br>Thread 1 | Factor | S16 | 0 = Trapezoidal<br>1 to 32766 = s-curve<br>32767 = Full s-curve<br>Default: 0 |

    ### ***Example***
    Use some S-Curve.\
    *@16 195 10813 (CR)*
    ### ***Response***
    ACK only
- ## SIF(Serial Interface)
    ### ***Description***
    >Allows the user to select between RS-232 and RS-485 serial communications
    hardware interface. This command is usually used at power up as part of the
    initialization program. Care should be taken when using this command, as
    communications may be lost if the host controller is not compatible with the new
    hardware setting.\

    QuickControl will automatically set this parameter at download if the box "Set to SIF
    currently being used by device" is checked. At download, QuickControl asks the
    device whether it is in RS-232 or RS-485 and then sets the SIF command accordingly.\
    \
    For RS-232 multi-drop, uncheck the box, set SIF to RS-232 and set ACK Delay (ADL)
    to some non zero value (i.e. 5).\

    If this command is sent in Immediate Mode, the response will be in the new interface.\

    See Technical Document QCI-TD053 Serial Communications on our website for
    details.

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | SIF | Program<br>Class D<br>186 (0xBA)<br>2 words<br>Thread 1&2 | Mode | U16 | 0 = RS-232 (Default)<br>1 = RS-485 |

    ### ***Example***
    Set up the device to use RS-232 for the serial interface\
    *@16 186 0 (CR)*
    ### ***Response***
    ACK only
- ## SLC(Single Loop Control)
    ### ***Description***
    >Configures the device to run in the standard single loop control mode. Encoder
    information for commutation, position, velocity and acceleration control is derived from
    the Internal Encoder.\
    \
    If a motion is running, the servo Trajectory Generator must be shut down prior to
    executing this command or an error will result.\
    \
    When entering single loop control, the device sets the current “Target” to the “Current
    position” (Internal Position from the Internal Encoder).\
    \
    By default, the device starts up in Single Loop Control mode.\
    \
    See the Dual Control Loop (DLC) command for cases where external encoder position
    control is required. Switching between Single Loop and Dual Loop modes usually
    requires changing the control loop tuning.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | SLC | Program<br>Class D<br>244 (0xF4)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Configure for Single Loop Control\
    *@16 244 (CR)*
    ### ***Response***
    ACK only
- ## TQL(Torque Limits)
    ### ***Description***
    >This command sets the torque limits for the different operating modes of the servo. The
    unit may be in either Open Loop or Closed Loop mode, and in either Moving or Molding
    mode. The four parameters supplied set the limits on the output torque for all four
    combinations: Closed Loop Holding, Closed Loop Moving, Open Loop Holding, and
    Open Loop Moving.\
    \
    See Technical Document QCI-TD051 Torque Control on our website for details on this
    command.

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | TQL | Program<br>Class D<br>149 (0x95)<br>5 words<br>Thread 1&2 | Closed Loop Holding<br>Closed Loop Moving<br>Open Loop Holding<br>Open Loop Moving | U16<br>U16<br>U16<br>U16 | 0 to 32767<br>0 to 32767<br>0 to 32767<br>0 to 32767 |

    ### ***Example***
    Set torque to:\
    Closed Loop Holding 75%\
    Closed Loop Moving 100%\
    Open Loop Holding 30%\
    Open Loop Moving 50%\
    *@16 149 15000 20000 6000 10000(CR)*
    ### ***Response***
    ACK only
- ## VLL(Velocity Limits)
    ### ***Description***
    >This command sets a limiter value within the servo control loop so as to limit the
    maximum velocity of the servo system. Both Moving and Holding limits are provided.
    Note: Moving is defined by anytime the motor is in motion (Trajectory Generator
    active) and during the settling time as defined in the Error Limits (ERL)
    command.\
    \
    Note: The trajectory will continue to change at the commanded rate, even if the
    physical motor has been limited by the velocity limit command. Use the Error
    Limits (ERL) command to either enable the “drag” mode, or combine with the
    error recovery commands to implement a shutdown if needed by the application.\
    Note: Bit 1 is set in the IS2 word if the velocity limit actually engages. This may be used
    to end a motion or to trigger an error recovery.\
    Note: The Gravity Offset Constant (GOC) is added following the velocity loop. Care
    must be exercised to verify that the GOC is not set so high as to override the
    velocity limit.\
    \
    The velocity limits are given in SilverLode Actual Velocity Units (SAV) (see User
    Manual for details).\
    \
    NOTE: The lower limit is 455 for motion to still be allowed.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | VLL<br>SN n/a<br>SD 08 | Program<br>Class D<br>69 (0x45)<br>5 words<br>Thread 1&2 | Moving Limit<br>Holding Limit | S16<br>S16 | 0 ot 32767 SAV<br>0 ot 32767 SAV |

    ### ***Example***
    Set Moving Limit to 133333 cps and Holding Limit to 13330 cps\
    *@16 69 16384 1638 (CR)*
    ### ***Response***
    ACK only

## MOVEMENT COMMANDS
- ## HLT(Halt)
    ### ***Description***
    >This command immediately shuts down any motion in progress (hard stop), disables the
    single step mode, and then causes the motor to load and run the Kill Motor Recovery
    program. (see Kill Motor Recovery (KMR) command for details.)\
    \
    This command stops the execution of all commands, programs and motions. When
    executed, it will stop any command or program in process. Unless the Kill Motor
    Recovery Program has been designated and the Kill Enable Driver (KED) has been
    enabled, the motor driver will be disabled. This allows the motor shaft to be manually
    spun.\
    \
    Bit #10 of the Internal Status Word (ISW) is “set” to indicate that a Halt command was
    sent. This is useful for determining the cause of the motor shut down when using an
    internal Kill Motor Recovery program.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | HLT | Immediate<br>Class A<br>2 (0x2)<br>1 word | NONE | NONE | NONE |

    ### ***Example***
    Halt any command, program or motion in process\
    *@16 2 (CR)*
    ### ***Response***
    ACK only
- ## MAT(Move Absolute, Time Baised)
    ### ***Description***
    >Move Absolute initiates a move to an absolute position.\
    \
    See Basic Motion and Programming Fundamentals in User Manual for more details.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Scaling in User Manual for more details on native time units

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | MAT | Program<br>Class D<br>176 (0xB0)<br>9 words<br>Thread 1 | Position<br>Acceleration Time<br>Total Time<br>Stop Enable<br>Stop State | S32<br>U32<br>U32<br>S16/U16<br>S16/U16 | $-2^{31}\ to\ 2^{31}$<br>0 to 65534 (7.86s)<br>$2\ to\ 2^{31}$<br>See above<br>See above |

    ### ***Example***
    Move the device to position 200 in 1.0
    seconds with a 0.1 second acceleration.\
    *@16 176 200 83 8333 0 0(CR)*
    ### ***Response***
    ACK only
- ## MAV(Move Absolute, Velocity Baised)
    ### ***Description***
    >Move Relative initiates a distance move relative to the current target position.\
    \
    See Basic Motion and Programming Fundamentals in User Manual for more details.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Scaling in User Manual for more details on native time units

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | MRT | Program<br>Class D<br>176 (0xB0)<br>9 words<br>Thread 1 | Position<br>Acceleration<br>Total Time<br>Stop Enable<br>Stop State | S32<br>U32<br>U32<br>S16/U16<br>S16/U16 | $-2^{31}\ to\ 2^{31}$<br>$1\ to \ 2^{30}$<br>$2\ to\ 2^{31}$<br>See above<br>See above |

    ### ***Example***
    Move the device 4000 counts from its
    current position. Do the move in 1 second
    with a 0.1 second acceleration.\
    *@16 177 4000 833 8333 0 0 (CR)*
    ### ***Response***
    ACK only
- ## MRT(Move Relative, Time Baised)
    ### ***Description***
    >Move Relative initiates a distance move relative to the current target position.\
    \
    See Basic Motion and Programming Fundamentals in User Manual for more details.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Scaling in User Manual for more details on native time units

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | MRT | Program<br>Class D<br>177 (0xB1)<br>9 words<br>Thread 1 | Distance<br>Ramp Time<br>Total Time<br>Stop Enable<br>Stop State | S32<br>U32<br>U32<br>S16/U16<br>S16/U16 | $-2^{31}\ to\ 2^{31}$<br>0 to 65534 (7.86s)<br>$2\ to\ 2^{31}$<br>See above<br>See above |

    ### ***Example***
    Move the device 4000 counts from its
    current position. Do the move in 1 second
    with a 0.1 second acceleration.\
    *@16 177 4000 833 8333 0 0 (CR)*
    ### ***Response***
    ACK only
- ## MRV(Move Relative, Velocity Baised)
    ### ***Description***
    >Move Relative initiates a distance move relative to the current target position.\
    \
    Note, acceleration to given velocity must be less than 7.86 seconds. That is:
    Velocity/Acceleration < 7.86\
    \
    See Basic Motion and Programming Fundamentals in User Manual for more details.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Scaling in User Manual for more details on native acceleration and velocity units.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | MRV | Program<br>Class D<br>135 (0x87)<br>9 words<br>Thread 1 | Distance<br>Acceleration<br>Velocity<br>Stop Enable<br>Stop State | S32<br>U32<br>U32<br>S16/U16<br>S16/U16 | $-2^{31}\ to\ 2^{31}$<br>$1\ to\ 2^{30}$<br>$0\ to\ 2^{31}$<br>See above<br>See above |

    ### ***Example***
    Move -4000 counts from its current position at
    1000cps.\
    *@16 135 -4000 3865 8053064 0 0(CR)*
    ### ***Response***
    ACK only
- ## PMC(Profile Move Continuous)
    ### ***Description***
    >The Profile Move commands are distinct from the Motion commands in that the move
    parameters can be modified while the motion is in progress. A change in a move
    parameter updates the move immediately and can alter the move profile “real-time”.\
    \
    The Profile Move Continuous puts the device into a move that does not end unless
    explicitly commanded (i.e. VMP, PMX). During the move, any move parameter can be
    updated either by a Host controller using the serial interface or by an internal program
    (Multi-Tasking operation is required).\
    \
    With this feature, any motion profile shape can be accomplished by changing the
    appropriate parameter at the desired time. Five parameters are associated with this
    command. Each of the parameters is dedicated to a specified User Data Register.
    Modifying the contents of the Data Register modifies the parameter.\
    \
    The following table shows the list of the parameters and their associated Data Register:

    | Register | Description | Data Range | Comment |
    | --- | --- | --- | --- |
    | 20 | Position | $-2^{31}\ to\ 2^{31}$ | This is an “Absolute” destination value. |
    | 21 | Acceleration | $2\ to\ 2^{30}$ | Sets the acceleration rate that is used when increasing the move speed. |
    | 22 | Velocity | $0\ to\ 2^{31}$ | The maximum speed that is allowed during a move |
    | 23 | Deceleration | $2\ to\ 2^{30}$ | Sets the deceleration rate that is used when decreasing the move speed. |
    | 24 | Offset | $-2^{31}\ to\ 2^{31}$ | A distance value to move that is added to the current position when a “Stop Condition” is encountered |
    >Data Registers must be pre-loaded with the move parameters prior to issuing the Profile
    Move Continuous command.\
    \
    Profiles Moves begin immediately after executing the command (within 120 usec.). The
    motor is accelerated using the Acceleration parameter until the maximum Velocity is
    reached. Deceleration begins when the distance of the move is such that the Absolute
    Position is achieved at the same time the motor has decelerated to “0” velocity.
    Depending on the parameters the maximum velocity may never be reached (Triangle
    Move).\
    \
    During a Profile Move, the device is constantly recalculating its intermediate move
    values (every 120 usec.). This is done by taking the given move parameters, the current
    position and current velocity and adjusting what is required to hit the absolute position.\
    \
    This means that the device can even go from a Velocity Mode into a Profile Move
    without needing to stop first (Multi-Tasking operation is required). Remember that the
    move calculations are being done continually. Therefore, the parameters can be
    changed at any time and affect the motion in process.\
    \
    The Acceleration and Deceleration parameters should typically be no greater than a
    ratio of 100:1 of each other (one value is no greater than 100 times the other) for
    numerical stability. For higher ratios user must verify proper operation.
    The Position parameter can act as a Relative Distance value by using the Add To
    Register command to increase or decrease the Position value. (See Add To Register for
    more details)\
    \
    The Offset parameter is used to extend a move by the offset distance after a Stop
    Condition is encountered. In cases where a move needs to continue a prescribed
    distance past the point where a sensor triggers a stop, this parameter can be used to
    precisely control that offset distance to be moved. Note that the offset is automatically
    negative if the direction of motion is negative when the input is found. The Offset
    parameter allows trailing edge registration operations.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Profile Move Operation in User Manual for details.\
    \
    Note: The Profiled motion commands combined with the “Drag” mode of the Error Limits
    will allow the user to reach the destination smoothly even if the rotor is restrained or
    torque limited, once the over torque condition has been removed.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PMC | Program<br>Class D<br>240 (0xF0)<br>3 words<br>Thread 1 | Stop Enable<br>Stop State | S16/U16<br>S16/U16 | See above<br>See above |

    ### ***Example***
    Put the device into a continuous Profile
    move. Stop if Input #1 is high (“1”).\
    *@16 240 –1 1 (CR)*
    ### ***Response***
    ACK only
- ## PMO(Profile Move Override)
    ### ***Description***
    >The Profile Move Override command allows a Profile Move Continuous to end when the
    Position is achieved. Normally the Move Continuous will not end until explicitly stopped
    by a Stop Condition or another command. The Override provides a graceful way to end
    the move so that the entire motion is completed with the motor stopping at the defined
    position. PMO will also override all other motions, including Step and Direction, if multitasking is enabled.\
    \
    PMO operates exactly like the Profile Move command except that it does not wait for
    the previous motion to complete.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Profile Move Operation in User Manual for details.

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PMO | Program<br>Class D<br>249 (0xF9)<br>3 words<br>Thread 1 | Stop Enable<br>Stop State | S16/U16<br>S16/U16 | See above<br>See above |

    ### ***Example***
    End the current Profile move when at
    “Position”. Stop if Input #1 is high (“1”).\
    *@16 249 –1 1 (CR)*
    ### ***Response***
    ACK only
- ## PMV(Profile Move)
    ### ***Description***
    >The Profile Move command works identical to the Profile Move Continuous except that
    when the Position is achieved, the move ends and the trajectory generator goes
    inactive. All of the parameters including the position can be changed while the move is
    executing. Once the move has ended, changing the parameters will have no effect.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.\
    \
    See Profile Move Operation in User Manual for details.

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PMV | Program<br>Class D<br>241 (0xF1)<br>3 words<br>Thread 1 | Stop Enable<br>Stop State | S16/U16<br>S16/U16 | See above<br>See above |

    ### ***Example***
    Start a Profile move. Stop if Input #1 is high (“1”).\
    *@16 241 –1 1 (CR)*
    ### ***Response***
    ACK only
- ## PMX(Profile Move Exit)
    ### ***Description***
    >Exits the current Profile Move allowing the move to stop using the Deceleration
    parameter stored in Data Register #23. This command will work to stop any Motion,
    Profile Move or Mode (as long as register 23 has been initialized). The deceleration
    begins immediately and the profile destination will normally not be reached.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PMX | Program<br>Class D<br>242 (0xF2)<br>1 word<br>Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Exit the current move\
    *@12 242 (CR)*
    ### ***Response***
    ACK only
- ## PVC(Profile Velocity Continuous)
    ### ***Description***
    >PVC accelerates the servo to the register based velocity using the register based
    acceleration. During the move, any move parameter can be updated. With the
    parameter Starting Data Register=N, the move parameters are as follows:\
    Register N = Acceleration\
    Register N + 1 = Velocity\
    The bits of the Mode word allow the following combinations:

    | Bit # | Description |
    | --- | --- |
    | 0 | End Command When Stopped<br>Set this bit to end the command when the servo is commanded to zero<br>velocity (velocity register=0) and stops (actual velocity=0). |
    | 1 | Override Existing Move<br>Set this bit to have PVC override any existing move commands<br>(requires multi-tasking enabled). By default this bit=0, which means<br>PVC will wait for the previous move to complete. |
    | 2 | Must be set to 0. |
    | 3 | Must be set to 0.<br>NOTE: EGM has the same command number as PVC (cmd=93). The<br>command is EGM if this bit=1 and PVC if this bit=0. |
    >The Profile Velocity Continuous puts the device into a move that does not end unless
    explicitly commanded (i.e. VMP).\
    \
    This command can also be used through the serial interface, however a NAK Busy will
    be reported when a Program or a motion command is executing.\
    \
    Acceleration is normally a positive value regardless of velocity sign. If a stop condition
    is met with the Acceleration register set to 0, the servo will ramp to a fairly fast stop
    (approximately 1 second from full speed). Setting the acceleration to a negative value
    will cause the motion to ramp to a stop and the command to end using the magnitude of
    the acceleration to decelerate. As long as “End Command When Stopped” is not set,
    PVC allows the motion to be stopped and restarted just by writing to the registers.\
    \
    See Scaling in User Manual for more details on native acceleration and velocity units.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | PVC<br>SN n/a<br>SD 30 | Program<br>Class D<br>93 (0x5D)<br>5 words<br>Thread 1 | Mode<br>Starting Data Register<br>Stop Enable<br>Stop State | U16<br>U16<br>S16/U16<br>S16/U16 | See Above. 0 = default<br>11 to 198<br>See Above<br>See Above |

    ### ***Example***
    Put device into velocity mode running at acceleration in register 21 and velocity in register 22:\
    *@16 93 0 21 0 0(CR)*
    ### ***Response***
    ACK only
- ## STP(Stop)
    ### ***Description***
    >The Stop command exits the executing program or motion. If a motion is running, the
    Deceleration parameter sets the deceleration as follows: If the parameter is zero, the
    device uses the executing command’s acceleration value for deceleration. If the
    parameter is positive, the device uses the given deceleration value. If the parameter is
    negative, the device does an immediate stop. The servo's target position value is set to
    the actual position. If the servo is not executing a motion, any Program Mode command
    executing is terminated and the servo returns to idle.\
    \
    When the Stop command is sent, the Program Buffer is over-written (similar to a Clear
    Program (CLP) command). The Program Buffer must be loaded again (Load Program
    (LPR) or Load And Run Program (LRP)) for program execution.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | STP | Immediate<br>Class F<br>3 (0x3)<br>3 words | Deceleration | S32 | **-1** = Stop Immediate or<br>**0** = Stop using previous<br>Acceleration or<br>1 to $2^{29}$|

    ### ***Example***
    Stop the device using the previous commanded acceleration.\
    *@16 3 0 (CR)*
    ### ***Response***
    ACK only
- ## VMI(Velocity Mode, Immediate Mode)
    ### ***Description***
    >Accelerates the servo from the present velocity to the indicated velocity using the given
    acceleration. If the servo has an active move operation in progress, that motion is taken
    over from its current velocity, and ramps to the new velocity at the given acceleration
    rate. Any program operating is stopped and the contents of the command buffer are
    modified. This command is used when the velocity mode needs to be controlled from a
    Host controller. This command can only be used through the serial interface. See the
    Velocity Mode, Program Mode (VMP) command for embedding this type of command in
    a program.\
    \
    NOTE: If the acceleration is negative, any accumulated position error is removed and
    the absolute value of the acceleration is then used.\
    \
    See Scaling in User Manual for more details on native acceleration and velocity units.\
    \
    See Using Inputs to Stop Motion in User Manual for Stop Enable and Stop State
    definitions.

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | VMI | Immediate<br>Class F<br>15 (0xF)<br>7 words | Acceleration<br>Velocity<br>Stop Enable<br>Stop State | S32<br>S32<br>S16/U16<br>S16/U16 | -1 to $-2^{30}$ or 1 to $2^{30}$<br>$-2^{31}$ to $2^{31}$<br>See above<br>See above |

    ### ***Example***
    Put the device into velocity mode running at 200 RPM.\
    *@16 15 200000 107374200 0 0(CR)*
    ### ***Response***
    ACK only

## DATA REGISTER COMMANDS
- ## RRG(Read Register)
    ### ***Description***
    >The Read Register command reads back data from a selected 32-Bit Data Register
    using the Serial Interface. Since it is an Immediate Mode, this command can be used at
    any time, even during program execution. Any Data Register can be read back using
    this command.

    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | RRG | Immediate<br>Class A<br>12 (0x0C)<br>2 words | Data Register<br><br>SD 31:<br>Data Register 2 (optional)<br>Data Register 3 (optional)<br>Data Register 4 (optional) | U16<br><br>SD 31<br>S16<br>S16<br>S16 | SD 31:<br>Optional arguements<br>to read additional<br>registers. |

    ### ***Example***
    Read back the motor’s current position.\
    *@16 12 1 (CR)*
    ### ***Response***
    Data Register data
    ### ***Response Example***
    Read Register command that requests the “Current Position” from device #16 (which is
    “10” in Hexadecimal); the last 8 digits represent the 32-bits of position data.\
    \
    The current position = 329,379 in decimal
    \# *10 000C 0005 06A3 (CR)*\
    \
    The return data breaks down as follows:
    | '#' Indicates<br>Return Data | Address | Command<br>Recieved | Upper Word<br>of Data | Lower Word<br>of Data | Carriage<br>Return ASCII |
    | :---: | :---: | :---: | :---: | :---: | :---: |
    | # | 10 | 000C | 0005 | 06A3 | (CR) |

- ## WRI(Write Register, Immediate Mode)
    ### ***Description***
    >This command writes the given data into the selected 32 bit Data Register. Using the
    Serial Interface this command can be used at any time, even during program execution.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | WRI | Immediate<br>Class A<br>11 (0x0B)<br>4 words<br> | Data Register<br><br>Data | U16<br><br>S32/U32 | Standard Register Range<br><br>0 to $2^{32}$ or -$2^{31}$ to $2^{31}$ |

    ### ***Example***
    Write the number “8000” to data register #12.\
    *@16 11 12 8000 (CR)*
    ### ***Response***
    ACK only

## MISC. COMMANDS
- ## CIS(Clear Internal Status)
    ### ***Description***
    >The Internal Status Word (ISW) is used to indicate different conditions or states in the
    device (see Internal Status Word (ISW) in User Manual for details). Several of the
    conditions are “latched” and therefore are persistent even after the condition has
    changed. The CIS command is used to clear the latched conditions in the ISW.\
    \
    This command should be used after a Kill Motor condition has occurred before normal
    operation can be restored.\
    \
    SD08: This command also clears the latched bits in the Internal Status Word 2 (IS2) in
    the SilverDust units. (See User Manual for more details.)
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | CIS | Program<br>Class D<br>163 (0xA3)<br>1 word<br>Thread 1&2 | NONE | NONE | NONE |

    ### ***Example***
    Clear the Internal Status Word.\
    *@16 163 (CR)*
    ### ***Response***
    ACK only
- ## CKS(Check Internal Status)
    ### ***Description***
    >This command checks the conditions of the Internal Status Word in the same manner
    as does the Jump command. If the condition enabled is true, bit #6 of the Polling Status
    is set to “1”. A zero in the Condition Enable parameter unconditionally sets bit #6 of the
    Polling Status Word.\
    \
    This command may be used to convey information from a program executing back to
    the host processor that is polling the device.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | CKS | Program<br>Class D<br>164 (0xA4)<br>3 words<br>Thread 1&2 | Condition Enable<br><br>Condition State | U16<br><br>U16 | 0 to 65535<br><br>0 to 65535 |

    ### ***Example***
    Check for a Last Calculation Was Positive and report to Host using Polling Status Word.\
    *@16 164 4 4 (CR)*
    ### ***Response***
    ACK only
- ## CME(Clear Max Error)
    ### ***Description***
    >The Maximum Error (absolute value of the Position Error) is updated and latched each
    servo cycle. The value is limited to a single word, saturating at 32767 (0x7FFF) as a
    maximum value. This command allows the Maximum Error value to be reset to zero so
    that the Maximum Error for a new motion profile may be determined.\
    \
    The Maximum Error value is stored in a Dedicated Data Register and may be read
    using the Read Register command.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | CME | Program<br>Class D<br>147 (0x93)<br>1 word<br>Thread 1&2 | NONE | NONE | NONE |

    ### ***Example***
    Clear the Maximum Error value.\
    *@16 147 (CR)*
    ### ***Response***
    ACK only
- ## TTP(Target To Position)
    ### ***Description***
    >This command copies the current Position value into the Target register. This is useful
    for removing errors when an obstruction is encountered without losing track of position.
    This allows the next motion to move and ramp as expected rather than having to unwind
    the accumulated error. This is useful for homing against a hard stop where error is
    intentionally introduced, and for removing error before enabling the motor drivers after
    they have been disabled.\
    \
    The Target value is updated by the Trajectory Generator, the Step & Direction mode or
    one of the Input Modes. The servo loop uses the Target value as the input position
    parameter. If the motor is unable to achieve the Target position windup will occur. This
    command removes the windup error.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | TTP | Program<br>Class D<br>146 (0x92)<br>1 word<br> Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Sets the Target to the current position.\
    *@16 146 (CR)*
    ### ***Response***
    ACK only
- ## ZTP(Zero Target and Position)
    ### ***Description***
    >This command zeros the Target register and the Position register. This command zeros
    out both registers and removes any Position Error that may exist. This is useful for
    homing routines to denote the current location as “Zero” so that all other locations can
    be defined as an offset from “Zero”.\
    \
    This command removes any Windup that may exist from a previous motion.
    ### ***Command Info***
    | Command | Command Type/Num | Parameters | Param Type | Parameter Range |
    | --- | --- | --- | --- | --- |
    | ZTP | Program<br>Class D<br>145 (0x91)<br>1 word<br> Thread 1 | NONE | NONE | NONE |

    ### ***Example***
    Sets the Target & Position to zero (“0”).\
    *@16 145 (CR)*
    ### ***Response***
    ACK only