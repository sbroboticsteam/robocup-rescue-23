# Arudino Command Parser

## Sending a command:

Commands are sent over serial to an arduino in a gcode like fashion, with the command name first, then all the parameters separated by a space, and then finally a newline character "\\n"

"Command" "param" "param" ... "param" "\\n"

## Commands:

### **DIS** - Distance

Get distance of one of the ultrasonic sensors

| Param     | Description                                      | Values    |
| --------- | ------------------------------------------------ | --------- |
| sensorNum | The ultrasonic sensor number to get distance for | (int) 1-4 |

Returns:

Distance in cm of given sensor

### **SFR** - Set Frequency

Set pwm frequency across all channels

| Param | Description      | Values        |
| ----- | ---------------- | ------------- |
| hz    | Frequency to set | (int) 40-1600 |

### **SDT** - Set Duty Cycle

Sets duty cycle for a specific channel

| Param     | Description                                                     | Values        |
| --------- | --------------------------------------------------------------- | ------------- |
| channel   | PWM channel to set                                              | (int) 0-15    |
| dutyCycle | Duty cycle to be set<br />Formatted as percent (XXX.xx = XXXxx) | (int) 0-10000 |

### **GDT** - Get Duty Cycle

Get duty cycle for specific channel

| Param   | Description                       | Values     |
| ------- | --------------------------------- | ---------- |
| channel | PWM channel to get duty cycle for | (int) 0-15 |

Returns:

Duty Cycle for a particular channel
