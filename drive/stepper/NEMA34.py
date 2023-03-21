import serial
import logging
from serial.tools.list_ports import comports

# https://drive.google.com/drive/folders/17MDqtxjRITVGbvIFIhSZzGf61syQ4ilb?usp=sharing

# TODO - Verify anywhere that uses stopEnable / stopState


class NEMA34():

    COMMANDS = {
        "POR": 27,
        "CPL": 1,
        "RIS": 20,
        "ADC": 150,
        "AHD": 230,
        "AHM": 219,
        "BRT": 174,
        "CER": 65,
        "DIR": 184,
        "DMD": 228,
        "EMD": 227,
        "ERL": 151,
        "GCL": 142,
        "GOC": 237,
        "GOL": 143,
        "KDD": 183,
        "KED": 182,
        "KMC": 167,
        "KMX": 167,
        "KMR": 181,
        "LVP": 131,
        "LVT": 212,
        "MTT": 214,
        "OVT": 213,
        "PLR": 208,
        "PRO": 185,
        "SCF": 195,
        "SIF": 186,
        "SLC": 244,
        "TQL": 149,
        "VLL": 69,
        "HLT": 2,
        "MAT": 196,
        "MAV": 134,
        "MRT": 177,
        "MRV": 135,
        "PMC": 240,
        "PMO": 249,
        "PMV": 241,
        "PMX": 242,
        "PVC": 93,
        "STP": 3,
        "VMI": 15,
        "RRG": 12,
        "WRI": 11,
        "CIS": 163,
        "CKS": 164,
        "CME": 147,
        "TTP": 146,
        "ZTP": 145
    }

    def __init__(self, motor_port) -> None:
        logging.basicConfig(
            format='%(asctime)s %(message)s', level=logging.DEBUG)
        if motor_port not in [port.name for port in comports()]:
            raise Exception("Invalid port entered.")

        self.serial = serial.Serial(
            baudrate=57600,
            stopbits=serial.STOPBITS_TWO,
            port=motor_port,
            timeout=1,
            parity=serial.PARITY_NONE
        )
        self.id = 16

        logging.info(f"Motor initialized: ({motor_port}, {57600})")

    def __exit__(self):
        self.serial.close()

    def POR(self):
        CID = 27
        self.execute(CID)

    def CPL(self, address: int):
        if address < 0 or address > 65535:
            logging.error(f"Address ({address}) is out of range")
            return

        CID = 1
        self.execute(CID, address)

    def RIS(self):
        CID = 20
        self.execute(CID)

    def AHC(self, openToClosed: int, closedToOpen: int):
        if openToClosed < 0 or openToClosed > 140:
            logging.error(f"Open to closed ${openToClosed} is invalid")
            return
        if closedToOpen < -140 or closedToOpen > 140:
            logging.error(f"Closed to open ${closedToOpen} is invalid")
            return

        CID = 150
        self.execute(CID, openToClosed, closedToOpen)

    def AHD(self, delayCount: int = 1250):
        if delayCount < 0 or delayCount > 65535:
            logging.error(f"Delay count ({delayCount}) out of range")
            return

        CID = 230
        self.execute(CID, delayCount)

    def AHM(self, mode: int = 0):
        if mode != 0 and mode != 1:
            logging.error("Mode is invalid")
            return

        CID = 219
        self.execute(CID, mode)

    def BRT(self, speed: int = 576):
        possibles = [3, 12, 24, 48, 96, 192, 288,
                     384, 576, 1000, 1152, 2304, 2500]
        if speed not in possibles and (speed < -32767 or speed > -11):
            logging.error(f"Baudrate ({speed}) is invalid")
            return

        CID = 174
        self.execute(CID, speed)

    def CER(self, process: int):
        if process < -1 or process > 65535:
            logging.error("Process is invalid")
            return

        CID = 65
        self.execute(CID, process)

    def DIR(self, mode: int = 0):
        if mode != 0 and mode != 1:
            logging.error("Mode is invalid")
            return

        CID = 184
        self.execute(CID, mode)

    def DMD(self):
        CID = 228
        self.execute(CID)

    def EMD(self):
        CID = 227
        self.execute(CID)

    def ERL(self, movingLimit: int = 0, holdingLimit: int = 0, delayToHolding: int = 100):
        if movingLimit < -32768 or movingLimit > 32767:
            logging.error(f"Moving limit ({movingLimit}) out of range")
            return
        if holdingLimit < -32768 or holdingLimit > 32767:
            logging.error(f"Holding limit ({holdingLimit}) out of range")
            return
        if delayToHolding < 0 or delayToHolding > 65535:
            logging.error(f"Delay to holding ({delayTotHolding}) out of range")
            return

        CID = 151
        self.execute(CID, movingLimit, holdingLimit, delayToHolding)

    def GCL(self):
        CID = 142
        self.execute(CID)

    def GOC(self, gravityOffset: int = 0):
        if gravityOffset < -32768 or gravityOffset > 32767:
            logging.error(f"Gravity offset ({gravityOffset}) out of range")
            return

        CID = 237
        self.execute(CID, gravityOffset)

    def GOL(self):
        CID = 143
        self.execute(CID)

    def KDD(self):
        CID = 183
        self.execute(CID)

    def KED(self):
        CID = 182
        self.execute(CID)

    def KMC(self, conditionEnable: int, conditionState: int):
        if conditionEnable < 0 or conditionEnable > 65535:
            logging.error(f"Condition enable ({conditionEnable}) out of range")
            return
        if conditionState < 0 or conditionState > 65535:
            logging.error(f"Condition state ({conditionState}) out of range")
            return

        CID = 167
        self.execute(CID, conditionEnable, conditionState)

    def KMX(self, condEnISW: int, condStISW: int, condEnIS2: int, condStIS2: int, condEnXIO: int, condStXIO: int):
        if condEnISW < 0 or condEnISW > 65535:
            logging.error(f"Condition enable IS2 ({condEnISW}) out of range")
            return
        if condStISW < 0 or condStISW > 65535:
            logging.error(f"Condition state IS2 ({condStISW}) out of range")
            return
        if condEnIS2 < 0 or condEnIS2 > 65535:
            logging.error(f"Condition enable ISW ({condEnIS2}) out of range")
            return
        if condStIS2 < 0 or condStIS2 > 65535:
            logging.error(f"Condition state ISW ({condStIS2}) out of range")
            return
        if condEnXIO < 0 or condEnXIO > 65535:
            logging.error(f"Condition enable XIO ({condEnXIO}) out of range")
            return
        if condStXIO < 0 or condStXIO > 65535:
            logging.error(f"Condition state XIO ({condStXIO}) out of range")
            return

        CID = 167
        self.execute(CID, condEnISW, condStISW, condEnIS2,
                     condStIS2, condEnXIO, condStXIO)

    def KMR(self, process: int):
        if process < -1 or process > 65535:
            logging.error("Process is invalid")
            return

        CID = 181
        self.execute(CID, process)

    def LVP(self, voltage: int = 0):
        if voltage != 0 and (voltage < 10 or voltage > 48):
            logging.error(f"Voltage ({voltage}) is out of range")
            return

        CID = 131
        self.execute(CID, voltage)

    def LVT(self, voltage: int = 10):
        if voltage != 0 and (voltage < 10 or voltage > 48):
            logging.error(f"Voltage ({voltage}) is out of range")
            return

        CID = 212
        self.execute(CID, voltage)

    def MTT(self, temperature: int = 0):
        if temperature < 0 or temperature > 80:
            logging.error(f"Temperature ({temperature}) is out of range")
            return

        CID = 214
        self.execute(CID, temperature)

    def OVT(self, voltage: int = 52):
        if voltage < 1 or voltage > 53:
            logging.error(f"Voltage ({voltage}) is out of range")
            return

        CID = 213
        self.execute(CID, voltage)

    def PLR(self, process: int):
        if process < -1 or process > 32767:
            logging.error(f"Process ({process}) is out of range")
            return

        CID = 208
        self.execute(CID, process)

    def PRO(self, word: int):
        if word < -32768 or word > 32767:
            logging.error(f"Word is invalid")
            return

        CID = 185
        self.execute(CID, word)

    def SCF(self, factor: int = 0):
        if factor < 0 or factor > 32767:
            logging.error(f"Factor ({factor}) is out of range")
            return

        CID = 195
        self.execute(CID, factor)

    def SIF(self, mode: int = 0):
        if mode != 0 and mode != 1:
            logging.error("Invalid mode")
            return

        CID = 186
        self.execute(CID, mode)

    def SLC(self):
        CID = 244
        self.execute(CID)

    def TQL(self, closedLoopHolding: int, closedLoopMoving: int, openLoopHolding: int, openLoopMoving: int):
        if closedLoopHolding < 0 or closedLoopHolding > 32767:
            logging.error(
                f"Closed loop holding ({closedLoopHolding}) out of range")
            return
        if closedLoopMoving < 0 or closedLoopMoving > 32767:
            logging.error(
                f"Closed loop moving ({closedLoopMoving}) out of range")
            return
        if openLoopHolding < 0 or openLoopHolding > 32767:
            logging.error(
                f"Open loop holding ({openLoopHolding}) out of range")
            return
        if openLoopMoving < 0 or openLoopMoving > 32767:
            logging.error(f"Open loop moving ({openLoopMoving}) out of range")
            return

        CID = 149
        self.execute(CID, closedLoopHolding, closedLoopMoving,
                     openLoopHolding, openLoopMoving)

    def VLL(self, movingLimit: int, holdingLimit: int):
        if movingLimit < 0 or movingLimit > 32767:
            logging.error(f"Moving limit ({movingLimit}) out of range")
            return
        if holdingLimit < 0 or holdingLimit > 32767:
            logging.error(f"Holding Limit ({holdingLimit}) out of range")
            return

        CID = 69
        self.execute(CID, movingLimit, holdingLimit)

    def HLT(self):
        CID = 2
        self.execute(CID)

    def MAT(self, position: int, accelerationTime: int, totalTime: int, stopEnable: int, stopState: int):
        if position < -2147483648 or position > 2147483647:
            logging.error(f"Postion ({position}) out of range")
            return
        if accelerationTime < 0 or accelerationTime > 65534:
            logging.error(
                f"Acceleration time ({accelerationTime}) out of range")
            return
        if totalTime < 2 or totalTime > 2147483647:
            logging.error(f"Total time ({totalTime}) out of range")
            return
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 196
        self.execute(CID, position, accelerationTime,
                     totalTime, stopEnable, stopState)

    def MAV(self, position: int, acceleration: int, totalTime: int, stopEnable: int, stopState: int):
        if position < -2147483648 or position > 2147483647:
            logging.error(f"Postion ({position}) out of range")
            return
        if acceleration < 1 or acceleration > 1073741824:
            logging.error(f"Acceleration ({acceleration}) out of range")
            return
        if totalTime < 2 or totalTime > 2147483648:
            logging.error(f"Total time ({totalTime}) out of range")
            return
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 134
        self.execute(CID, position, accelerationTime,
                     totalTime, stopEnable, stopState)

    def MRT(self, distance: int, rampTime: int, totalTime: int, stopEnable: int, stopState: int):
        if distance < -2147483648 or distance > 2147483647:
            logging.error(f"Distance ({distance}) out of range")
            return
        if rampTime < 0 or rampTime > 65535:
            logging.error(f"Ramp time ({rampTime}) out of range")
            return
        if totalTime < 2 or totalTime > 2147483648:
            logging.error(f"Total time ({totalTime}) out of range")
            return
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 177
        self.execute(CID, distance, rampTime, totalTime, stopEnable, stopState)

    def MRV(self, distance: int, acceleration: int, velocity: int, stopEnable: int, stopState: int):
        if distance < -2147483648 or distance > 2147483647:
            logging.error(f"Distance ({distance}) out of range")
            return
        if acceleration < 1 or acceleration > 1073741824:
            logging.error(f"Acceleration ({acceleration}) out of range")
            return
        if velocity < 2 or velocity > 2147483648:
            logging.error(f"Velocity ({velocity}) out of range")
            return
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 135
        self.execute(CID, distance, acceleration,
                     velocity, stopEnable, stopState)

    def PMC(self, stopEnable: int, stopState: int):
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 240
        self.execute(CID, stopEnable, stopState)

    def PMO(self, stopEnable: int, stopState: int):
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 249
        self.execute(CID, stopEnable, stopState)

    def PMV(self, stopEnable, stopState):
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 241
        self.execute(CID, stopEnable, stopState)

    def PMX(self):
        CID = 242
        self.execute(CID)

    def PVC(self, startingDataRegister: int, stopEnable: int, stopState: int, mode: int = 0):
        if mode < -32678 or mode > 32767:
            logging.error(f"Mode is invalid")
            return
        if startingDataRegister < 11 or startingDataRegister > 98:
            logging.error(
                f"Starting data register ({startingDataRegister}) is out of range")
            return
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 93
        self.execute(CID, mode, startingDataRegister, stopEnable, stopState)

    def STP(self, deceleration: int):
        if deceleration < -1 or deceleration > 536870911:
            logging.error(f"Deceleration ({deceleration}) out of range")
            return

        CID = 3
        self.execute(CID, deceleration)

    def VMI(self, acceleration: int, velocity: int, stopEnable: int, stopState: int):
        if acceleration < -1073741823 or acceleration > 1073741823 or acceleration == 0:
            logging.error(f"Acceleration ${acceleration} is invalid")
            return
        if velocity < -2147483648 or acceleration > 2147483647:
            logging.error(f"Velocity ${velocity} is invalid")
            return
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = 15
        self.execute(CID, acceleration, velocity, stopEnable, stopState)

    # TODO fix standard register range and optional commands
    def RRG(self, dataRegister: int, dataRegister2: int = -1, dataRegister3: int = -1, dataRegister4: int = -1):
        # Check if registers are within Standard Register Range (TM)
        if dataRegister < 0 or dataRegister > 1:
            logging.error(f"Data register 1 is invalid")
            return

        for dReg in {dataRegister2, dataRegister3, dataRegister4}:
            if dReg != -1:
                if dReg < 0 or dReg > 1:
                    logging.error(f"Data register is invalid")
                    return

        # This is confusing, would this work??
        CID = 12
        if dataRegister2 == -1:
            self.execute(CID, dataRegister)
        if dataRegister3 == -1:
            self.execute(CID, dataRegister, dataRegister2)
        if dataRegister4 == -1:
            self.execute(CID, dataRegister, dataRegister2, dataRegister3)
        self.execute(CID, dataRegister, dataRegister2,
                     dataRegister3, dataRegister4)

    def WRI(self, dataRegister: int, data: int):
        # TODO - What is a standard register range??????
        if dataRegister == False:  # Fix this
            logging.error("Data register is invalid")
            return
        if (data < 0 or data > 4294967295) and (data < -2147483648 or data > 2147483647):
            logging.error(f"Data is invalid ${data}")
            return

        CID = 11
        self.execute(CID, dataRegister, data)

    def CIS(self):
        CID = 163
        self.execute(CID)

    def CKS(self, conditionEnable: int, conditionState: int):
        if conditionEnable < 0 or conditionEnable > 65535:
            logging.error(f"Condition enable ({conditionEnable}) out of range")
            return
        if conditionState < 0 or conditionState > 65535:
            logging.error(f"Condition state ({conditionState}) out of range")
            return

        CID = 164
        self.execute(CID, conditionEnable, conditionState)

    def CME(self):
        CID = 147
        self.execute(CID)

    def TTP(self):
        CID = 146
        self.execute(CID)

    def ZTP(self):
        CID = 145
        self.execute(CID)

    def execute(self, *args):
        self.serial.write(self.encode(args))
        logging.info(self.read())

    def encode(self, *args) -> bytes:
        ascii_string = f"@{self.id} {' '.join(map(str, args[:-1]))} \r"
        return ascii_string.encode()

    def read(self):
        return self.serial.read_until(expected='\r'.encode())
