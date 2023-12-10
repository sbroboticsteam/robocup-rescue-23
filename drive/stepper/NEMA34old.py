import serial
import logging
from serial.tools.list_ports import comports

# https://drive.google.com/drive/folders/17MDqtxjRITVGbvIFIhSZzGf61syQ4ilb?usp=sharing

# TODO - Verify anywhere that uses stopEnable / stopState


class NEMA34():
    global COMMANDS

    COMMANDS = {
        "POL": 0,
        "CPL": 1,
        "HLT": 2,
        "STP": 3,
        "RST": 4,
        "RVN": 5,
        "RPB": 6,
        "CLP": 8,
        "SDL": 9,
        "RUN": 10,
        "WRI": 11,
        "RRG": 12,
        "SPR": 13,
        "LPR": 14,
        "VMI": 15,
        "RIS": 20,
        "RIO": 21,
        "IMW": 25,
        "POR": 27,
        "WRX": 30,
        "CII": 31,
        "RRW": 32,
        "PUP": 33,
        "ADX": 64,
        "CER": 65,
        "ETP": 66,
        "ETN": 67,
        "FL2": 68,
        "VLL": 69,
        "CT2": 70,
        "CBD": 71,
        "CDL": 72,
        "CID": 73,
        "CNL": 74,
        "T1F": 75,
        "T2S": 76,
        "T2K": 77,
        "PLS": 78,
        "PLT": 79,
        "CDR": 80,
        "CNR": 81,
        "SMD": 86,
        "JRB": 89,
        "PCB": 89,
        "SSI": 92,
        "EGM": 93,
        "PVC": 93,
        "END": 128,
        "PWO": 129,
        "SEF": 130,
        "LVP": 131,
        "MAV": 134,
        "MRV": 135,
        "JGE": 137,
        "JGR": 137,
        "JLE": 137,
        "JLT": 137,
        "JNE": 137,
        "JRE": 137,
        "WCL": 138,
        "WCW": 139,
        "DLT": 140,
        "DLY": 140,
        "WDL": 141,
        "GCL": 142,
        "GOL": 143,
        "ZTG": 144,
        "ZTP": 145,
        "TTP": 146,
        "CME": 147,
        "CTC": 148,
        "TQL": 149,
        "AHC": 150,
        "ERL": 151,
        "WRF": 154,
        "WRP": 154,
        "IDT": 155,
        "LRP": 156,
        "CLX": 158,
        "VMP": 159,
        "RAV": 160,
        "RRV": 161,
        "JMP": 162,
        "JOI": 162,
        "CIS": 163,
        "CKS": 164,
        "CLC": 165,
        "CLM": 166,
        "KMC": 167,
        "MCT": 168,
        "FLC": 169,
        "EEM": 170,
        "DDB": 171,
        "DEM": 171,
        "ADL": 173,
        "BRT": 174,
        "MAT": 176,
        "MRT": 177,
        "RAT": 178,
        "RRT": 179,
        "SSD": 180,
        "KMR": 181,
        "KED": 182,
        "KDD": 183,
        "DIR": 184,
        "PRO": 185,
        "SIF": 186,
        "EDL": 187,
        "CIO": 188,
        "EMN": 192,
        "SEE": 192,
        "ARI": 193,
        "WBS": 194,
        "SCF": 195,
        "RSM": 196,
        "RLM": 197,
        "RSN": 198,
        "RLN": 199,
        "CLD": 200,
        "PCI": 201,
        "PCL": 201,
        "PRI": 202,
        "PRT": 202,
        "WBE": 204,
        "SOB": 205,
        "COB": 206,
        "ACR": 207,
        "PLR": 208,
        "FOR": 209,
        "NXT": 210,
        "LVT": 212,
        "OVT": 213,
        "MTT": 214,
        "CTW": 215,
        "PIM": 216,
        "VIM": 217,
        "TIM": 218,
        "AHM": 219,
        "KMX": 220,
        "SSL": 221,
        "RSD": 223,
        "EMT": 225,
        "DMT": 226,
        "EMD": 227,
        "DMD": 228,
        "HSM": 229,
        "AHD": 230,
        "PCM": 231,
        "PCG": 232,
        "XRV": 233,
        "XAV": 234,
        "XRT": 235,
        "XAT": 236,
        "GOC": 237,
        "JNA": 238,
        "JOR": 239,
        "PMC": 240,
        "PMV": 241,
        "PMX": 242,
        "DLC": 243,
        "SLC": 244,
        "PCP": 245,
        "ATR": 248,
        "PMO": 249,
        "JAN": 250,
        "EDH": 251,
        "DIF": 252,
        "IMS": 253,
        "IMQ": 254,
        "RSP": 255,
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
        CID = COMMANDS.get("POR")
        self.execute(CID)

    def CPL(self, address: int):
        if address < 0 or address > 65535:
            logging.error(f"Address ({address}) is out of range")
            return

        CID = COMMANDS.get("CPL")
        self.execute(CID, address)

    def RIS(self):
        CID = COMMANDS.get("RIS")
        self.execute(CID)

    def AHC(self, openToClosed: int, closedToOpen: int):
        if openToClosed < 0 or openToClosed > 140:
            logging.error(f"Open to closed ${openToClosed} is invalid")
            return
        if closedToOpen < -140 or closedToOpen > 140:
            logging.error(f"Closed to open ${closedToOpen} is invalid")
            return

        CID = COMMANDS.get("AHC")
        self.execute(CID, openToClosed, closedToOpen)

    def AHD(self, delayCount: int = 1250):
        if delayCount < 0 or delayCount > 65535:
            logging.error(f"Delay count ({delayCount}) out of range")
            return

        CID = COMMANDS.get("AHD")
        self.execute(CID, delayCount)

    def AHM(self, mode: int = 0):
        if mode != 0 and mode != 1:
            logging.error("Mode is invalid")
            return

        CID = COMMANDS.get("AHM")
        self.execute(CID, mode)

    def BRT(self, speed: int = 576):
        possibles = [3, 12, 24, 48, 96, 192, 288,
                     384, 576, 1000, 1152, 2304, 2500]
        if speed not in possibles and (speed < -32767 or speed > -11):
            logging.error(f"Baudrate ({speed}) is invalid")
            return

        CID = COMMANDS.get("BRT")
        self.execute(CID, speed)

    def CER(self, process: int):
        if process < -1 or process > 65535:
            logging.error("Process is invalid")
            return

        CID = COMMANDS.get("CER")
        self.execute(CID, process)

    def DIR(self, mode: int = 0):
        if mode != 0 and mode != 1:
            logging.error("Mode is invalid")
            return

        CID = COMMANDS.get("DIR")
        self.execute(CID, mode)

    def DMD(self):
        CID = COMMANDS.get("DMD")
        self.execute(CID)

    def EMD(self):
        CID = COMMANDS.get("EMD")
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

        CID = COMMANDS.get("ERL")
        self.execute(CID, movingLimit, holdingLimit, delayToHolding)

    def GCL(self):
        CID = COMMANDS.get("GCL")
        self.execute(CID)

    def GOC(self, gravityOffset: int = 0):
        if gravityOffset < -32768 or gravityOffset > 32767:
            logging.error(f"Gravity offset ({gravityOffset}) out of range")
            return

        CID = COMMANDS.get("GOC")
        self.execute(CID, gravityOffset)

    def GOL(self):
        CID = COMMANDS.get("GOL")
        self.execute(CID)

    def KDD(self):
        CID = COMMANDS.get("KDD")
        self.execute(CID)

    def KED(self):
        CID = COMMANDS.get("KED")
        self.execute(CID)

    def KMC(self, conditionEnable: int, conditionState: int):
        if conditionEnable < 0 or conditionEnable > 65535:
            logging.error(f"Condition enable ({conditionEnable}) out of range")
            return
        if conditionState < 0 or conditionState > 65535:
            logging.error(f"Condition state ({conditionState}) out of range")
            return

        CID = COMMANDS.get("KMC")
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

        CID = COMMANDS.get("KMX")
        self.execute(CID, condEnISW, condStISW, condEnIS2,
                     condStIS2, condEnXIO, condStXIO)

    def KMR(self, process: int):
        if process < -1 or process > 65535:
            logging.error("Process is invalid")
            return

        CID = COMMANDS.get("KMR")
        self.execute(CID, process)

    def LVP(self, voltage: int = 0):
        if voltage != 0 and (voltage < 10 or voltage > 48):
            logging.error(f"Voltage ({voltage}) is out of range")
            return

        CID = COMMANDS.get("LVP")
        self.execute(CID, voltage)

    def LVT(self, voltage: int = 10):
        if voltage != 0 and (voltage < 10 or voltage > 48):
            logging.error(f"Voltage ({voltage}) is out of range")
            return

        CID = COMMANDS.get("LVT")
        self.execute(CID, voltage)

    def MTT(self, temperature: int = 0):
        if temperature < 0 or temperature > 80:
            logging.error(f"Temperature ({temperature}) is out of range")
            return

        CID = COMMANDS.get("MTT")
        self.execute(CID, temperature)

    def OVT(self, voltage: int = 52):
        if voltage < 1 or voltage > 53:
            logging.error(f"Voltage ({voltage}) is out of range")
            return

        CID = COMMANDS.get("OVT")
        self.execute(CID, voltage)

    def PLR(self, process: int):
        if process < -1 or process > 32767:
            logging.error(f"Process ({process}) is out of range")
            return

        CID = COMMANDS.get("PLR")
        self.execute(CID, process)

    def PRO(self, word: int):
        if word < -32768 or word > 32767:
            logging.error(f"Word is invalid")
            return

        CID = COMMANDS.get("PRO")
        self.execute(CID, word)

    def SCF(self, factor: int = 0):
        if factor < 0 or factor > 32767:
            logging.error(f"Factor ({factor}) is out of range")
            return

        CID = COMMANDS.get("SCF")
        self.execute(CID, factor)

    def SIF(self, mode: int = 0):
        if mode != 0 and mode != 1:
            logging.error("Invalid mode")
            return

        CID = COMMANDS.get("SIF")
        self.execute(CID, mode)

    def SLC(self):
        CID = COMMANDS.get("SLC")
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

        CID = COMMANDS.get("TQL")
        self.execute(CID, closedLoopHolding, closedLoopMoving,
                     openLoopHolding, openLoopMoving)

    def VLL(self, movingLimit: int, holdingLimit: int):
        if movingLimit < 0 or movingLimit > 32767:
            logging.error(f"Moving limit ({movingLimit}) out of range")
            return
        if holdingLimit < 0 or holdingLimit > 32767:
            logging.error(f"Holding Limit ({holdingLimit}) out of range")
            return

        CID = COMMANDS.get("VLL")
        self.execute(CID, movingLimit, holdingLimit)

    def HLT(self):
        CID = COMMANDS.get("HLT")
        self.execute(CID)

    def MAT(self, position: int, accelerationTime: int, totalTime: int, stopEnable: int = 0, stopState: int = 0):
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

        CID = COMMANDS.get("MAT")
        self.execute(CID, position, accelerationTime,
                     totalTime, stopEnable, stopState)

    def MAV(self, position: int, acceleration: int, totalTime: int, stopEnable: int = 0, stopState: int = 0):
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

        CID = COMMANDS.get("MAV")
        self.execute(CID, position, acceleration,
                     totalTime, stopEnable, stopState)

    def MRT(self, distance: int, rampTime: int, totalTime: int, stopEnable: int = 0, stopState: int = 0):
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

        CID = COMMANDS.get("MRT")
        self.execute(CID, distance, rampTime, totalTime, stopEnable, stopState)

    def MRV(self, distance: int, acceleration: int, velocity: int, stopEnable: int = 0, stopState: int = 0):
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

        CID = COMMANDS.get("MRV")
        self.execute(CID, distance, acceleration,
                     velocity, stopEnable, stopState)

    def PMC(self, stopEnable: int = 0, stopState: int = 0):
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = COMMANDS.get("PMC")
        self.execute(CID, stopEnable, stopState)

    def PMO(self, stopEnable: int = 0, stopState: int = 0):
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = COMMANDS.get("PMO")
        self.execute(CID, stopEnable, stopState)

    def PMV(self, stopEnable, stopState):
        if stopEnable < 0 or stopEnable > 32767:
            logging.error(f"Stop enable invalid")
            return
        if stopState not in {0, 1, 2, 3}:
            logging.error(f"Stop state is invalid")
            return

        CID = COMMANDS.get("PMV")
        self.execute(CID, stopEnable, stopState)

    def PMX(self):
        CID = COMMANDS.get("PMX")
        self.execute(CID)

    def PVC(self, startingDataRegister: int, stopEnable: int = 0, stopState: int = 0, mode: int = 0):
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

        CID = COMMANDS.get("PVC")
        self.execute(CID, mode, startingDataRegister, stopEnable, stopState)

    def STP(self, deceleration: int):
        if deceleration < -1 or deceleration > 536870911:
            logging.error(f"Deceleration ({deceleration}) out of range")
            return

        CID = COMMANDS.get("STP")
        self.execute(CID, deceleration)

    def VMI(self, acceleration: int, velocity: int, stopEnable: int = 0, stopState: int = 0):
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

        CID = COMMANDS.get("VMI")
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
        CID = COMMANDS.get("RRG")
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

        CID = COMMANDS.get("WRI")
        self.execute(CID, dataRegister, data)

    def CIS(self):
        CID = COMMANDS.get("CIS")
        self.execute(CID)

    def CKS(self, conditionEnable: int, conditionState: int):
        if conditionEnable < 0 or conditionEnable > 65535:
            logging.error(f"Condition enable ({conditionEnable}) out of range")
            return
        if conditionState < 0 or conditionState > 65535:
            logging.error(f"Condition state ({conditionState}) out of range")
            return

        CID = COMMANDS.get("CKS")
        self.execute(CID, conditionEnable, conditionState)

    def CME(self):
        CID = COMMANDS.get("CME")
        self.execute(CID)

    def TTP(self):
        CID = COMMANDS.get("TTP")
        self.execute(CID)

    def ZTP(self):
        CID = COMMANDS.get("ZTP")
        self.execute(CID)

    def CUSTOM(self, CommandID, *args):
        self.execute(CommandID, *args)

    def execute(self, *args):
        self.serial.write(self.encode(*args))
        logging.info(self.read())

    def encode(self, *args) -> bytes:
        ascii_string = f"@{self.id} {' '.join(map(str, args))} \r"
        logging.info(f"-->@{self.id} {' '.join(map(str, args))} \r")
        return ascii_string.encode()

    def read(self):
        response = self.serial.read_until(expected='\r'.encode())
        response_string = str(response, encoding='utf-8')
        response_data: str = response_string.split(" ")
        response_type: str = response_data[0]
        response_args = response_data[1:]

        for i in range(0, len(response_args)):
            response_args[i] = int(response_args[i], 16)

        if (response_type == "*"):
            # ACK
            respondingID = response_args[0]
            return (f"<--@{respondingID} ACK")
        elif (response_type == "!"):
            # NAK
            respondingID = response_args[0]
            commandBeingRespondedTo = response_args[1]
            NAKCode = response_args[2]
            codes = {
                1: "Bad Command",
                2: "Device Busy",
                3: "Reserved",
                4: "Reserved",
                5: "Bad Format",
                6: "Buffer Full ",
                7: "Bad Address",
                8: "Bad Response Packet Request",
                9: "Bad PUP Lockout Code",
                10: "Bad Checksum"
            }
            return (f"<--@{respondingID} {commandBeingRespondedTo} NAK {codes.get(NAKCode)}")
        elif (response_type == "#"):
            # Data
            respondingID = response_args[0]
            commandBeingRespondedTo = response_args[1]
            data = response_args[2:]
            return (f"<--@{respondingID} Data {data}")
        else:
            # unknown response type
            print(f"-----{response}")
            return (f"<-- Unknown response type")
        # return response
