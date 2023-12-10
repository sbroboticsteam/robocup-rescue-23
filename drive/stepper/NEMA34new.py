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
        
    def CPL(self, address: int):
        if address < 0 or address > 65535:
            logging.error(f"Address ({address}) is out of range")
            return

        CID = COMMANDS.get("CPL")
        self.execute(CID, address)
        
    def POL(self, address: int):

        CID = COMMANDS.get("POL")
        self.execute(CID, address)
        
        