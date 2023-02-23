import serial
import logging
from serial.tools.list_ports import comports

class NEMA34():

    COMMANDS = {
        "POR": 27,
        "CPL": 1,
        "RIS": 20,
        "ADL": 173,
        "ADC": 207,
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
        logging.basicConfig(format='%(asctime)s %(message)s', level=logging.DEBUG)
        if motor_port not in [port.name for port in comports()]:
            raise Exception("Invalid port entered.")

        self.serial = serial.Serial(
                baudrate = 57600,
                stopbits = serial.STOPBITS_TWO,
                port = motor_port,
                timeout = 1,
                parity = serial.PARITY_NONE
            )
        self.id = 16

        loggging.info(f"Motor initialized: ({motor_port}, {57600})")

    def __exit__(self):
        self.serial.close()

    def POR(self):
        CID = 27
        self.execute(CID)

    def CPL(self, address -> int):
        CID = 1
        if address < 0 or address > 65535:
            logging.error(f"Address ({}) is out of range")
            return

        self.execute(CID, address)

    def RIS(self):
        CID = 20
        self.execute(CID)

    def ADL(self):
        CID = 173
        self.execute(CID)

    def ADC(self):
        CID = 207
        self.execute(CID)

    def AHD(self):
        CID = 230
        self.execute(CID)

    def AHM(self):
        CID = 219
        self.execute(CID)

    def BRT(self):
        CID = 174
        self.execute(CID)

    def CER(self):
        CID = 65
        self.execute(CID)

    def DIR(self):
        CID = 184
        self.execute(CID)

    def DMD(self):
        CID = 228
        self.execute(CID)

    def EMD(self):
        CID = 227
        self.execute(CID)

    def ERL(self):
        CID = 151
        self.execute(CID)

    def GCL(self):
        CID = 142
        self.execute(CID)

    def GOC(self):
        CID = 237
        self.execute(CID)

    def GOL(self):
        CID = 143
        self.execute(CID)

    def KDD(self):
        CID = 183
        self.execute(CID)

    def KED(self):
        CID = 182
        self.execute(CID)

    def KMC(self):
        CID = 167
        self.execute(CID)

    def KMX(self):
        CID = 167
        self.execute(CID)

    def KMR(self):
        CID = 181
        self.execute(CID)

    def LVP(self):
        CID = 131
        self.execute(CID)

    def LVT(self):
        CID = 212
        self.execute(CID)

    def MTT(self):
        CID = 214
        self.execute(CID)

    def OVT(self):
        CID = 213
        self.execute(CID)

    def PLR(self):
        CID = 208
        self.execute(CID)

    def PRO(self):
        CID = 185
        self.execute(CID)

    def SCF(self):
        CID = 195
        self.execute(CID)

    def SIF(self):
        CID = 186
        self.execute(CID)

    def SLC(self):
        CID = 244
        self.execute(CID)

    def TQL(self):
        CID = 149
        self.execute(CID)

    def VLL(self):
        CID = 69
        self.execute(CID)

    def HLT(self):
        CID = 2
        self.execute(CID)

    def MAT(self):
        CID = 196
        self.execute(CID)

    def MAV(self):
        CID = 134
        self.execute(CID)

    def MRT(self):
        CID = 177
        self.execute(CID)

    def MRV(self):
        CID = 135
        self.execute(CID)

    def PMC(self):
        CID = 240
        self.execute(CID)

    def PMO(self):
        CID = 249
        self.execute(CID)

    def PMV(self):
        CID = 241
        self.execute(CID)

    def PMX(self):
        CID = 242
        self.execute(CID)

    def PVC(self):
        CID = 93
        self.execute(CID)

    def STP(self):
        CID = 3
        self.execute(CID)

    def VMI(self):
        CID = 15
        self.execute(CID)

    def RRG(self):
        CID = 12
        self.execute(CID)

    def WRI(self):
        CID = 11
        self.execute(CID)

    def CIS(self):
        CID = 163
        self.execute(CID)

    def CKS(self):
        CID = 164
        self.execute(CID)

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