#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "serialib.h"

#define POL_COMM 0
#define CPL_COMM 1
#define HLT_COMM 2
#define STP_COMM 3
#define RST_COMM 4
#define RVN_COMM 5
#define RPB_COMM 6
#define CLP_COMM 8
#define SDL_COMM 9
#define RUN_COMM 10
#define WRI_COMM 11
#define RRG_COMM 12
#define SPR_COMM 13
#define LPR_COMM 14
#define VMI_COMM 15
#define RIS_COMM 20
#define RIO_COMM 21
#define IMW_COMM 25
#define POR_COMM 27
#define WRX_COMM 30
#define CII_COMM 31
#define RRW_COMM 32
#define PUP_COMM 33
#define ADX_COMM 64
#define CER_COMM 65
#define ETP_COMM 66
#define ETN_COMM 67
#define FL2_COMM 68
#define VLL_COMM 69
#define CT2_COMM 70
#define CBD_COMM 71
#define CDL_COMM 72
#define CID_COMM 73
#define CNL_COMM 74
#define T1F_COMM 75
#define T2S_COMM 76
#define T2K_COMM 77
#define PLS_COMM 78
#define PLT_COMM 79
#define CDR_COMM 80
#define CNR_COMM 81
#define SMD_COMM 86
#define JRB_COMM 89
#define PCB_COMM 89
#define SSI_COMM 92
#define EGM_COMM 93
#define PVC_COMM 93
#define END_COMM 128
#define PWO_COMM 129
#define SEF_COMM 130
#define LVP_COMM 131
#define MAV_COMM 134
#define MRV_COMM 135
#define JGE_COMM 137
#define JGR_COMM 137
#define JLE_COMM 137
#define JLT_COMM 137
#define JNE_COMM 137
#define JRE_COMM 137
#define WCL_COMM 138
#define WCW_COMM 139
#define DLT_COMM 140
#define DLY_COMM 140
#define WDL_COMM 141
#define GCL_COMM 142
#define GOL_COMM 143
#define ZTG_COMM 144
#define ZTP_COMM 145
#define TTP_COMM 146
#define CME_COMM 147
#define CTC_COMM 148
#define TQL_COMM 149
#define AHC_COMM 150
#define ERL_COMM 151
#define WRF_COMM 154
#define WRP_COMM 154
#define IDT_COMM 155
#define LRP_COMM 156
#define CLX_COMM 158
#define VMP_COMM 159
#define RAV_COMM 160
#define RRV_COMM 161
#define JMP_COMM 162
#define JOI_COMM 162
#define CIS_COMM 163
#define CKS_COMM 164
#define CLC_COMM 165
#define CLM_COMM 166
#define KMC_COMM 167
#define MCT_COMM 168
#define FLC_COMM 169
#define EEM_COMM 170
#define DDB_COMM 171
#define DEM_COMM 171
#define ADL_COMM 173
#define BRT_COMM 174
#define MAT_COMM 176
#define MRT_COMM 177
#define RAT_COMM 178
#define RRT_COMM 179
#define SSD_COMM 180
#define KMR_COMM 181
#define KED_COMM 182
#define KDD_COMM 183
#define DIR_COMM 184
#define PRO_COMM 185
#define SIF_COMM 186
#define EDL_COMM 187
#define CIO_COMM 188
#define EMN_COMM 192
#define SEE_COMM 192
#define ARI_COMM 193
#define WBS_COMM 194
#define SCF_COMM 195
#define RSM_COMM 196
#define RLM_COMM 197
#define RSN_COMM 198
#define RLN_COMM 199
#define CLD_COMM 200
#define PCI_COMM 201
#define PCL_COMM 201
#define PRI_COMM 202
#define PRT_COMM 202
#define WBE_COMM 204
#define SOB_COMM 205
#define COB_COMM 206
#define ACR_COMM 207
#define PLR_COMM 208
#define FOR_COMM 209
#define NXT_COMM 210
#define LVT_COMM 212
#define OVT_COMM 213
#define MTT_COMM 214
#define CTW_COMM 215
#define PIM_COMM 216
#define VIM_COMM 217
#define TIM_COMM 218
#define AHM_COMM 219
#define KMX_COMM 220
#define SSL_COMM 221
#define RSD_COMM 223
#define EMT_COMM 225
#define DMT_COMM 226
#define EMD_COMM 227
#define DMD_COMM 228
#define HSM_COMM 229
#define AHD_COMM 230
#define PCM_COMM 231
#define PCG_COMM 232
#define XRV_COMM 233
#define XAV_COMM 234
#define XRT_COMM 235
#define XAT_COMM 236
#define GOC_COMM 237
#define JNA_COMM 238
#define JOR_COMM 239
#define PMC_COMM 240
#define PMV_COMM 241
#define PMX_COMM 242
#define DLC_COMM 243
#define SLC_COMM 244
#define PCP_COMM 245
#define ATR_COMM 248
#define PMO_COMM 249
#define JAN_COMM 250
#define EDH_COMM 251
#define DIF_COMM 252
#define IMS_COMM 253
#define IMQ_COMM 254
#define RSP_COMM 255




#define MAX_STRING_SIZE 2048

class NEMA34 {
private:
    serialib serial;
    int id;
public:
    NEMA34(std::string motor_port) 
    {
        //timeout is 1000
        
        char serialibRet = serial.openDevice(motor_port.c_str(), 57600, SERIAL_DATABITS_8, SERIAL_PARITY_NONE, SERIAL_STOPBITS_2);
        if (serialibRet == 1)	//if it equals 1, its sucess
        {

            id = 16;
            std::cout << "Motor initialized: (" << motor_port << ", " << 57600 << ")" << std::endl;
            return;
        }


        std::cout << "ERROR WITH OPENING DEVICE: " << serialibRet << "\n";

        switch (serialibRet)
        {
        case -1:
            std::cout << "serialib returned -1: device not found" << std::endl;
            break;

        case -2:
            std::cout << "serialib returned -2: error while opening the device" << std::endl;
            break;

        case -3:
            std::cout << "serialib returned -3: error while getting port parameters" << std::endl;
            break;

        case -4:
            std::cout << "serialib returned -4: Speed(Bauds) not recognized" << std::endl;
            break;

        case -5:
            std::cout << "serialib returned -5: error while writing port parameters" << std::endl;
            break;

        case -6:
            std::cout << "serialib returned -6: error while writing timeout parameters" << std::endl;
            break;

        case -7:
            std::cout << "serialib returned -7: Databits not recognized" << std::endl;
            break;

        case -8:
            std::cout << "serialib returned -8: Stopbits not recognized" << std::endl;
            break;

        case -9:
            std::cout << "serialib returned -9: Parity not recognized" << std::endl;
            break;

        default:
            std::cout << "serialib returned " << serialibRet << ":[unknown return val]" << std::endl;
            break;
        }
        



    }
    ~NEMA34() {
        serial.closeDevice();
    }
    void execute(std::vector<int> args) {
        std::string command = encode(args);
        serial.writeString(command.c_str());
        std::cout << read() << std::endl;
    }
    std::string encode(std::vector<int> args) {
        std::string ascii_string = "@" + std::to_string(id) + " ";
        for (int i = 0; i < args.size(); i++) {
            ascii_string += std::to_string(args[i]) + " ";
        }
        ascii_string += "\r";
        std::cout << "-->@" << id << " ";
        for (int i = 0; i < args.size(); i++) {
            std::cout << args[i] << " ";
        }
        std::cout << "\r" << std::endl;
        return ascii_string;
    }
    std::string read() {
        
        char serialStr[MAX_STRING_SIZE];

        //get string from serialib
        serial.readString(serialStr, '\r', MAX_STRING_SIZE - 1, 1000);  
        std::string response = serialStr;

        std::vector<std::string> response_data;
        std::string response_type = "";
        std::string response_args = "";
        std::string response_string = response;
        size_t pos = 0;
        std::string token;
        while ((pos = response_string.find(" ")) != std::string::npos) {
            token = response_string.substr(0, pos);
            response_data.push_back(token);
            response_string.erase(0, pos + 1);
        }
        response_type = response_data[0];
        response_args = response_data[1];
        for (int i = 2; i < response_data.size(); i++) {
            response_args += " " + response_data[i];
        }
        std::vector<int> response_args_int;
        pos = 0;
        while ((pos = response_args.find(" ")) != std::string::npos) {
            token = response_args.substr(0, pos);
            response_args_int.push_back(std::stoi(token, nullptr, 16));
            response_args.erase(0, pos + 1);
        }
        if (response_type == "*") {
            int respondingID = response_args_int[0];
            return "<--@" + std::to_string(respondingID) + " ACK";
        }
        else if (response_type == "!") {
            int respondingID = response_args_int[0];
            int commandBeingRespondedTo = response_args_int[1];
            int NAKCode = response_args_int[2];
            std::map<int, std::string> codes = {
                {1, "Bad Command"},
                {2, "Device Busy"},
                {3, "Reserved"},
                {4, "Reserved"},
                {5, "Bad Format"},
                {6, "Buffer Full"},
                {7, "Bad Address"},
                {8, "Bad Response Packet Request"},
                {9, "Bad PUP Lockout Code"},
                {10, "Bad Checksum"}
            };
            return "<--@" + std::to_string(respondingID) + " " + std::to_string(commandBeingRespondedTo) + " NAK " + codes[NAKCode];
        }
        else if (response_type == "") {
            int respondingID = response_args_int[0];
            int commandBeingRespondedTo = response_args_int[1];
            std::vector<int> data(response_args_int.begin() + 2, response_args_int.end());
            std::string data_string = "";
            for (int i = 0; i < data.size(); i++) {
                data_string += std::to_string(data[i]) + " ";
            }
            return "<--@" + std::to_string(respondingID) + " Data " + data_string;
        }
        else {
            std::cout << "-----" << response << std::endl;
            return "<-- Unknown response type";
        }
    }
    void WRI(int dataRegister, int data) {
        if (dataRegister == false) {
            std::cout << "Data register is invalid" << std::endl;
            return;
        }
        if ((data < 0 || data > 4294967295) && (data < -2147483648 || data > 2147483647)) {
            std::cout << "Data is invalid $" << data << std::endl;
            return;
        }
        std::vector<int> args = { WRI_COMM, dataRegister, data };
        execute(args);
    }
    void PVC(int startingDataRegister, int stopEnable = 0, int stopState = 0, int mode = 0) {
        if (mode < -32678 || mode > 32767) {
            std::cout << "Mode is invalid" << std::endl;
            return;
        }
        if (startingDataRegister < 11 || startingDataRegister > 98) {
            std::cout << "Starting data register (" << startingDataRegister << ") is out of range" << std::endl;
            return;
        }
        if (stopEnable < 0 || stopEnable > 32767) {
            std::cout << "Stop enable invalid" << std::endl;
            return;
        }
        if (stopState != 0 && stopState != 1 && stopState != 2 && stopState != 3) {
            std::cout << "Stop state is invalid" << std::endl;
            return;
        }
        std::vector<int> args = { PVC_COMM, mode, startingDataRegister, stopEnable, stopState };
        execute(args);
    }
    void STP(int deceleration) {
        if (deceleration < -1 || deceleration > 536870911) {
            std::cout << "Deceleration (" << deceleration << ") out of range" << std::endl;
            return;
        }
        std::vector<int> args = { STP_COMM, deceleration };
        execute(args);
    }
    void CPL(int address) {
        if (address < 0 || address > 65535) {
            std::cout << "Address (" << address << ") is out of range" << std::endl;
            return;
        }
        std::vector<int> args = { CPL_COMM, address };
        execute(args);
    }
    void POL() {
        std::vector<int> args = { POL_COMM };
        execute(args);
    }
    void RRG(int dataRegister) {
        if (dataRegister < 0 || dataRegister > 60) {
            std::cout << "Data register (" << dataRegister << ") is invalid" << std::endl;
            return;
        }
        std::vector<int> args = { RRG_COMM, dataRegister };
        execute(args);
    }
};


