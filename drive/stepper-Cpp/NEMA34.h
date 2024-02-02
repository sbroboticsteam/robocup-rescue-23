#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <sstream>

#include "serialib.h"
#include "instructionNames.h"

#define MAX_STRING_SIZE 2048

class NEMA34 {
private:
    serialib serial;
    int id;

public:
    NEMA34(std::string motor_port) {
        // timeout is 1000

        char serialibRet = serial.openDevice(motor_port.c_str(), 57600, SERIAL_DATABITS_8, SERIAL_PARITY_NONE, SERIAL_STOPBITS_2);
        if (serialibRet == 1) // if it equals 1, its sucess
        {

            id = 16;
            std::cout << "Motor initialized: (" << motor_port << ", " << 57600 << ")" << std::endl;
            return;
        }

        std::cout << "ERROR WITH OPENING DEVICE: " << serialibRet << "\n";

        switch (serialibRet) {
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

        exit(1);
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
        for (unsigned int i = 0; i < args.size(); i++) {
            ascii_string += std::to_string(args[i]) + " ";
        }
        ascii_string += "\r";
        std::cout << "-->@" << id << " ";
        for (unsigned int i = 0; i < args.size(); i++) {
            std::cout << args[i] << " ";
        }
        std::cout << "\r" << std::endl;
        return ascii_string;
    }

    std::vector<std::string> split(const std::string& s, char delim) {
        std::vector<std::string> result;
        std::stringstream ss(s);
        std::string item;

        while (getline(ss, item, delim)) {
            result.push_back(item);
        }
        return result;
    }

    std::string read() {
        char serialStr[MAX_STRING_SIZE];

        // get string from serialib
        serial.readString(serialStr, '\r', MAX_STRING_SIZE - 1, 1000);

        std::string response = serialStr;

        if (response.empty()) // check if no response, if so, exit read func
        {
            std::cout << "[WARNING] No Reponse from QuickSilver" << std::endl;
            return "";
        }

        std::string response_string = response; // Raw response from serial

        std::vector<std::string> response_data; // Split into individual strings
        response_data = split(response_string, ' ');

        std::string response_type = ""; // First string is the response type
        response_type = response_data[0];

        std::vector<std::string> response_args; // All other strings are the response args
        // Take all the response arguements from response data (>1) and convert them from hex to int
        for (unsigned int i = 1; i < (unsigned int)response_data.size(); i++) {
            response_args.push_back(std::to_string(std::stoi(response_data[i], 0, 16)));
        }

        if (response_type == "*") {
            // ACK
            std::string respondingID = response_args[0];
            return "<--@" + respondingID + " ACK";
        } else if (response_type == "!") {
            // NAK
            std::string respondingID = response_args[0];
            std::string commandBeingRespondedTo = response_args[1];
            std::string NAKCode = response_args[2];
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
                {10, "Bad Checksum"} };
            return "<--@" + respondingID + " " + commandBeingRespondedTo + " NAK " + codes[stoi(NAKCode)];
        } else if (response_type == "#") {
            // Data
            std::string respondingID = response_args[0];
            // int commandBeingRespondedTo = response_args_ints[1];
            std::string data_string = "";
            for (unsigned int i = 2; i < response_args.size(); i++) {
                data_string += response_args[i] + " ";
            }
            return "<--@" + respondingID + " Data " + data_string;
        } else {
            // Unknown response type
            std::cout << "-----" << response << std::endl;
            return "<-- Unknown response type";
        }
    }

    void WRI(int dataRegister, int data) {
        if (dataRegister == false) { // FIXME shouldnt this be NULL?
            std::cout << "Data register is invalid" << std::endl;
            return;
        }

        /// TODO Range of int takes care of this, but maybe we should check for longs?
        // make sure data is within range
        // if ((data < 0 || data > 4294967295) && (data < -2147483648 || data > 2147483647)) {
        //     std::cout << "Data is invalid $" << data << std::endl;
        //     return;
        // }

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
