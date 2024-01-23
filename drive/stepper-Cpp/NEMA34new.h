#include <iostream>
#include <string>
#include <vector>
#include <map>


#include "serialib.h"
#include "instructNames.h"


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

        if (response.empty())       //check if no response, if so, exit read func
        {
            std::cout << "[WARNING] No Reponse from QuickSilver" << std::endl;
            return "";
        }

        std::vector<std::string> response_data;
        std::string response_type = "";
        std::string response_args = "";
        std::string response_string = response;
        size_t pos = 0;
        std::string token;

        while ((pos = response_string.find(" ")) != std::string::npos) 
        {
            token = response_string.substr(0, pos);
            response_data.push_back(token);
            response_string.erase(0, pos + 1);
        }
        response_type = response_data[0];
        response_args = response_data[1];                                        //TODO: Find out why this crashes (parsing issue?)
        
        for (int i = 2; i < response_data.size(); i++) 
        {
            response_args += " " + response_data[i];
        }

        std::vector<int> response_args_int;
        pos = 0;

        while ((pos = response_args.find(" ")) != std::string::npos) 
        {
            token = response_args.substr(0, pos);
            response_args_int.push_back(std::stoi(token, nullptr, 16));
            response_args.erase(0, pos + 1);
        }

        if (response_type == "*") 
        {
            int respondingID = response_args_int[0];
            return "<--@" + std::to_string(respondingID) + " ACK";
        }
        else if (response_type == "!") 
        {
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
        else 
        {
            std::cout << "-----" << response << std::endl;
            return "<-- Unknown response type";
        }
    }
    void WRI(int dataRegister, int data) {
        
        if (dataRegister == false) 
        {
            std::cout << "Data register is invalid" << std::endl;
            return;
        }


        //make sure data is within range
        if ((data < 0 || data > 4294967295) && (data < -2147483648 || data > 2147483647)) 
        {
            std::cout << "Data is invalid $" << data << std::endl;
            return;
        }
        std::vector<int> args = { WRI_COMM, dataRegister, data };
        execute(args);
    }
    void PVC(int startingDataRegister, int stopEnable = 0, int stopState = 0, int mode = 0) {
        if (mode < -32678 || mode > 32767) 
        {
            std::cout << "Mode is invalid" << std::endl;
            return;
        }
        if (startingDataRegister < 11 || startingDataRegister > 98) 
        {
            std::cout << "Starting data register (" << startingDataRegister << ") is out of range" << std::endl;
            return;
        }
        if (stopEnable < 0 || stopEnable > 32767) 
        {
            std::cout << "Stop enable invalid" << std::endl;
            return;
        }
        if (stopState != 0 && stopState != 1 && stopState != 2 && stopState != 3) 
        {
            std::cout << "Stop state is invalid" << std::endl;
            return;
        }
        std::vector<int> args = { PVC_COMM, mode, startingDataRegister, stopEnable, stopState };
        execute(args);
    }
    void STP(int deceleration) {
        if (deceleration < -1 || deceleration > 536870911) 
        {
            std::cout << "Deceleration (" << deceleration << ") out of range" << std::endl;
            return;
        }
        std::vector<int> args = { STP_COMM, deceleration };
        execute(args);
    }
    void CPL(int address) {
        if (address < 0 || address > 65535) 
        {
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
        if (dataRegister < 0 || dataRegister > 60) 
        {
            std::cout << "Data register (" << dataRegister << ") is invalid" << std::endl;
            return;
        }
        std::vector<int> args = { RRG_COMM, dataRegister };
        execute(args);
    }
};


