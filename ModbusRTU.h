// ModbusRTU.h
// Kelvin Lo - 20250422
// develop for window platform to use modbusRTU

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <windows.h>
#include <vector>
#include <cstdint>
#include <string>
#include <iostream>

class ModbusRTU {
public:
    ModbusRTU(int com, int baudRate);
    ~ModbusRTU();

    bool connect();
    void disconnect();
    
    bool readCoils(uint8_t slaveId, uint16_t startAddress, uint8_t* buffer, size_t quantity);
    bool readDiscreteInputs(uint8_t slaveId, uint16_t startAddress, uint8_t* buffer, size_t quantity);
    bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, size_t quantity);
    bool readInputRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, size_t quantity);
    bool writeSingleCoil(uint8_t slaveId, uint16_t address, bool value);
    bool writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value);
    bool writeMultipleCoils(uint8_t slaveId, uint16_t startAddress, const std::vector<bool>& values);
    bool writeMultipleRegisters(uint8_t slaveId, uint16_t startAddress, const std::vector<uint16_t>& values);
    
    CString getLastError() const; // Function to return the last error message
	void resetSerialPort();

private:
    HANDLE hSerial;
    int dCom;
    int baudRate;
    mutable std::string errorString; // Store error messages

    bool sendRequest(const uint8_t* request, size_t requestLength);
    bool receiveResponse(uint8_t* response, size_t responseLength);
    uint16_t crc16(const uint8_t* buffer, size_t length);
    
    void buildReadRequest(uint8_t functionCode, uint8_t slaveId, uint16_t startAddress, uint16_t quantity, std::vector<uint8_t>& request);
    void buildWriteSingleCoilRequest(uint8_t slaveId, uint16_t address, bool value, std::vector<uint8_t>& request);
    void buildWriteSingleRegisterRequest(uint8_t slaveId, uint16_t address, uint16_t value, std::vector<uint8_t>& request);
    void buildWriteMultipleCoilsRequest(uint8_t slaveId, uint16_t startAddress, const std::vector<bool>& values, std::vector<uint8_t>& request);
    void buildWriteMultipleRegistersRequest(uint8_t slaveId, uint16_t startAddress, const std::vector<uint16_t>& values, std::vector<uint8_t>& request);
    void setErrorString(const std::string& message); // Set error message
};

// Implementation

inline ModbusRTU::ModbusRTU(int com, int baudRate)
    : dCom(com), baudRate(baudRate), hSerial(INVALID_HANDLE_VALUE) {}

inline ModbusRTU::~ModbusRTU() 
{
    disconnect();
}

inline bool ModbusRTU::connect() 
{
    CString sz_com;
	sz_com.Format("\\\\.\\COM%d", dCom);
    hSerial = CreateFile(sz_com,
                         GENERIC_READ | GENERIC_WRITE,
                         0,
                         NULL,
                         OPEN_EXISTING,
                         0,
                         NULL);

    if (hSerial == INVALID_HANDLE_VALUE) 
    {
        setErrorString("Error opening serial port.");
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) 
    {
        setErrorString("Error getting state.");
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) 
    {
        setErrorString("Error setting serial port state.");
        return false;
    }

	//Set to Timeouts
	COMMTIMEOUTS timeout;
	GetCommTimeouts(hSerial, &timeout);
	timeout.ReadIntervalTimeout = 10; //1
	timeout.ReadTotalTimeoutConstant = 10;
	timeout.ReadTotalTimeoutMultiplier = 1;
	timeout.WriteTotalTimeoutConstant = 10;
	timeout.WriteTotalTimeoutMultiplier = 1;
	if (!SetCommTimeouts(hSerial, &timeout))
	{
		setErrorString("Error setting serial timeout state.");
		return false;
	}

    return true;
}

inline void ModbusRTU::disconnect() 
{
	if (hSerial != INVALID_HANDLE_VALUE) 
	{
		CloseHandle(hSerial);
		hSerial = INVALID_HANDLE_VALUE;
	}
}

inline void ModbusRTU::buildReadRequest(uint8_t functionCode, uint8_t slaveId, uint16_t startAddress, uint16_t quantity, std::vector<uint8_t>& request) 
{
    request.clear();
    request.resize(6);
    request[0] = slaveId;
    request[1] = functionCode; // Function code
    request[2] = (startAddress >> 8) & 0xFF; // Start address high byte
    request[3] = startAddress & 0xFF;        // Start address low byte
    request[4] = (quantity >> 8) & 0xFF;     // Quantity high byte
    request[5] = quantity & 0xFF;            // Quantity low byte

    // Calculate CRC and append
    uint16_t crc = crc16(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
}

inline void ModbusRTU::buildWriteSingleCoilRequest(uint8_t slaveId, uint16_t address, bool value, std::vector<uint8_t>& request) 
{
    request.resize(8);
    request[0] = slaveId;
    request[1] = 0x05; // Function code for Write Single Coil
    request[2] = (address >> 8) & 0xFF; // Address high byte
    request[3] = address & 0xFF;        // Address low byte
    request[4] = value ? 0xFF : 0x00;   // ON if true, OFF if false
    request[5] = 0x00;                   // Always 0 for coils

    // Calculate CRC and append
    uint16_t crc = crc16(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
}

inline void ModbusRTU::buildWriteSingleRegisterRequest(uint8_t slaveId, uint16_t address, uint16_t value, std::vector<uint8_t>& request) 
{
    request.resize(8);
    request[0] = slaveId;
    request[1] = 0x06; // Function code for Write Single Register
    request[2] = (address >> 8) & 0xFF; // Address high byte
    request[3] = address & 0xFF;        // Address low byte
    request[4] = (value >> 8) & 0xFF;   // Value high byte
    request[5] = value & 0xFF;          // Value low byte

    // Calculate CRC and append
    uint16_t crc = crc16(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
}

inline void ModbusRTU::buildWriteMultipleCoilsRequest(uint8_t slaveId, uint16_t startAddress, const std::vector<bool>& values, std::vector<uint8_t>& request) 
{
    size_t quantity = values.size();
    request.resize(7 + (quantity + 7) / 8);
    request[0] = slaveId;
    request[1] = 0x0F; // Function code for Write Multiple Coils
    request[2] = (startAddress >> 8) & 0xFF;
    request[3] = startAddress & 0xFF;
    request[4] = (quantity >> 8) & 0xFF;
    request[5] = quantity & 0xFF;
    request[6] = (quantity + 7) / 8; // Byte count

    // Set coil values
    for (size_t i = 0; i < quantity; ++i) {
        if (values[i]) {
            request[7 + (i / 8)] |= (1 << (i % 8));
        }
    }

    // Calculate CRC and append
    uint16_t crc = crc16(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
}

inline void ModbusRTU::buildWriteMultipleRegistersRequest(uint8_t slaveId, uint16_t startAddress, const std::vector<uint16_t>& values, std::vector<uint8_t>& request) 
{
    size_t quantity = values.size();
    request.resize(7 + 2 * quantity);
    request[0] = slaveId;
    request[1] = 0x10; // Function code for Write Multiple Registers
    request[2] = (startAddress >> 8) & 0xFF;
    request[3] = startAddress & 0xFF;
    request[4] = (quantity >> 8) & 0xFF;
    request[5] = quantity & 0xFF;
    request[6] = 2 * quantity; // Byte count

    // Set register values
    for (size_t i = 0; i < quantity; ++i) 
    {
        request[7 + 2 * i] = (values[i] >> 8) & 0xFF;
        request[8 + 2 * i] = values[i] & 0xFF;
    }

    // Calculate CRC and append
    uint16_t crc = crc16(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
}

inline bool ModbusRTU::readCoils(uint8_t slaveId, uint16_t startAddress, uint8_t* buffer, size_t quantity) 
{
    std::vector<uint8_t> request;
    buildReadRequest(0x01, slaveId, startAddress, quantity, request);

    if (!sendRequest(request.data(), request.size())) 
    {
        return false;
    }

    uint8_t response[256];
    if (!receiveResponse(response, 5 + (quantity + 7) / 8)) 
    {
        return false;
    }

    if (response[0] != slaveId || response[1] != 0x01) 
    {
        setErrorString("Invalid response for read coils.");
        return false;
    }

    std::memcpy(buffer, &response[3], (quantity + 7) / 8);
    return true;
}

inline bool ModbusRTU::readDiscreteInputs(uint8_t slaveId, uint16_t startAddress, uint8_t* buffer, size_t quantity) 
{
    std::vector<uint8_t> request;
    buildReadRequest(0x02, slaveId, startAddress, quantity, request);

    if (!sendRequest(request.data(), request.size())) 
    {
        return false;
    }

    uint8_t response[256];
    if (!receiveResponse(response, 5 + (quantity + 7) / 8)) 
    {
        return false;
    }

    if (response[0] != slaveId || response[1] != 0x02) 
    {
        setErrorString("Invalid response for read discrete inputs.");
        return false;
    }

    std::memcpy(buffer, &response[3], (quantity + 7) / 8);
    return true;
}

inline bool ModbusRTU::readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, size_t quantity) 
{
    std::vector<uint8_t> request;
    buildReadRequest(0x03, slaveId, startAddress, quantity, request);

    if (!sendRequest(request.data(), request.size())) 
    {
        return false;
    }

    uint8_t response[256];
    if (!receiveResponse(response, 5 + 2 * quantity)) 
    {
        return false;
    }

    if (response[0] != slaveId || response[1] != 0x03) 
    {
        setErrorString("Invalid response for read holding registers.");
        return false;
    }

    for (size_t i = 0; i < quantity; ++i) 
    {
        buffer[i] = (response[3 + 2 * i] << 8) | response[4 + 2 * i];
    }

    return true;
}

inline bool ModbusRTU::readInputRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, size_t quantity) {
    std::vector<uint8_t> request;
    buildReadRequest(0x04, slaveId, startAddress, quantity, request);

    if (!sendRequest(request.data(), request.size())) 
    {
        return false;
    }

    uint8_t response[256];
    if (!receiveResponse(response, 5 + 2 * quantity)) 
    {
        return false;
    }

    if (response[0] != slaveId || response[1] != 0x04) 
    {
        setErrorString("Invalid response for read input registers.");
        return false;
    }

    for (size_t i = 0; i < quantity; ++i) 
    {
        buffer[i] = (response[3 + 2 * i] << 8) | response[4 + 2 * i];
    }

    return true;
}

inline bool ModbusRTU::writeSingleCoil(uint8_t slaveId, uint16_t address, bool value) 
{
    std::vector<uint8_t> request;
    buildWriteSingleCoilRequest(slaveId, address, value, request);
    
    return sendRequest(request.data(), request.size());
}

inline bool ModbusRTU::writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) 
{
    std::vector<uint8_t> request;
    buildWriteSingleRegisterRequest(slaveId, address, value, request);
    
    return sendRequest(request.data(), request.size());
}

inline bool ModbusRTU::writeMultipleCoils(uint8_t slaveId, uint16_t startAddress, const std::vector<bool>& values) 
{
    std::vector<uint8_t> request;
    buildWriteMultipleCoilsRequest(slaveId, startAddress, values, request);
    
    return sendRequest(request.data(), request.size());
}

inline bool ModbusRTU::writeMultipleRegisters(uint8_t slaveId, uint16_t startAddress, const std::vector<uint16_t>& values) 
{
    std::vector<uint8_t> request;
    buildWriteMultipleRegistersRequest(slaveId, startAddress, values, request);
    
    return sendRequest(request.data(), request.size());
}

inline bool ModbusRTU::sendRequest(const uint8_t* request, size_t requestLength) 
{
    DWORD bytesWritten;
    if (!WriteFile(hSerial, request, requestLength, &bytesWritten, NULL) || bytesWritten != requestLength) 
    {
        setErrorString("Error sending request.");
		resetSerialPort();
        return false;
    }
    return true;
}

#define MAX_MSG_LENGTH 260
inline bool ModbusRTU::receiveResponse(uint8_t* response, size_t responseLength) 
{
    DWORD bytesRead;
    if (!ReadFile(hSerial, response, MAX_MSG_LENGTH, &bytesRead, NULL) || bytesRead != responseLength) {
        setErrorString("Error receiving response.");
        return false;
    }
	resetSerialPort();
    return true;
}

inline uint16_t ModbusRTU::crc16(const uint8_t* buffer, size_t length) 
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < length; ++pos) {
        crc ^= (uint16_t)buffer[pos];
        for (size_t i = 8; i != 0; --i) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

inline void ModbusRTU::setErrorString(const std::string& message) 
{
    errorString = message;
}

inline CString ModbusRTU::getLastError() const 
{
    return errorString.c_str();
}

inline void ModbusRTU::resetSerialPort() {
	PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
}
#endif // MODBUS_RTU_H