/*
    Please refer to: 
        - https://aticleworld.com/serial-port-programming-using-win32-api/
        - https://docs.microsoft.com/en-us/previous-versions/ff802693(v=msdn.10)?redirectedfrom=MSDN
    for and idea on how this abstraction was written
*/



#pragma once
#include <Windows.h>

class SerialPort
{
	private:
        HANDLE hPort;  // Handle to the Serial port
        BOOL   Status; // Status
        DCB dcbSerialParams;   //DCB structure
        COMMTIMEOUTS timeouts;  //timeouts structure
        DWORD BytesWritten;          // No of bytes written to the port
        DWORD dwEventMask;     // Event mask to trigger
        char  ReadData;        //temperory Character
        DWORD NoBytesRead;     // Bytes read by ReadFile()
        unsigned char loop;

        //wide character type
        wchar_t PortNo[20]; //contain friendly name

    public:
        SerialPort();               //default constructor (does not open port yet)
        int open(char * portname);  //fxn to open user specified port
        void write(char * buff);     //fxn to write to port
        void close();                //fxn to close port

};

SerialPort::SerialPort() {
    //initializes things
    this->BytesWritten = 0;          // No of bytes written to the port
    this->loop = 0;                  // read loop
    this->dcbSerialParams = { 0 };   // dcb struct for timeouts

    //initializes buffers
    memset(this->PortNo, 0, sizeof(this->PortNo));           // buffer to contain complete port name
    
}

int SerialPort::open(char * port_name) {

    //convert char to wchart_t
    size_t newsize = strlen(port_name) + 1;
    wchar_t * buff = new wchar_t[newsize];      // buffer before making padding with correct info
    mbstowcs(buff, port_name, newsize); //converting to wchar_t
    
    //wchar_t* buff = L"COM8";
    swprintf_s(this->PortNo, 20, L"\\\\.\\%s",buff);  // make port name recognizable to windows

    this->hPort = CreateFile(this->PortNo, //friendly name
        GENERIC_READ | GENERIC_WRITE,      // Read/Write Access
        0,                                 // No Sharing, ports cant be shared
        NULL,                              // No Security
        OPEN_EXISTING,                     // Open existing port only
        0,                                 // Non Overlapped I/O
        NULL);                             // Null for Comm Devices


    //error handling incase port cannot be opened
    if (this->hPort == INVALID_HANDLE_VALUE) {
        OutputDebugStringA("\nError: cannot open port\n");
        return -1;
    }


    //Setting the Parameters for the SerialPort
    this->dcbSerialParams.DCBlength = sizeof(this->dcbSerialParams);
    this->Status = GetCommState(this->hPort, &this->dcbSerialParams); //retreives  the current settings
    if (this->Status == FALSE){
        OutputDebugStringA("\nError in getting com port state\n\n");
        return -1;
    }

    
    dcbSerialParams.BaudRate = CBR_9600;      //BaudRate = 9600
    dcbSerialParams.ByteSize = 8;             //ByteSize = 8
    dcbSerialParams.StopBits = ONESTOPBIT;    //StopBits = 1
    dcbSerialParams.Parity = NOPARITY;      //Parity = None
    Status = SetCommState(hPort, &dcbSerialParams);


    if (Status == FALSE){
        OutputDebugStringA("\nError in setting DCB struct\n\n");
        return -1;
    }

    //Setting Timeouts
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if (SetCommTimeouts(hPort, &timeouts) == FALSE)
    {
        OutputDebugStringA("\nError in setting time outs\n\n");
        return -1;
    }

    OutputDebugStringA("\nOpening port success\n");
    return 0;
}

void SerialPort::write(char* buff) {
    
    char SerialBuffer[128] = { 0 };     
    sprintf(SerialBuffer, "%s\n", buff);

    Status = WriteFile(this->hPort,// Handle to the Serialport
        SerialBuffer,            // Data to be written to the port
        sizeof(SerialBuffer),   // No of bytes to write into the port
        &BytesWritten,          // No of bytes written to the port
        NULL);
}
void SerialPort::close() {
    CloseHandle(this->hPort);
}