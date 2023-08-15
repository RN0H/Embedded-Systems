#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include<windows.h>

class Communication
{

private:

    HANDLE CommHandle;
    const char*  CommPort;
    bool   CommStatus;

public:

    Communication();
   ~Communication();

    static Communication* GetSerialPortInstance();
    HANDLE GetCommHandle();
    const char*  GetCommPort();
    bool   GetCommStatus();

    bool   OpenPort(const char* PortName, DWORD Baudrate, BYTE DataBit, BYTE StopBit, BYTE PARITYFLAG);
    bool   WritePort(const char* );
    SSIZE_T   ReadPort(char* Buffer, size_t size);
    bool   ClosePort();

    bool SetPortConfig(DWORD Baudrate, BYTE DataBit, BYTE StopBit, BYTE PARITY);
    bool SetPortTimeouts();

    size_t len(const char* );

};



#endif // COMMUNICATION_H
