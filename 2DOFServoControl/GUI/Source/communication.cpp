#include "communication.h"
#include<windows.h>
#include<stdio.h>
#include <QDebug>

static Communication* comptr;

/* Constructor */
Communication::Communication()
{

}

Communication::~Communication()
{
    delete comptr;
}

bool Communication::OpenPort(const char* PortName, DWORD Baudrate, BYTE DataBit, BYTE StopBit, BYTE PARITY)
{
    /* Check if comptr is defined*/
    if (comptr == NULL)
        return false;

    /* Set Portname.
     * Create Handle
     * Set Port Timeouts
     * Set Port Config
     */
    comptr->CommPort = PortName;
    comptr->CommHandle = CreateFileA(comptr->CommPort,                 // Name of the Port to be Opened
                                    GENERIC_READ | GENERIC_WRITE,      // Read/Write Access
                                    0,                                 // No Sharing, ports cant be shared
                                    NULL,                              // No Security
                                    OPEN_EXISTING,                     // Open existing port only
                                    0,                                 // Non Overlapped I/O
                                    NULL);                             // Null for Comm Devices

    if (comptr->CommHandle == INVALID_HANDLE_VALUE)
    {
        qDebug()<<"error open port" <<comptr->CommPort;
        printf("\n   Error! - Port %s can't be opened", comptr->CommPort);
        return false;
    }

        printf("opening serial port successful");
        if (!comptr->SetPortTimeouts()){
                qDebug() << "\n Timeouts not succesfully intialized";
                printf("\n Timeouts not succesfully intialized");
        }
        else{
                qDebug() << "\n Timeouts  succesfully intialized";
                printf("\n Timeouts  succesfully intialized");

        }
        return comptr->SetPortConfig(Baudrate, DataBit, StopBit, PARITY);

}

bool Communication::SetPortTimeouts()
{

    COMMTIMEOUTS timeouts = {0x0};

    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(comptr->CommHandle, &timeouts) == FALSE){
        qDebug() << "\n   Error! in Setting Time Outs";
        printf("\n   Error! in Setting Time Outs");
            return false;
    }
    qDebug()<<"\n\n   Setting Serial Port Timeouts Successfull";
    printf("\n\n   Setting Serial Port Timeouts Successfull");
    return true;
}


bool Communication::SetPortConfig(DWORD Baudrate, BYTE DataBit, BYTE StopBit, BYTE PARITYFLAG)
{
/* Set DCB params of the port
 *
 */

DCB dcbSerialParams       = {0x0};                        // Initializing DCB structure
dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

comptr->CommStatus = GetCommState(comptr->CommHandle, &dcbSerialParams);
if (!comptr->CommStatus){
    qDebug()<<"\n   Error! in GetCommState()";
    printf("\n   Error! in GetCommState()");
}

dcbSerialParams.BaudRate = Baudrate;      // Setting BaudRate = 9600
dcbSerialParams.ByteSize = DataBit;             // Setting ByteSize = 8
dcbSerialParams.StopBits = StopBit ;    // Setting StopBits = 1
dcbSerialParams.Parity   = PARITYFLAG;      // Setting Parity = None

comptr->CommStatus = SetCommState(comptr->CommHandle, &dcbSerialParams);

        if (!comptr->CommStatus)
            {
                qDebug()<<"\n error dcb";
                printf("\n   Error! in Setting DCB Structure");

            }

        printf("\n   Setting DCB Structure Successfull\n");
        printf("\n       Baudrate = %d", dcbSerialParams.BaudRate);
        printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
        printf("\n       StopBits = %d", dcbSerialParams.StopBits);
        printf("\n       Parity   = %d", dcbSerialParams.Parity);
        qDebug()<<"Baudrate"<<dcbSerialParams.BaudRate<<"ByteSize"<<dcbSerialParams.ByteSize<<"StopBits"<<dcbSerialParams.StopBits<<"parity"<<dcbSerialParams.Parity;


        if (!comptr->CommStatus)
            {
                printf("\n   Error! in Setting DCB Structure");
                qDebug()<<"error dcb";
                return false;
            }
        qDebug()<<"dcb configured";
        return true;
}


size_t Communication::len(const char* buffer){
    size_t cnt = 0;
    while(*buffer!='\0'){
        cnt++;
        buffer++;
    }
    return cnt;
}

bool Communication::WritePort(const char* cBuffer)
{
    //Buffer should be  char or byte array, otherwise write wil fail

    DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
    DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
    dNoOFBytestoWrite = (DWORD)len(cBuffer);// Calculating the no of bytes to write into the port
    comptr->CommStatus = WriteFile(comptr->CommHandle,
                                    cBuffer,            // Data to be written to the port
                                    dNoOFBytestoWrite,   // No of bytes to write into the port
                                    &dNoOfBytesWritten,  // No of bytes written to the port
                                    NULL);
    return true;

}

SSIZE_T  Communication::ReadPort(char* Buffer, size_t size)
{
    DWORD received;
    comptr->CommStatus = ReadFile(comptr->CommHandle,
                                  Buffer,
                                  size,
                                  &received,
                                  NULL);
    if (!comptr->CommStatus){
        printf("Unable to Read");
        return 0;
    }
    return received;

}

bool Communication::ClosePort()
{
    if (!comptr->CommStatus){
        printf("Closing handle \n");
        CloseHandle(comptr->CommHandle);
        return true;
    }
    printf("Already closed \n");
    return false;
}



/*Get SerialPort Instance*/
Communication *Communication::GetSerialPortInstance()
{
    if (comptr == NULL)
        comptr = new Communication();
    return comptr;
}


/* Return handle*/
HANDLE Communication::GetCommHandle()
{
    return CommHandle;
}

/* return Com name*/
const char* Communication::GetCommPort()
{
    return CommPort;
}

/* Return status of Com*/
bool Communication::GetCommStatus()
{
    return CommStatus;
}

