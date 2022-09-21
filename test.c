#include<stdio.h>
#include<windows.h>

typedef struct ReceiveData1
{
    int AutoManualVar;
    int EStopVar;
    int GearVar;
    int SpeedVar=1;
    int SteeringVar;
    int BrakeVar;

    int32 EncoderVar;

}

ReceiveData;

HANDLE ERP42ComDevId;

char szPort[15]="\\\\.\\COM3";
unsigned char tx_d[14] = {0, };
unsigned char Sendbuf[14]={0,0,0,1,0,0,0,0,0,0,1,0,0,0};
unsigned char rx_d[18]={0, };
unsigned char rx_d_sensor1[18] = {0, };
unsigned int speed = 0;
unsigned char connectstate = 0;
DWORD dwByte;

void delay(int ms){
    Sleep(ms);
}
void Open(char *szport){
    if (connectstate == 0;){
        ERP42ComDevId = CreateFile(
            szport,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            0,
            NULL
        );

        COMMTIMEOUTS CommTimeOuts
        GetCommTimeouts(ERP42ComDevId, &CommTimeOuts);
        CommTimeOuts.ReadIntervalTimeout = 0;
        CommTimeOuts.ReadTotalTimeoutMultiplier=0;
        CommTimeOuts.ReadTotalTimeoutConstant = 0;
        CommTimeOuts.WriteTotalTimeoutConstant = 0;
        SetCommTimeouts(ERP42ComDevId, &CommTimeOuts);

        DCB dcb;
        GetCommState(ERP42ComDevId, &dcb);
        dcb.BaudRate = CBR_115200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = 1;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;

        SetCommState(ERP42ComDevId, &dcb);
        Sleep(1000);
        connectstate = 1; 
    }
}

int Alive(int aliveValue){
    Sendbuf[11] = (byte)aliveValue;
    aliveValue = aliveValue +1;

    return aliveValue;
}

void Close(){
    if (connectstate == 1){
        Sleep(1000);
        CloseHandle(ERP42ComDevId);
        connectstate = 0;
    }
}

void SendData(unsigned char *Sendbuf){
    tx_d[0]=0x53;
    tx_d[1]=0x54;
    tx_d[2]=0x58;
    tx_d[3]=Sendbuf[3];
    tx_d[4]=Sendbuf[4];
    tx_d[5]=Sendbuf[5];
    tx_d[6]=Sendbuf[6];
    tx_d[7]=Sendbuf[7];
    tx_d[8]=Sendbuf[8];
    tx_d[9]=Sendbuf[9];
    tx_d[10]=Sendbuf[10];
    tx_d[11]=Sendbuf[11];
    tx_d[12]=0x0D;
    tx_d[13]=0x0A;

    PurgeComm(ERP42ComDevId, PURGE_TXCLEAR);
    WriteFile(ERP42ComDevId, tx_d, 14, &dwByte, 0);
}

void Speed(int speedValue){
    Sendbuf[6]=0;
    Sendbuf[7]=0;

    Sendbuf[6]=(byte)(speedValue / 256);
    Sendbuf[7]=(byte)(speedValue % 256);

    SendData(Sendbuf);
}

void Steering(int steeringvalue){
    Sendbuf[8]=0;
    Sendbuf[9]=0;
    Sendbuf[8]=(byte)(steeringvalue / 256);
    Sendbuf[9]=(byte)(steeringvalue % 256);

    SendData(Sendbuf);
}

void AutoManual(int AutoManualValue){
    Sendbuf[3]=(byte)(AutoManualValue);
    Senddata(Sendbuf);
}

void EStop(int EstopValue){
    Sendbuf[4]=(byte)EstopValue;
    SendData(Sendbuf);
}

void Gear(int GearValue){
    Sendbuf[5]=(byte)GearValue;
    SendData(Sendbuf);
}

void Break(int BreakValue){
    Sendbuf[10]=(byte)BreakValue;
    SendData(Sendbuf);
}

void checkdata(){
    if((rx_d[0]==0x53)&&(rx_d[1]==0x54)&&(rx_d[2]==0x58)&&(rx_d[16]==0x0D)&&(rx_d[17]==0x0A)){
        
        rx_d_sensor1[3]=rx_d[3];
        rx_d_sensor1[4]=rx_d[4];
        rx_d_sensor1[5]=rx_d[5];
        rx_d_sensor1[6]=rx_d[6];
        rx_d_sensor1[7]=rx_d[7];
        rx_d_sensor1[8]=rx_d[8];
        rx_d_sensor1[9]=rx_d[9];
        rx_d_sensor1[10]=rx_d[10];
        rx_d_sensor1[11]=rx_d[11];
        rx_d_sensor1[12]=rx_d[12];
        rx_d_sensor1[13]=rx_d[13];
        rx_d_sensor1[14]=rx_d[14];

    }
}

ReceiveData RecData(){
    ReceiveData ReceiveData1;
    SendData(Sendbuf);

    ReadFile(ERP42ComDevId,rx_d, 18, &dwByte, 0);
    checkdata();

    PurgeComm(ERP42ComDevId, PURGE_RXCLEAR);

    ReceiveData1.AutoManualVar = rx_d_sensor1[3];
    ReceiveData1.EStopVar = rx_d_sensor1[4];
    ReceiveData1.GearVar = rx_d_sensor1[5];
    ReceiveData1.SpeedVar = rx_d_sensor1[7]*256+rx_d_sensor1[6];
    ReceiveData1.SteeringVar = rx_d_sensor1[9]*256 + rx_d_sensor1[8];
    ReceiveData1.BrakeVar = rx_d_sensor1[10];
    ReceiveData1.EncoderVar = (rx_d_sensor1[14]*256+rx_d_sensor1[13])*256+(rx_d_sensor1[12]*256+rx_d_sensor1[11]);

    return ReceiveData1;
}





























