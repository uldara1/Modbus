#ifndef MODBUS_h
#define MODBUS_h
#include <Arduino.h>
#include <Stream.h>

class Modbus
{
private:
    /* data */
    int mode_ = -1;
    HardwareSerial* s ;
    byte rawRx[200];
    int  lenRx = 0;
    byte dataRx[100];
    int  datalen = 0;
    int  SlaveID = 0x01;
    byte txout[9] = {0,0,0,0,0,0,0,0,0};
    #define Coil_Register       0x01
    #define Discret_Register    0x02
    #define Holding_Register    0x03
    #define Input_Register      0x04


    

public:
    Modbus();
    Modbus(HardwareSerial &st);
    
    bool init(int mode);
    byte byteRead(int nb);
    int blockRead(int index);
    int coilRead(int address);                                      //Return 1 byte = 8 bit coil
    int coilRead(int id, int address);
    int discreteInputRead(int address);
    int discreteInputRead(int id, int address);
    long holdingRegisterRead(int address);
    long holdingRegisterRead(int id, int address, int block);
    long inputRegisterRead(int address);
    long inputRegisterRead(int id, int address, int block);
    
    int coilWrite(int address, uint8_t value);
    int coilWrite(int id, int address, uint8_t value);
    int holdingRegisterWrite(int address, uint16_t value);
    int holdingRegisterWrite(int id, int address, uint16_t value);\
    void RxRead(byte *raw, uint8_t &rlen);
    void TxRead(byte *raw, uint8_t &rlen);
    //Read multiple coils, discrete inputs, holding registers, or input register values.
    //int requestFrom(int type, int address, int nb, byte *ret,int len);
    bool requestFrom(int slaveId, int type, int address,int nb);
    //  ~Modbus();
    int CheckCRC(byte *buf, int len);
};





// Modbus::Modbus(/* args */)
// {
// }

// Modbus::~Modbus()
// {
// }



#endif
