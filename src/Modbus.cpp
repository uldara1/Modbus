#include <Modbus.h>
#include <Arduino.h>

Modbus::Modbus()
{
    this->s = NULL;
    this->mode_ = -1;
}

Modbus::Modbus(HardwareSerial &st)
{
    this->s = &st;
}

bool Modbus::init(int mode, bool en_log)
{
     this->mode_ =  mode;
     this->log   =  en_log;
     pinMode(mode_,OUTPUT);
     digitalWrite(mode_, 0);  
     
     return true;
}

void Modbus::setTimeout(uint16_t timeout)
{
  timeout_ = timeout;
}

byte Modbus::byteRead(int index)
{
  return rawRx[index+3];
}

int Modbus::blockRead(int index)
{
   return  ((dataRx[index*2] << 8) | dataRx[index*2+1]);
}

int Modbus::coilRead(int address){
 
    return coilRead(SlaveID,address);
}

int Modbus::coilRead(int id, int address){
   if(requestFrom(id,Coil_Register,address,1))
   {
    byte x = byteRead(0);
    return bitRead(x,0);
   }else
   {
    return -1;
   }
}

int Modbus::discreteInputRead(int address)
{
   return discreteInputRead(SlaveID,address);
}

int Modbus::discreteInputRead(int id, int address)
{
   if(requestFrom(id,Discret_Register,address,1))
   {
    byte x = byteRead(0);
    return bitRead(x,0);
   }else
   {
    return -1;
   }
}

long Modbus::holdingRegisterRead(int address)
{
  return holdingRegisterRead(SlaveID, address, 1);
}

long Modbus::holdingRegisterRead(int id, int address, int block)
{
  if(block > 2){block = 2;}
  if(requestFrom(SlaveID, Holding_Register, address, block))
  {
    if(block == 2)
    {
      return (blockRead(0) << 16 | blockRead(1));
    }
    else{
      return blockRead(0);
    }
  }
  else{
    return -1;
  }

}

long Modbus::inputRegisterRead(int address)
{
   return inputRegisterRead(SlaveID , address, 1);
}

long Modbus::inputRegisterRead(int id, int address, int block)
{
  if(block > 2){block = 2;}
  if(requestFrom(id, Input_Register,address,block))
  {
    if(block == 2)
    {
      return (blockRead(0) << 16 | blockRead(1));
    }
    else{
      return blockRead(0);
    }
  }
  else
  {
    return -1;
  }
}

int Modbus::requestFrom(int slaveId, int type, int address,int nb)
{
    
    // address = address - 1;
    int crc ;
    txout[0] = slaveId;
    txout[1] = type;
    txout[2] = address >> 8;
    txout[3] = address;
    txout[4] = nb >> 8;
    txout[5] = nb;
    crc = this->CheckCRC(txout,6);
    txout[6] = crc ;
    txout[7] = crc >> 8;
 
     
    if(log){
      Serial.print("TX: ");
       for(int i =0; i < 8; i++)
            {
                Serial.printf("%02X ",txout[i] );
            }
            Serial.print("\t");
     }

    digitalWrite(mode_,1);
    delay(1);
    this->s->write(txout,8);
    this->s->flush();
    digitalWrite(mode_,0);
    delay(1);
    uint32_t t = millis();
    lenRx   = 0;
    datalen = 0;
    int ll = 0;
    int rx;
    byte found = 0;
  
    while((millis() - t) < timeout_){
       if(this->s->available())
       {
        rx = this->s->read();
        t = millis();
        
        if(found == 0)
        {
          if(txout[ll] == rx){ll++;}else{ll = 0;}
          if(ll == 2)
          { 
            found = 1; 
          }
        }
        else if(found == 1){
        // Serial.print("Len: ");
        //  Serial.println(rx,DEC);
         rawRx[0] = txout[0];
         rawRx[1] = txout[1];
         rawRx[2] = rx;
         lenRx = 3;
         found = 2;
        } 
        else if(found == 2)
        {
         this->rawRx[lenRx++] =  rx;
         if(lenRx >= rawRx[2] + 5) { break; }
        }
        
       }
        

    }

    if(log){
        Serial.print("RX: ");
        for(int i =0; i < lenRx; i++)
            {
             Serial.printf("%02X ",rawRx[i] );
            }
            Serial.println();
     }

    if(lenRx > 2){
        int crc1 = rawRx[lenRx - 1] <<8 | rawRx[lenRx - 2];
        int crc2 = CheckCRC(rawRx, lenRx - 2);
        //Serial.printf("CRC1: %04X CRC2: %04X\n",crc1, crc2);

         if(crc1 == crc2)
          {
        
            datalen = rawRx[2];

        //  for(int i = 0; i < datalen;i++){
        //     dataRx[i] = rawRx[i+3];
        //   }
           return datalen;
          }
         else{ return -1; }
    }else{
        return -1;
    }
}

int Modbus::ReadCoilReg(int add)
{
    return ReadCoilReg(1,  add, 1);
}

int Modbus::ReadCoilReg(int slaveId, int add)
{
    return ReadCoilReg( slaveId, add, 1);
}

int Modbus::ReadCoilReg(int slaveId, int add, int nbit)
{
   if(requestFrom(slaveId,Coil_Register,add,nbit))
   {
    return byteRead(0);
   }else
   {
    return -1;
   }
 
}

int Modbus::ReadDiscretReg(int add)
{
    return ReadDiscretReg(1,add,1);
}

int Modbus::ReadDiscretReg(int slaveId, int add)
{
    return ReadDiscretReg(slaveId,add,1);
}

int Modbus::ReadDiscretReg(int slaveId, int add, int nbit)
{
    if(requestFrom(slaveId,Discret_Register,add,nbit)) {
    return byteRead(0);
   }
   else {
    return -1;
   }
}

int Modbus::ReadHoldingReg(int add)
{
    return 0;
}

int Modbus::ReadHoldingReg(int slaveId, int add)
{
    return 0;
}

int Modbus::ReadHoldingReg(int slaveId, int add, int nbyte)
{
    return 0;
}

int Modbus::ReadInputReg(int add)
{
    return 0;
}

int Modbus::ReadInputReg(int slaveId, int add)
{
    return 0;
}

int Modbus::ReadInputReg(int slaveId, int add, int nbyte)
{
    return 0;
}

uint8_t Modbus::uint8(int add)
{
    return rawRx[add*2+3];
}

int8_t Modbus::int8(int add)
{
    return rawRx[add*2+3];
}

uint16_t Modbus::uint16(int add)
{
    uint16_t add_ = (add)*2 + 3;
 
    return (rawRx[add_] << 8 | rawRx[add_+1]);
}

int16_t Modbus::int16(int add)
{
    int16_t add_ = (add)*2 + 3;
 
    return (rawRx[add_] << 8 | rawRx[add_+1]);
}

uint32_t Modbus::uint32(int add, bool byteHL)
{
    uint32_t val ;
    if (byteHL)
    {
      val = uint16(add) << 16 | uint16(add+1);
    }
    else
    {
      val = uint16(add+1)<< 16 | uint16(add);
    }
    return val;
}

void Modbus::RxRaw(byte *raw, uint8_t &rlen)
{
   
   for(int i =0; i < lenRx; i++)
    {
      raw[i] = rawRx[i];
      //  if(rawRx[i] < 16)
      //    {
      //      Serial.print("0");
      //     }
      //     Serial.print(rawRx[i],HEX);
    }
     rlen = this->lenRx;
    //  Serial.println(rlen);
}

void Modbus::TxRaw(byte *raw, uint8_t &rlen)
{
   
   
   for(int i =0; i < 8; i++)
    {
      raw[i] = txout[i];
      //  if(rawRx[i] < 16)
      //    {
      //      Serial.print("0");
      //     }
      //     Serial.print(rawRx[i],HEX);
    }
     rlen = 8;
    //  Serial.println(rlen);
}

uint16_t Modbus::CheckCRC(uint8_t *buf, uint16_t len) {
    // from https://www.modbustools.com/modbus_crc16.htm -> https://www.modbustools.com/modbus.html#crc
    static const uint16_t wCRCTable[] = {
       0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
       0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
       0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
       0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
       0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
       0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
       0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
       0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
       0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
       0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
       0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
       0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
       0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
       0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
       0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
       0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
       0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
       0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
       0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
       0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
       0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
       0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
       0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
       0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
       0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
       0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
       0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
       0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
       0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
       0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
       0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
       0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (len--) {
        nTemp = *buf++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}


    