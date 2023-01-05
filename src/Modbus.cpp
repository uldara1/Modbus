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


bool Modbus::init(int mode)
{
     this->mode_ =  mode;
     pinMode(mode_,OUTPUT);
     digitalWrite(mode_, 0);  
     return true;
}

byte Modbus::byteRead(int index)
{
  return dataRx[index];
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
   return inputRegisterRead(SlaveID , Input_Register, 1);
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






bool Modbus::requestFrom(int slaveId, int type, int address,int nb)
{
    
    address = address - 1;
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
    
    

      //  for(int i =0; i < 8; i++)
      //       {
      //           if(txout[i] < 16)
      //           {
      //               Serial.print("0");
      //           }
      //           Serial.print(txout[i],HEX);
      //       }
      //       Serial.println();

    
    digitalWrite(mode_,1);
    delay(1);
    this->s->write(txout,8);
    this->s->flush();
    digitalWrite(mode_,0);
    delay(1);
    long t = millis();
    lenRx   = 0;
    datalen = 0;
    byte ll = 0;
    byte rx;
    byte found = 0;
    
  
    while(millis() - t < 100){
       if(this->s->available())
       {
        rx = this->s->read();
       
        if(found == 0)
        {
          if(txout[ll] == rx){ll++;}else{ll = 0;}
          if(ll == 2)
          { 
            found = 1; 
          }
        }
        else if(found == 1){
        //  Serial.print("Len: ");
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
         if(lenRx >= rawRx[2] + 5)
         {
          break;
         }
        }
 
        t = millis();
       }

    }


        // for(int i =0; i < lenRx; i++)
        //     {
        //         if(rawRx[i] < 16)
        //         {
        //             Serial.print("0");
        //         }
        //         Serial.print(rawRx[i],HEX);
        //     }
        //     Serial.println();


    if(lenRx > 2){
        int crc1 = rawRx[lenRx - 1] <<8 | rawRx[lenRx - 2];
        int crc2 = CheckCRC(rawRx, lenRx - 2);

        if(crc1 == crc2)
        {
        
         datalen = rawRx[2];

         for(int i = 0; i < datalen;i++){
            dataRx[i] = rawRx[i+3];
         }



          return true;
        }else{
            return false;
        }
    }else{
        return false;
    }
}



void Modbus::RxRead(byte *raw, uint8_t &rlen)
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


void Modbus::TxRead(byte *raw, uint8_t &rlen)
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


int Modbus::CheckCRC(byte *buf, int len)
{
  int nominal = 0xA001;
  int crc = 0xFFFF;
  unsigned char pos,i;
 
  for ( pos = 0; pos < len; pos++) {
    crc ^= (unsigned int)buf[pos];          // XOR byte into least sig. byte of crc
 
    for (i = 8; i != 0; i--) {        // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= nominal;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}


    