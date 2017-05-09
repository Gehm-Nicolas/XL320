#include "XL320.h"
#include "DCMotor.h"

XL320::XL320(long baudRate, unsigned char left_id, unsigned char right_id, HardwareSerial &serIn): _serial(serIn)
{
  this->baudRate = baudRate;
  this->left_id = left_id;
  this->right_id = right_id;
  this->received_id = 0;
  this->instruction = DXL_NO_DATA;
  this->total_parameters = 0;
}

void XL320::init()
{
  _serial.begin(this->baudRate);
  //_serial.setTimeOut();
}

void XL320::end()
{
  _serial.end();
}

int XL320::checkMessages()
{
  int c;
  int state = 0;
  int pktLen = 0;
  unsigned char sum_params = 0;
  unsigned char dst_id = 0;
  int quit = 0;
  int index = 0;
  this->instruction = DXL_NO_DATA;

  for(index=0;index<32;index++)this->parameters[index]=0;

  index = 0;

  while (!quit)
  {
    c = _serial.read();
    if (c != -1)
    {
      switch (state)
      {
        case 0:
        case 1: if ((unsigned char)c == 0xFF){
                  state = state + 1;
                }else{
                  state = 0;
                  this->total_parameters = 0;
                  quit = 1;
                }
                break;
        case 2: if ((unsigned char)c == 0xFD){
                  state = state + 1;
                }else{
                  state = 0;
                  this->total_parameters = 0;
                  quit = 1;
                }
                break;
        case 3: if((unsigned char)c == 0x00){
                  state = state + 1;
                }else{
                  state = 0;
                  this->total_parameters = 0;
                  quit = 1;
                }
                break;
        case 4: dst_id = (unsigned char)c;
                instruction = DXL_NO_DATA;
                sum_params = 0;
                if(dst_id == this->left_id || dst_id == this->right_id){
                  this->received_id = dst_id;
                  state = state + 1;
                  //TODO: Broadcast id (0xFE)
                }else{
                  state = 0;
                  this->total_parameters = 0;
                  quit = 1;
                }
                break;
        case 5: pktLen = c & 0x00FF;
                state = state + 1;
                break;
        case 6: pktLen = pktLen + ((c & 0x00FF)<<8);
                if(pktLen < 3){
                  state = 0;
                  this->total_parameters = 0;
                  quit = 1;
                  break;
                }
                this->total_parameters = pktLen - 3;//3==instruction+crcH+crcL
                state = state + 1;
                break;
        case 7: pktLen = pktLen - 1;
                this->instruction = (unsigned char)c;
                sum_params = sum_params + c;
                if (pktLen == 2){
                  state = 9;
                }else{
                  state = state + 1;
                }
                break;
        case 8: pktLen = pktLen - 1;
                this->parameters[index]=(unsigned char)c;
                if (pktLen == 2){
                  state = state + 1;
                }else{
                  index = index + 1;
                }
                break;
        case 9: crc = c & 0x00FF;
                state = state + 1;
                break;
        case 10:crc = crc + ( (c & 0x00FF) <<8);
                //TODO:Verify CRC
                state = 0;
                quit = 1;
                break;
        default:state = 0;
                quit = 1;
                break;
        }
        if(quit == 1) serialFlush();
      }else{
        state = 0;
        quit = 1;
        this->total_parameters = 0;
      }
    }
    //TODO check this Return
    return 0;
  }

int XL320::dataLength(int kindOfData)
  {
    /*
    Return the quantity of bytes of each kind of Data present on Servo (XL320) memory.
    All data use 1 or 2 bytes to be saved.
    */
    if (kindOfData == DXL_MODEL_NUMBER    || kindOfData == DXL_CW_ANGLE_LIMIT ||
        kindOfData == DXL_CCW_ANGLE_LIMIT || kindOfData == DXL_MAX_TORQUE ||
        kindOfData == DXL_GOAL_POSITION   || kindOfData == DXL_GOAL_SPEED ||
        kindOfData == DXL_GOAL_TORQUE     || kindOfData == DXL_PRESENT_POSITION ||
        kindOfData == DXL_PRESENT_SPEED   || kindOfData == DXL_PRESENT_LOAD ||
        kindOfData == DXL_PUNCH){
      return 2;
    }else{
      return 1;
    }
  }

int XL320::makeReturnPacket(unsigned char* arrayAddr, unsigned char id, unsigned char instruction, int memAddress, unsigned char error, unsigned int data)
{
  int data_len = dataLength(memAddress);
  unsigned char* message = arrayAddr;
  unsigned short crc = 0;

  for (int i=0; i<32; i++) message[i] = 0;
  message[0] = 0xFF;
  message[1] = 0xFF;
  message[2] = 0xFD;
  message[3] = 0x00;
  message[4] = id;

  switch (instruction) {
    case DXL_WRITE_DATA:message[5] = 0x04;  //LSB (pkt_length)
                        message[6] = 0x00;  //MSB (pkt_length)
                        message[7] = DXL_STATUS;
                        message[8] = error;
                        crc = crc16(message,message[5]+7);
                        message[9] = crc & 0x00FF; //LSB
                        message[10] = (crc & 0xFF00) >> 8;     //MSB
                        return 10;
                        break;
    case DXL_READ_DATA: message[5] = data_len+4;//LSB -> PKT_LENGTH(instruction+error+data_length+16bit_CRC)
                        message[6] = 0x00;      //MSB
                        message[7] = DXL_STATUS;
                        message[8] = error;
                        if(data_len == 1){
                          message[9] = data;
                          crc = crc16(message,message[5]+7);
                          message[10] = crc & 0x00FF; //LSB
                          message[11] = (crc & 0xFF00) >> 8; //MSB
                          return 11;
                        }else{
                          message[9] = data & 0x00FF;         //LSB
                          message[10] = (data & 0xFF00) >> 8;  //MSB
                          crc = crc16(message,message[5]+7);
                          message[11] = crc & 0x00FF;         //LSB
                          message[12] = (crc & 0xFF00) >> 8;  //MSB
                          return 12;
                        }
                        break;
    default:            message[5] = 0x04;  //LSB (pkt_length)
                        message[6] = 0x00;  //MSB (pkt_length)
                        message[7] = DXL_STATUS;
                        message[8] = DXL_ERROR_INVALID_INSTRUCTION;
                        crc = crc16(message,message[5]+7);
                        message[9] = crc & 0x00FF; //LSB
                        message[10] = (crc & 0xFF00) >> 8;     //MSB
                        return 10;
                        break;
  }
}

unsigned short XL320::crc16(unsigned char* data_blk,int data_blk_size)
{
  /*in: data_blk -  entire packet except last 2 crc bytes
    out: crc_accum - 16 bytes (1 word)*/
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

  unsigned short crc_accum = 0;
  int i = 0;
  for(int j=0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

/*unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for(j = 0; j < data_blk_size; j++)
  {
      i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
      crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}*/


void XL320::sendStatusPacket(unsigned char id, unsigned char instruction,int memAddress, unsigned char error, unsigned int data)
  {
    unsigned char statusPkt[32];
    int pktLength = makeReturnPacket(statusPkt,id,instruction,memAddress,error,data);

    _serial.flush();//wait data to be sent?*/
    _serial.write((uint8_t *)statusPkt,(pktLength+1));
    //_serial.flush();//wait data to be sent?*/
  }

void XL320::serialFlush(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}
