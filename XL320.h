#ifndef XL320_h
#define XL320_h

#define LIBRARY_VERSION 1.0.0

#include <HardwareSerial.h>
#include <Arduino.h>

#define DXL_NO_DATA     0xFF
#define DXL_PING        0x01
#define DXL_READ_DATA   0x02
#define DXL_WRITE_DATA  0x03
#define DXL_REG_WRITE   0x04
#define DXL_ACTION      0x05
#define DXL_RESET       0x06
#define DXL_REBOOT      0x08
#define DXL_STATUS      0x55//Return instruction for Instruction Packet
#define DXL_SYNC_READ   0x82
#define DXL_SYNC_WRITE  0x83
#define DXL_BULK_READ   0x92
#define DXL_BULK_WRITE  0x93

#define DXL_ERROR_OVERLOAD              1
#define DXL_ERROR_OVER_HEATING          2
#define DXL_ERROR_INPUT_VOLTAGE         3
#define DXL_ERROR_INVALID_INSTRUCTION   4

//Memory Addresses
// -------- EEPROM --------//
#define DXL_MODEL_NUMBER      0
#define DXL_FIRMWARE_VERSION  2
#define DXL_ID                3
#define DXL_BAUD_RATE         4
#define DXL_DELAY_TIME        5
#define DXL_CW_ANGLE_LIMIT    6   // min angle, default 0
#define DXL_CCW_ANGLE_LIMIT   8   // max angle, default 300
#define DXL_CONTROL_MODE      11  // joint or wheel mode, default joint (servo)
#define DXL_TEMP_LIMIT        12
#define DXL_VOLT_LOW_LIMIT    13
#define DXL_VOLT_HIGH_LIMIT   14
#define DXL_MAX_TORQUE        15
#define DXL_RETURN_LEVEL      17
#define DXL_ALARM_SHUTDOWN    18
// -------- RAM -----------//
#define DXL_TORQUE_ENABLE     24
#define DXL_LED               25
#define DXL_D_GAIN            27
#define DXL_I_GAIN            28
#define DXL_P_GAIN            29
#define DXL_GOAL_POSITION     30
#define DXL_GOAL_SPEED        32
#define DXL_GOAL_TORQUE       35
#define DXL_PRESENT_POSITION  37
#define DXL_PRESENT_SPEED     39
#define DXL_PRESENT_LOAD      41
#define DXL_PRESENT_VOLTAGE   45
#define DXL_PRESENT_TEMP      46
#define DXL_REGISTERED_INST   47
#define DXL_MOVING            49
#define DXL_HW_ERROR_STATUS   50
#define DXL_PUNCH             51

class XL320 {
  public:
    XL320(long baudRate, unsigned char left_id, unsigned char right_id, HardwareSerial &serIn);
    void init();
    void end();
    int checkMessages();
    int dataLength(int kindOfData);

    void sendStatusPacket(unsigned char id, unsigned char instruction,int memAddress, unsigned char error, unsigned int data);
    int makeReturnPacket(unsigned char* arrayAddr, unsigned char id, unsigned char instruction, int memAddress, unsigned char error, unsigned int data);
    unsigned short crc16(unsigned char* data_blk,int data_blk_size);
    void serialFlush();//cleans the data in buffer

    long baudRate;
    unsigned char received_id;
    unsigned char right_id;
    unsigned char left_id;
    unsigned char instruction;
    unsigned char parameters[32];
    unsigned char total_parameters;
    unsigned char crc;
    HardwareSerial& _serial;
};

#endif
