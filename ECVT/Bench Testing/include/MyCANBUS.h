#include <FlexCAN_T4.h>

#define CANBaudRate 1000000

#define MOTEC_id0   0x100
#define MOTEC_id1   0x101
#define MOTEC_id2   0x102
#define MOTEC_id3   0x103
#define MOTEC_id4   0x104
#define MOTEC_id5   0x105
#define MOTEC_id6   0x106
#define MOTEC_id7   0x107
#define MOTEC_id8   0x108
#define MOTEC_id9   0x109
#define MOTEC_idA   0x10A
#define MOTEC_idB   0x10B
#define MOTEC_idC   0x10C
#define MOTEC_idD   0x10D
#define MOTEC_idE   0x10E
#define MOTEC_idF   0x10F

#define DCM_id0   0x110 // I2C BQ25730
#define DCM_id1   0x111
#define DCM_id2   0x112
#define DCM_id3   0x113
#define DCM_id4   0x114
#define DCM_id5   0x115
#define DCM_id6   0x116
#define DCM_id7   0x117
#define DCM_id8   0x118
#define DCM_id9   0x119
#define DCM_idA   0x11A
#define DCM_idB   0x11B
#define DCM_idC   0x11C
#define DCM_idD   0x11D
#define DCM_idE   0x11E
#define DCM_idF   0x11F

#define PDM_id0   0x120
#define PDM_id1   0x121
#define PDM_id2   0x122
#define PDM_id3   0x123
#define PDM_id4   0x124
#define PDM_id5   0x125
#define PDM_id6   0x126
#define PDM_id7   0x127
#define PDM_id8   0x128
#define PDM_id9   0x129
#define PDM_idA   0x12A
#define PDM_idB   0x12B
#define PDM_idC   0x12C
#define PDM_idD   0x12D
#define PDM_idE   0x12E
#define PDM_idF   0x12F

#define TCU_id0   0x130 //RPM
#define TCU_id1   0x131
#define TCU_id2   0x132
#define TCU_id3   0x133
#define TCU_id4   0x134
#define TCU_id5   0x135
#define TCU_id6   0x136
#define TCU_id7   0x137
#define TCU_id8   0x138
#define TCU_id9   0x139
#define TCU_idA   0x13A
#define TCU_idB   0x13B
#define TCU_idC   0x13C
#define TCU_idD   0x13D
#define TCU_idE   0x13E
#define TCU_idF   0x13F


FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

/**
 * @brief 
 * * @param id 11bit standard id
 * * @param x an array of uint8_t
 */
struct CAN_Data
{
    uint16_t id = 0;
    uint8_t x[8] = {0};
};





void MyCanStart(bool chip);
bool MyCanSend(CAN_Data Data);
void MOTEC_Received(const CAN_message_t &msg);
void TCU_Received(const CAN_message_t &msg);
void DCM_Received(const CAN_message_t &msg);
void PDM_Received(const CAN_message_t &msg);



CAN_Data MOTEC_Data_RX0;
CAN_Data MOTEC_Data_RX1;
CAN_Data MOTEC_Data_RX2;
CAN_Data MOTEC_Data_RX3;
CAN_Data MOTEC_Data_RX4;
CAN_Data MOTEC_Data_RX5;
CAN_Data MOTEC_Data_RX6;
CAN_Data MOTEC_Data_RX7;
CAN_Data MOTEC_Data_RX8;
CAN_Data MOTEC_Data_RX9;
CAN_Data MOTEC_Data_RXA;
CAN_Data MOTEC_Data_RXB;
CAN_Data MOTEC_Data_RXC;
CAN_Data MOTEC_Data_RXD;
CAN_Data MOTEC_Data_RXE;
CAN_Data MOTEC_Data_RXF;

CAN_Data DCM_Data_RX0;
CAN_Data DCM_Data_RX1;
CAN_Data DCM_Data_RX2;
CAN_Data DCM_Data_RX3;
CAN_Data DCM_Data_RX4;
CAN_Data DCM_Data_RX5;
CAN_Data DCM_Data_RX6;
CAN_Data DCM_Data_RX7;
CAN_Data DCM_Data_RX8;
CAN_Data DCM_Data_RX9;
CAN_Data DCM_Data_RXA;
CAN_Data DCM_Data_RXB;
CAN_Data DCM_Data_RXC;
CAN_Data DCM_Data_RXD;
CAN_Data DCM_Data_RXE;
CAN_Data DCM_Data_RXF;
CAN_Data DCM_Data_TX;

CAN_Data PDM_Data_RX0;
CAN_Data PDM_Data_RX1;
CAN_Data PDM_Data_RX2;
CAN_Data PDM_Data_RX3;
CAN_Data PDM_Data_RX4;
CAN_Data PDM_Data_RX5;
CAN_Data PDM_Data_RX6;
CAN_Data PDM_Data_RX7;
CAN_Data PDM_Data_RX8;
CAN_Data PDM_Data_RX9;
CAN_Data PDM_Data_RXA;
CAN_Data PDM_Data_RXB;
CAN_Data PDM_Data_RXC;
CAN_Data PDM_Data_RXD;
CAN_Data PDM_Data_RXE;
CAN_Data PDM_Data_RXF;
CAN_Data PDM_Data_TX;

CAN_Data TCU_Data_RX0;
CAN_Data TCU_Data_RX1;
CAN_Data TCU_Data_RX2;
CAN_Data TCU_Data_RX3;
CAN_Data TCU_Data_RX4;
CAN_Data TCU_Data_RX5;
CAN_Data TCU_Data_RX6;
CAN_Data TCU_Data_RX7;
CAN_Data TCU_Data_RX8;
CAN_Data TCU_Data_RX9;
CAN_Data TCU_Data_RXA;
CAN_Data TCU_Data_RXB;
CAN_Data TCU_Data_RXC;
CAN_Data TCU_Data_RXD;
CAN_Data TCU_Data_RXE;
CAN_Data TCU_Data_RXF;
CAN_Data TCU_Data_TX;




/**
 * @brief Starts CAN2
 * * @param chip true for PDM or DCM
 */
void MyCanStart(bool chip)
{
    if(chip)
    {
        pinMode(2,OUTPUT);                // CAN ShutDown
        pinMode(3,OUTPUT);                // CAN StandBy 
        digitalWrite(2,LOW);              // Low is normal operations
        digitalWrite(3,LOW);              // Low is normal operations
    }
    
    Can2.begin();
    Can2.setBaudRate(CANBaudRate);  // 1MHz
    Can2.setMaxMB(5);           // 1 TX MB & 4 RX

    // Setting up the MailBoxes
    Can2.setMB(MB0, TX, STD);
    Can2.setMB(MB1, RX, STD);
    Can2.setMB(MB2, RX, STD);
    Can2.setMB(MB3, RX, STD);
    Can2.setMB(MB4, RX, STD);

    // Adding the filters to each MailBox
    Can2.setMBFilterRange(MB1, MOTEC_id0,   MOTEC_idF);   // MOTEC
    Can2.setMBFilterRange(MB2, DCM_id0,     DCM_idF);     // DCM
    Can2.setMBFilterRange(MB3, PDM_id0,     PDM_idF);     // PDM
    Can2.setMBFilterRange(MB4, TCU_id0,     TCU_idF);     // TCU


    

    //What to do when receiving a message 
    Can2.onReceive(MB1, MOTEC_Received);        // MOTEC
    Can2.onReceive(MB2, DCM_Received);          // DCM
    Can2.onReceive(MB3, PDM_Received);          // PDM
    Can2.onReceive(MB4, TCU_Received);          // TCU

    Can2.enableMBInterrupts();
}

/**
 * @brief Sends the given message
 * @param Data a predefined strut called CAN_Data 
 * @return bool - This will return true if the message was sent 
 */
bool MyCanSend(CAN_Data Data)
{
  CAN_message_t msg;
  msg.id = Data.id;
  msg.len = 8;
  msg.buf[0]  = Data.x[0];
  msg.buf[1]  = Data.x[1];
  msg.buf[2]  = Data.x[2];
  msg.buf[3]  = Data.x[3];
  msg.buf[4]  = Data.x[4];
  msg.buf[5]  = Data.x[5];
  msg.buf[6]  = Data.x[6];
  msg.buf[7]  = Data.x[7];

  Can2.write(msg);
  return true;
}


/**
 * @brief This function is called when a MOTEC message is received
 */
void MOTEC_Received(const CAN_message_t &msg)
{
    switch(msg.id)
    {
    case MOTEC_id0:   MOTEC_Data_RX0.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX0.x[i] = msg.buf[i];}
                break;

    case MOTEC_id1:   MOTEC_Data_RX1.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX1.x[i] = msg.buf[i];}
                break;

    case MOTEC_id2:   MOTEC_Data_RX2.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX2.x[i] = msg.buf[i];}
                break;

    case MOTEC_id3:   MOTEC_Data_RX3.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX3.x[i] = msg.buf[i];}
                break;

    case MOTEC_id4:   MOTEC_Data_RX4.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX4.x[i] = msg.buf[i];}
                break;

    case MOTEC_id5:   MOTEC_Data_RX5.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX5.x[i] = msg.buf[i];}
                break;

    case MOTEC_id6:   MOTEC_Data_RX6.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX6.x[i] = msg.buf[i];}
                break;

    case MOTEC_id7:   MOTEC_Data_RX7.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX7.x[i] = msg.buf[i];}
                break;

    case MOTEC_id8:   MOTEC_Data_RX8.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX8.x[i] = msg.buf[i];}
                break;

    case MOTEC_id9:   MOTEC_Data_RX9.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RX9.x[i] = msg.buf[i];}
                break;

    case MOTEC_idA:   MOTEC_Data_RXA.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RXA.x[i] = msg.buf[i];}
                break;

    case MOTEC_idB:   MOTEC_Data_RXB.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RXB.x[i] = msg.buf[i];}
                break;

    case MOTEC_idC:   MOTEC_Data_RXC.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RXC.x[i] = msg.buf[i];}
                break;

    case MOTEC_idD:   MOTEC_Data_RXD.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RXD.x[i] = msg.buf[i];}
                break;

    case MOTEC_idE:   MOTEC_Data_RXE.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RXE.x[i] = msg.buf[i];}
                break;

    case MOTEC_idF:   MOTEC_Data_RXF.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {MOTEC_Data_RXF.x[i] = msg.buf[i];}
                break;
    }
}

/**
 * @brief This function is called when a DCM message is received
 */
void DCM_Received(const CAN_message_t &msg)
{
    switch(msg.id)
    {
    case DCM_id0:   DCM_Data_RX0.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX0.x[i] = msg.buf[i];}
                break;

    case DCM_id1:   DCM_Data_RX1.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX1.x[i] = msg.buf[i];}
                break;

    case DCM_id2:   DCM_Data_RX2.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX2.x[i] = msg.buf[i];}
                break;

    case DCM_id3:   DCM_Data_RX3.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX3.x[i] = msg.buf[i];}
                break;

    case DCM_id4:   DCM_Data_RX4.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX4.x[i] = msg.buf[i];}
                break;

    case DCM_id5:   DCM_Data_RX5.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX5.x[i] = msg.buf[i];}
                break;

    case DCM_id6:   DCM_Data_RX6.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX6.x[i] = msg.buf[i];}
                break;

    case DCM_id7:   DCM_Data_RX7.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX7.x[i] = msg.buf[i];}
                break;

    case DCM_id8:   DCM_Data_RX8.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX8.x[i] = msg.buf[i];}
                break;

    case DCM_id9:   DCM_Data_RX9.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RX9.x[i] = msg.buf[i];}
                break;

    case DCM_idA:   DCM_Data_RXA.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RXA.x[i] = msg.buf[i];}
                break;

    case DCM_idB:   DCM_Data_RXB.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RXB.x[i] = msg.buf[i];}
                break;

    case DCM_idC:   DCM_Data_RXC.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RXC.x[i] = msg.buf[i];}
                break;

    case DCM_idD:   DCM_Data_RXD.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RXD.x[i] = msg.buf[i];}
                break;

    case DCM_idE:   DCM_Data_RXE.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RXE.x[i] = msg.buf[i];}
                break;

    case DCM_idF:   DCM_Data_RXF.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {DCM_Data_RXF.x[i] = msg.buf[i];}
                break;
    }
}

/**
 * @brief This function is called when a PDM message is received
 */
void PDM_Received(const CAN_message_t &msg)
{
    switch(msg.id)
    {
    case PDM_id0:   PDM_Data_RX0.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX0.x[i] = msg.buf[i];}
                break;

    case PDM_id1:   PDM_Data_RX1.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX1.x[i] = msg.buf[i];}
                break;

    case PDM_id2:   PDM_Data_RX2.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX2.x[i] = msg.buf[i];}
                break;

    case PDM_id3:   PDM_Data_RX3.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX3.x[i] = msg.buf[i];}
                break;

    case PDM_id4:   PDM_Data_RX4.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX4.x[i] = msg.buf[i];}
                break;

    case PDM_id5:   PDM_Data_RX5.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX5.x[i] = msg.buf[i];}
                break;

    case PDM_id6:   PDM_Data_RX6.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX6.x[i] = msg.buf[i];}
                break;

    case PDM_id7:   PDM_Data_RX7.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX7.x[i] = msg.buf[i];}
                break;

    case PDM_id8:   PDM_Data_RX8.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX8.x[i] = msg.buf[i];}
                break;

    case PDM_id9:   PDM_Data_RX9.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RX9.x[i] = msg.buf[i];}
                break;

    case PDM_idA:   PDM_Data_RXA.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RXA.x[i] = msg.buf[i];}
                break;

    case PDM_idB:   PDM_Data_RXB.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RXB.x[i] = msg.buf[i];}
                break;

    case PDM_idC:   PDM_Data_RXC.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RXC.x[i] = msg.buf[i];}
                break;

    case PDM_idD:   PDM_Data_RXD.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RXD.x[i] = msg.buf[i];}
                break;

    case PDM_idE:   PDM_Data_RXE.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RXE.x[i] = msg.buf[i];}
                break;

    case PDM_idF:   PDM_Data_RXF.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {PDM_Data_RXF.x[i] = msg.buf[i];}
                break;
    }
}

/**
 * @brief This function is called when a TCU message is received
 */
void TCU_Received(const CAN_message_t &msg)
{
    switch(msg.id)
    {
    case TCU_id0:   TCU_Data_RX0.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX0.x[i] = msg.buf[i];}
                break;

    case TCU_id1:   TCU_Data_RX1.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX1.x[i] = msg.buf[i];}
                break;

    case TCU_id2:   TCU_Data_RX2.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX2.x[i] = msg.buf[i];}
                break;

    case TCU_id3:   TCU_Data_RX3.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX3.x[i] = msg.buf[i];}
                break;

    case TCU_id4:   TCU_Data_RX4.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX4.x[i] = msg.buf[i];}
                break;

    case TCU_id5:   TCU_Data_RX5.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX5.x[i] = msg.buf[i];}
                break;

    case TCU_id6:   TCU_Data_RX6.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX6.x[i] = msg.buf[i];}
                break;

    case TCU_id7:   TCU_Data_RX7.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX7.x[i] = msg.buf[i];}
                break;

    case TCU_id8:   TCU_Data_RX8.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX8.x[i] = msg.buf[i];}
                break;

    case TCU_id9:   TCU_Data_RX9.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RX9.x[i] = msg.buf[i];}
                break;

    case TCU_idA:   TCU_Data_RXA.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RXA.x[i] = msg.buf[i];}
                break;

    case TCU_idB:   TCU_Data_RXB.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RXB.x[i] = msg.buf[i];}
                break;

    case TCU_idC:   TCU_Data_RXC.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RXC.x[i] = msg.buf[i];}
                break;

    case TCU_idD:   TCU_Data_RXD.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RXD.x[i] = msg.buf[i];}
                break;

    case TCU_idE:   TCU_Data_RXE.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RXE.x[i] = msg.buf[i];}
                break;

    case TCU_idF:   TCU_Data_RXF.id = msg.id;
                for (size_t i = 0; i <= 7; i++)
                {TCU_Data_RXF.x[i] = msg.buf[i];}
                break;
    }
}
