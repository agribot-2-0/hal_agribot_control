/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/TTCommHB.h>

// mask used for extracting address and source, without PG
#define ADDR_MASK 0xFFFF

#define HANDSHAKE_DATA 0x10
#define PROTOCOL_RESPONSE_FRAME_ID 0x00EF64C8
#define PROTOCOL_FRAME_ID 0x000EFC864
#define TTC_HB_ID 0x00FFDFC8
#define TTC_RESPONSE_HB_ID 0x00FFDF64

TTCommHB::TTCommHB(const char* pchCanChannel)
{
  _CanHandler.iInitialize(pchCanChannel);
  _u8HbVal = 0;
}

void TTCommHB::vSendHanshake()
{
  can_frame message;
  message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
  message.can_dlc = 8;
  message.data[0] = HANDSHAKE_DATA;
  message.data[1] = 0;
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  _CanHandler.iSendFrame(&message);
}

bool TTCommHB::bIsHandshakeReceived()
{
  can_frame message;
  while (_CanHandler.bReceiveFrame(&message))
  {
    // if protocol frame availible
    if ((message.can_id & ADDR_MASK) == (PROTOCOL_FRAME_ID & ADDR_MASK))
    {
      // if it is handshake confirmation
      if (message.data[0] == HANDSHAKE_DATA)
      {
        return true;
      }
    }
  }
  return false;
}

void TTCommHB::vSendHB()
{
  can_frame message;
  message.can_id = TTC_HB_ID;
  message.can_dlc = 8;
  message.data[0] = (this->_u8HbVal);
  message.data[1] = 0;
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  _CanHandler.iSendFrame(&message);
}

int TTCommHB::iHBReceived()
{
  can_frame message;
  while (_CanHandler.bReceiveFrame(&message))
  {
    // if hb frame received
    if ((message.can_id & ADDR_MASK) == (TTC_RESPONSE_HB_ID & ADDR_MASK))
    {
      // if it is expected hb value
      if (_u8HbVal + 1 == message.data[0])
      {
        _u8HbVal += 2;
        return CORRECT_HB_RECEIVED;
      }
      // if it is wrong hb value
      else
      {
        return WRONG_HB_RECEIVED;
      }
    }
  }
  return -1;
}

void TTCommHB::vResetCnt()
{
  _u8HbVal = 0;
}
