#pragma once

namespace pv
{

  struct SerialCommand 
  {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
  };

  struct SerialFeedback
  {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
  };

}