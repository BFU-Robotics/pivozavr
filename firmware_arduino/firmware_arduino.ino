#include "settings.hpp"
#include "bridges.hpp"

// Настройка мостов
HardwareSerial& mainSerial = Serial;
HardwareSerial& frontSerial = Serial3;
HardwareSerial& midSerial = Serial2;
HardwareSerial& rearSerial = Serial1;
pv::Bridge bridgeFront(pv::Settings::relayFrontPin, frontSerial);
pv::Bridge bridgeMid(pv::Settings::relayMidPin, midSerial);
pv::Bridge bridgeRear(pv::Settings::relayRearPin, rearSerial);

void setup() 
{
  bridgeFront.On();
  bridgeMid.On();
  bridgeRear.On();

  mainSerial.begin(pv::Settings::serialBaud);
}

unsigned long iTimeSend = 0;

void loop(void)
{ 
  unsigned long timeNow = millis();

  // Check for new received data
  bridgeFront.Receive();
  bridgeMid.Receive();
  bridgeRear.Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + pv::Settings::timeSend;
  
  bridgeFront.Send(0, 0);
  bridgeMid.Send(-0, -0);
  bridgeRear.Send(0, -0);
}
