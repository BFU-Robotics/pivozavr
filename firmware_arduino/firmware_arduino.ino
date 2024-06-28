#include "settings.hpp"
#include "bridges.hpp"

#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

// Настройка мостов
HardwareSerial& mainSerial = Serial;
HardwareSerial& frontSerial = Serial3;
HardwareSerial& midSerial = Serial2;
HardwareSerial& rearSerial = Serial1;
pv::Bridge bridgeFront(pv::Settings::relayFrontPin, frontSerial);
pv::Bridge bridgeMid(pv::Settings::relayMidPin, midSerial);
pv::Bridge bridgeRear(pv::Settings::relayRearPin, rearSerial);

modbusDevice regBank;
modbusSlave slave;

int powerStatus = 0; 

void setup() 
{
  //mainSerial.begin(pv::Settings::serialBaud);

  regBank.setId(1);
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  regBank.add(30007);
  regBank.add(30008);
  regBank.add(40001);
  regBank.add(40002);
  regBank.add(40003);
  regBank.add(40004); 
  regBank.add(40005);
  regBank.add(40006);
  regBank.add(40007);
  slave._device = &regBank;  
  slave.setBaud(pv::Settings::serialBaud); 

  regBank.set(40001, 0);
  regBank.set(40002, 0);
  regBank.set(40003, 0);
  regBank.set(40004, 0);
  regBank.set(40005, 0);
  regBank.set(40006, 0);
  regBank.set(40007, 0);
}

unsigned long iTimeSend = 0;

void loop(void)
{ 
  if(powerStatus != regBank.get(40007))
  {
    if(regBank.get(40007) == 1)
    {
        bridgeFront.On();
        bridgeMid.On();
        bridgeRear.On();
        delay(1000);
        powerStatus = 1;
    }
    else
    {
        bridgeFront.Off();
        bridgeMid.Off();
        bridgeRear.Off();
        delay(1000);
        powerStatus = 0;
    }
  }

  if(powerStatus == 1)
  {

    unsigned long timeNow = millis();

    // Check for new received data
    bridgeFront.Receive();
    bridgeMid.Receive();
    bridgeRear.Receive();

    // Send commands
    if (iTimeSend > timeNow) return;
    iTimeSend = timeNow + pv::Settings::timeSend;
    
    bridgeFront.Send(-regBank.get(40001), -regBank.get(40002));
    bridgeMid.Send(-regBank.get(40003), -regBank.get(40004));
    bridgeRear.Send(-regBank.get(40005), -regBank.get(40006));

    pv::SerialFeedback feedbackFront = bridgeFront.getFeedback();
    regBank.set(30001, -(word) feedbackFront.speedL_meas );
    regBank.set(30002, (word) feedbackFront.speedR_meas );
    pv::SerialFeedback feedbackMid = bridgeMid.getFeedback();
    regBank.set(30003, -(word) feedbackMid.speedL_meas );
    regBank.set(30004, (word) feedbackMid.speedR_meas );
    pv::SerialFeedback feedbackRear = bridgeRear.getFeedback();
    regBank.set(30005, -(word) feedbackRear.speedL_meas );
    regBank.set(30006, (word) feedbackRear.speedR_meas );
  }

  slave.run(); 
}
