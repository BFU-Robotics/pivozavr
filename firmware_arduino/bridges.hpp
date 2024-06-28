#pragma once
#include "data_formats.hpp"

namespace pv
{

  class Bridge
  {
    int m_relayPin;
    HardwareSerial& m_serial;
    uint8_t m_idx = 0;                        // Index for new data pointer
    uint16_t m_bufStartFrame;                 // Buffer Start Frame
    byte *m_p;                                // Pointer declaration for the new received data
    byte m_incomingByte;
    byte m_incomingBytePrev;
    SerialFeedback m_newFeedback;
    SerialFeedback m_feedback;

    void ToggleRelayPin(int dt)
    {
      digitalWrite(m_relayPin, LOW);
      delay(dt);
      digitalWrite(m_relayPin, HIGH);
    }

    public:
      Bridge(int relayPin, HardwareSerial& serial) : m_relayPin(relayPin), m_serial(serial)  
      {
        pinMode(m_relayPin, OUTPUT);
        digitalWrite(m_relayPin, HIGH);

        m_serial.begin(Settings::hoverSerialBaud);
      }

      On()
      {
        ToggleRelayPin(Settings::onDelayMs);
      }

      Off()
      {
        ToggleRelayPin(Settings::offDelayMs);
      }

      SerialFeedback getFeedback()
      {
        return m_feedback;
      }

      void Send(int16_t uSteer, int16_t uSpeed)
      {
        SerialCommand cmd;
        // Create command
        cmd.start    = (uint16_t)Settings::startFrame;
        cmd.steer    = (int16_t)uSteer;
        cmd.speed    = (int16_t)uSpeed;
        cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);

        // Write to Serial
        m_serial.write((uint8_t *) &cmd, sizeof(cmd)); 
      }

      void Receive()
      {
          // Check for new data availability in the Serial buffer
          if (m_serial.available()) 
          {
              m_incomingByte = m_serial.read();                                   // Read the incoming byte
              m_bufStartFrame	= ((uint16_t)(m_incomingByte) << 8) | m_incomingBytePrev;       // Construct the start frame
          }
          else 
          {
              return;
          }

          // Copy received data
          if (m_bufStartFrame == Settings::startFrame) 
          {	                    // Initialize if new data is detected
              m_p       = reinterpret_cast<byte*>(&m_newFeedback);
              *m_p++    = m_incomingBytePrev;
              *m_p++    = m_incomingByte;
              m_idx     = 2;	
          } else if (m_idx >= 2 && m_idx < sizeof(SerialFeedback)) {  // Save the new received data
              *m_p++    = m_incomingByte; 
              m_idx++;
          }	
          
          // Check if we reached the end of the package
          if (m_idx == sizeof(SerialFeedback)) {
              uint16_t checksum;
              checksum = (uint16_t)(m_newFeedback.start ^ m_newFeedback.cmd1 ^ m_newFeedback.cmd2 ^ m_newFeedback.speedR_meas ^ m_newFeedback.speedL_meas
                                  ^ m_newFeedback.batVoltage ^ m_newFeedback.boardTemp ^ m_newFeedback.cmdLed);

              // Check validity of the new data
              if (m_newFeedback.start == Settings::startFrame && checksum == m_newFeedback.checksum) {
                  // Copy the new data
                  memcpy(&m_feedback, &m_newFeedback, sizeof(SerialFeedback));

                  // Print data to built-in Serial
                  //Serial.print("1: ");   Serial.print(feedback.cmd1);
                  //Serial.print(" 2: ");  Serial.print(feedback.cmd2);
                  //Serial.print(" 3: ");  Serial.print(feedback.speedR_meas);
                  //Serial.print(" 4: ");  Serial.print(feedback.speedL_meas);
                  //Serial.print(" 5: ");  Serial.print(feedback.batVoltage);
                  //Serial.print(" 6: ");  Serial.print(feedback.boardTemp);
                  //Serial.print(" 7: ");  Serial.println(feedback.cmdLed);
              } else {
                //Serial.println("Non-valid data skipped");
              }
              m_idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
          }

          // Update previous states
          m_incomingBytePrev = m_incomingByte;
      }
  };

}