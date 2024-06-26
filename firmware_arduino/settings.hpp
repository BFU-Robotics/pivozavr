#pragma once

namespace pv
{
  struct Settings
  {
    static int relayFrontPin;
    static int relayMidPin;
    static int relayRearPin;
    static int onDelayMs;
    static int offDelayMs;
    static uint16_t startFrame;
    static uint32_t hoverSerialBaud;
    static uint32_t serialBaud;
    static uint32_t timeSend;
  };

  int Settings::relayFrontPin = 49;
  int Settings::relayMidPin = 51;
  int Settings::relayRearPin = 53;
  int Settings::onDelayMs = 50;
  int Settings::offDelayMs = 100;
  uint16_t Settings::startFrame = 0xABCD;
  uint32_t Settings::hoverSerialBaud = 115200;
  uint32_t Settings::serialBaud = 115200;
  uint32_t Settings::timeSend = 100;
}