#ifndef __SRC_EH_ANALOG_H__
#define __SRC_EH_ANALOG_H__

#include <Adafruit_ADS1X15.h>

#include "sensesp/sensors/sensor.h"

using namespace sensesp;


FloatProducer* ConnectTempSender(Adafruit_ADS1115* ads1115, int channel, String name);

FloatProducer* ConnectPressSender(Adafruit_ADS1115* ads1115, int channel, String name);

#endif
