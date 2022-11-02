#include "eh_guages.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include <iostream>

// ADS1115 input hardware scale factor (input voltage vs voltage at ADS1115)
const float kAnalogInputScale = 29. / 2.048;

// Engine Hat constant measurement current (A)
const float kMeasurementCurrent = 0.01;

float scale = 1.0;

FloatProducer* ConnectTempSender(Adafruit_ADS1115* ads1115, int channel,
                                 String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];
  char sk_path[80];
  char meta_display_name[80];
  char meta_description[80];

  MovingAverage* avg = new MovingAverage(5, scale); 

  snprintf(config_path, sizeof(config_path), "/Temp %s/Voltage",
           name.c_str());
  auto guage_voltage =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        // return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
        return kAnalogInputScale * adc_output_volts;
      });

  snprintf(config_path, sizeof(config_path), "/Temp %s/Voltage SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.senderVoltage",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Voltage %s",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description),
           "Measured temp %s sender voltage", name.c_str());
  auto sender_voltage_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("V", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path), "/Guage %s/Temp Curve",
           name.c_str());
  auto guage_read = (new CurveInterpolator(nullptr, config_path))
                        ->set_input_title("Guage Volts")
                        ->set_output_title("Guage Reading - K");

  if (guage_read->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    guage_read->clear_samples();
    guage_read->add_sample(CurveInterpolator::Sample(0, 0));
    guage_read->add_sample(CurveInterpolator::Sample(180., 1));
    guage_read->add_sample(CurveInterpolator::Sample(300., 2));
  }

  snprintf(config_path, sizeof(config_path), "/Temp %s/SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.temperature",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Temp %s",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description), "Temp %s K",
           name.c_str());
  auto temperature_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("K", meta_display_name, meta_description));

  guage_voltage->connect_to(sender_voltage_sk_output);
  guage_voltage->connect_to(avg)
      ->connect_to(guage_read)
      ->connect_to(temperature_sk_output);
 
 return guage_read;
}

FloatProducer* ConnectPressSender(Adafruit_ADS1115* ads1115, int channel,
                                 String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];
  char sk_path[80];
  char meta_display_name[80];
  char meta_description[80];

  MovingAverage* avg = new MovingAverage(5, scale); 

  snprintf(config_path, sizeof(config_path), "/OP %s/Voltage",
           name.c_str());
  auto guage_voltage =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        // return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
        return kAnalogInputScale * adc_output_volts;
      });

  snprintf(config_path, sizeof(config_path), "/OP %s/Voltage SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.oilPressure.senderVoltage",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Voltage %s",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description),
           "Measured OP %s sender voltage", name.c_str());
  auto sender_voltage_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("V", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path), "/Guage %s/Pressure Curve",
           name.c_str());
  auto guage_read = (new CurveInterpolator(nullptr, config_path))
                        ->set_input_title("Guage volts")
                        ->set_output_title("Guage Reading - Pa");

  if (guage_read->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    guage_read->clear_samples();
    guage_read->add_sample(CurveInterpolator::Sample(0, 0));
    guage_read->add_sample(CurveInterpolator::Sample(180., 1));
    guage_read->add_sample(CurveInterpolator::Sample(300., 1));
  }

  snprintf(config_path, sizeof(config_path), "/OP %s/SK Path",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.oilPressure",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "OP %s psi",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description), "OP %s psi",
           name.c_str());
  auto oilpressure_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("Pa", meta_display_name, meta_description));

  guage_voltage->connect_to(sender_voltage_sk_output);
  guage_voltage->connect_to(avg)
      ->connect_to(guage_read)
      ->connect_to(oilpressure_sk_output);
     
 return guage_read;
}