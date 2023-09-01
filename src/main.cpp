#include <Arduino.h>

#include <Adafruit_ADS1X15.h>
#include <jled.h>

// System constant
#define RELEASE "1.0.0"

// Pin
#define READY_PIN   3
#define MOFSET_PIN  19
#define LED_PIN     18

// ADS Settings
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

// Mofset settings
bool  mofset_on = false;

// Voltage settings
constexpr int VD_R1 = 100000;
constexpr int VD_R2 = 10000;
float power_voltage = 36.0;
float divisor_voltage = 0;
float min_voltage = 0;
float max_voltage = 0;

// Protection
constexpr float start_resistor = 36.2;
constexpr float stop_resistor = 36.1;

// Led
constexpr int period_signal = 2000;
jled::Esp32Hal esp32_led(LED_PIN);
auto status_led = JLed(esp32_led);

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

void setup(void)
{
  Serial.begin(115200);
  log_i("Vesc OPV v%s", RELEASE);

  pinMode(MOFSET_PIN, OUTPUT);
  digitalWrite(MOFSET_PIN, LOW);
  log_i("MOFSET init and turn off");

  log_i("Getting differential reading from AIN0 (P) and AIN1 (N)");
  log_i("ADC Range (GAIN=1) +/- 4.096V, 1 bit = 0.125mV");

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  // if ads not start blink led one time
  if (!ads.begin()) {
    log_e("Failed to initialize ADS.");
    status_led.Blink(250, 250).DelayAfter(2000).Forever();
    while (1) {status_led.Update();}
  } else {
    log_d("ADS successfull init");
  }

  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);
  log_d("ADS interrupt attached");

  // Start continuous conversions.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  log_i("ADS start");

  if (divisor_voltage == 0) {
    divisor_voltage = power_voltage * VD_R2 / (VD_R1 + VD_R2);
    log_i("Offset not settings, used theorical value %.3f", divisor_voltage);
  }

  min_voltage = 0.8 * power_voltage;
  max_voltage = 0.95 * 4.096 * (VD_R1 + VD_R2) / VD_R2;
  log_i("voltage security min/max : %.3fv-%.3fv", min_voltage, max_voltage);

  // blink 5 times to said all is OK
  status_led.Blink(250, 250).Repeat(5);
  while (status_led.IsRunning()) {status_led.Update();}
}

void manage_mofset(const float voltage) {
  if (!mofset_on && (voltage >= start_resistor)) {
    digitalWrite(MOFSET_PIN, HIGH);
    mofset_on = true;
    status_led.On();
    log_d("Mofset on %.3f/%.3f", voltage, start_resistor);
  }

  if (mofset_on && voltage <= stop_resistor) {
    digitalWrite(MOFSET_PIN, LOW);
    mofset_on = false;
    status_led.Off();
    log_d("Mofset of %.3f/%.3f", voltage, stop_resistor);
  }
}

float check_voltage() {
    const int16_t ads_raw = ads.getLastConversionResults();
    const float   ads_voltage = ads.computeVolts(ads_raw);
    float voltage = power_voltage * (ads_voltage / divisor_voltage);

    // check over voltage
    if (voltage > max_voltage) {
      log_e("over voltage on adc %.3f/3.9v, STOP !", ads_voltage);
      status_led.Blink(250,750).Repeat(2).DelayAfter(2000).Forever();
      //digitalWrite(MOFSET_PIN, LOW);
      while (status_led.IsRunning()) {status_led.Update();}
    }

    // under voltage issue
    if ( voltage < min_voltage ) {
      log_e("under voltage %.3f/%.3f, STOP !", voltage, min_voltage);
      status_led.Blink(250,750).Repeat(3).DelayAfter(2000).Forever();
      //digitalWrite(MOFSET_PIN, LOW);
      while (status_led.IsRunning()) {status_led.Update();}
    }

    log_d("voltage checked %.3f", voltage);
    return voltage;
}

void loop(void) {

  // If we have new data, read it and update the mofset status
  if (new_data) {

    float voltage = check_voltage();
    manage_mofset(voltage);

    new_data = false;
    
  }

  status_led.Update(); // update in continue the status led to apply effect.
}