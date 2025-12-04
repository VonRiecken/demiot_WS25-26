const int LDR_PIN = 34;            // ADC pin
const int R_FIXED = 9900;         //  fixed resistor in divider
const int SAMPLES = 20;            // oversampling

void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(LDR_PIN, ADC_11db); // increase measurable range
  delay(1000);
}

int readAdcAverage(int pin, int samples=20) {
  long sum = 0;
  for (int i=0; i<samples; ++i) {
    sum += analogRead(pin);
    delay(5); // short delay between samples
  }
  return (int)(sum / samples);
}

void loop() {
  int adc = readAdcAverage(LDR_PIN, SAMPLES);
  float voltage = adc * (3.3 / 4095.0); // approximate
  Serial.printf("ADC=%d, V=%.3f\n", adc, voltage);
  delay(2000);
}