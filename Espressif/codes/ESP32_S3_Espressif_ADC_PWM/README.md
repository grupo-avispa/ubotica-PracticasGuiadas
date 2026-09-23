| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# _ESP32-S3-Espressif-ADC-PWM_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

ESPIDF project for ESP32 S3 that demonstrates the use of ADC and PWM peripherals, including calibration and reading analog values. 
  
For ADC reference: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html
For PWM reference: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/ledc.html
  
The example uses ADC in one-shot mode (https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_oneshot.html), that should be enough for us. If you want to use ADC in continuous mode to be faster, please check: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc/adc_continuous.html
  
For this example you will need a protoboard, one (or better two) potenciometers, a resistor (220 ohms would work), a LED and some jumper wires.

The example configure two ADC input channels for ADC1, named CHAN_A and CHAN_B. The particular ADC1 channel associated to each of these names is defined in the platformio.ini file, along with the desired attenuation value. Then, the values of these two ADC are read and printed in the console. 
 
Finally, the value of ADC CHAN_A is used to set the duty cycle of a PWM output so a LED connected to this PWM output pin will light up with a brightness proportional to the ADC CHAN_A value.
 

