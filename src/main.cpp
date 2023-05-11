#include <Arduino.h>

#define batteryVoltagePin 33
#define currentControlPin 25

float batteryVoltage = 0; // Voltage in volts
float drainCurrent = 0; // Current in amps
float shuntResistor = 0.47; // Resistance of shunt resistor in Ohms

uint16_t batteryVoltageRaw = 0;
uint16_t drainCurrentRaw = 0;

float upperVoltage = 4.2;
float lowerVoltage = 3.4;

hw_timer_t * timer = NULL;

float getBatteryVoltage();
uint16_t batteryCapacityTest();

void setup() {
  // put your setup code here, to run once:
    pinMode(batteryVoltagePin, INPUT);
    pinMode(currentControlPin, OUTPUT);

    analogWrite(currentControlPin, 0);

    Serial.begin(115200);

    timer = timerBegin(0, 8000, true);
}

void loop() {
    if(Serial.available())
    {
        char command = Serial.read();

        switch (command)
        {

            case 'a':
            {
                Serial.println("New drain current value in amps: ");

                while(!Serial.available())
                {
                    // wait until serial is available
                }

                float drainCurrent = Serial.parseFloat();
                int drainCurrentRaw = ((drainCurrent * shuntResistor) * 255);

                analogWrite(currentControlPin, drainCurrentRaw);

                Serial.print("Drain current is: ");
                Serial.print(drainCurrent);
                Serial.println("A.");
                
                break;
            }

            case 'b':
            {
                // set test starting voltage
                Serial.println("New test start value in volts: ");

                while(!Serial.available())
                {
                    // wait until serial is available
                }

                upperVoltage = Serial.parseFloat();
                

                Serial.print("Test start voltage is: ");
                Serial.print(upperVoltage);
                Serial.println("V.");
                
                break;
            }

            case 'c':
            {
                // set test end voltage
                Serial.println("New test end value in volts: ");

                while(!Serial.available())
                {
                    // wait until serial is available
                }

                lowerVoltage = Serial.parseFloat();
                

                Serial.print("Test end voltage is: ");
                Serial.print(lowerVoltage);
                Serial.println("V.");
                
                break;
            }

            case 'd':
            {
                // start battery capacity test
                batteryCapacityTest();
                break;
            }

            case 'e':
            {
                // voltage drop test
                break;
            }

            case 'f':
            {
                // display current battery voltage
                Serial.println(getBatteryVoltage());
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

float getBatteryVoltage()
{
    uint16_t batteryVoltageRaw = analogRead(batteryVoltagePin);
    batteryVoltage = ((batteryVoltageRaw * 4.2) / 4096);
    return batteryVoltage;
}

uint16_t batteryCapacityTest()
{
    uint16_t batteryCapacity = 0;

    if(getBatteryVoltage() > upperVoltage)
    {
        analogWrite(currentControlPin, drainCurrentRaw);

        while(getBatteryVoltage() > upperVoltage)
        {
            delay(1000); // do nothing but waste less cycles checking the voltage level
        }
        
        uint16_t counter = 0; // number of seconds that the voltage is above the cutoff voltage
        while(getBatteryVoltage() > lowerVoltage) // would be better to implement this using a timer
        {
            counter++; // increment counter per cycle
            Serial.println(getBatteryVoltage());
            delay(1000);
        }

        uint16_t batteryCapacity = (counter/3600) * drainCurrent;

    }

    return batteryCapacity;
}