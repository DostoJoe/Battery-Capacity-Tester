#include <Arduino.h>

#define batteryVoltagePin 33
#define currentControlPin 25
#define currentMeasurePin 32

float batteryVoltage = 0; // Voltage in volts
float drainCurrent = 0; // Current in amps
float shuntResistor = 0.47f; // Resistance of shunt resistor in Ohms

uint16_t batteryVoltageRaw = 0;
uint8_t drainCurrentRaw = 0;
uint16_t currentMeasureRaw = 0;

float upperVoltage = 4.2f;
float lowerVoltage = 3.4f;

int16_t getShuntDrop();
void setCurrent();
float getBatteryVoltage();
uint16_t batteryCapacityTest();

void setup() {
  // put your setup code here, to run once:
    pinMode(batteryVoltagePin, INPUT);
    pinMode(currentControlPin, OUTPUT);
    pinMode(currentMeasurePin, INPUT);

    analogWrite(currentControlPin, 0);

    Serial.begin(115200);
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

                drainCurrent = Serial.parseFloat();

                if((drainCurrent >= 0.20f && drainCurrent <= 1.50f))
                {
                    drainCurrentRaw = map(drainCurrent * 100, 20, 150, 50, 130); 
                }

                else if(drainCurrent == 0)
                {
                    drainCurrentRaw = drainCurrent;
                }

                else
                {
                    Serial.println("Out of range - please try again between 0.2A and 1.5A.\nTo turn off please enter 'a0'.");
                    break;
                }

                // analogWrite(currentControlPin, drainCurrentRaw);

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
                int16_t batteryCapacity = batteryCapacityTest();

                Serial.print("Capacity discharged: ");
                Serial.print(batteryCapacity);
                Serial.println("mAh");

                // int16_t batteryCapacityAdj = batteryCapacity * (0.8f / (upperVoltage - lowerVoltage));

                // Serial.print("Total Battery Capacity (estimated): "); // capacity between 4.2V and 3.4V
                // Serial.print(batteryCapacityAdj);
                // Serial.println("mAh");
                break;
            }

            case 'e':
            {
                // voltage drop test
                setCurrent();
                break;
            }

            case 'f':
            {
                // display current battery voltage
                Serial.println(getBatteryVoltage());
                batteryVoltageRaw = analogRead(batteryVoltagePin);
                Serial.println(batteryVoltageRaw);
                break;
            }

            case 'g':
            {
                int16_t shuntDropRaw = getShuntDrop();
                float realCurrent = (shuntDropRaw*3.30f/4095);
                if(shuntDropRaw > 1)
                {
                    realCurrent += 0.12f;
                }

                Serial.println(shuntDropRaw);
                Serial.println(realCurrent);
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

int16_t getShuntDrop()
{
    uint32_t currentMeasureRunning;
    uint16_t currentMeasure;
    for (int n = 0; n < 100; n++)
    {
        uint16_t buffer = analogRead(currentMeasurePin);
        if(buffer > 10)
        {
            currentMeasureRunning += buffer;
        }
    }
    currentMeasure = currentMeasureRunning/100;
    return currentMeasure;
}

void setCurrent()
{
    uint8_t margin = 30;

    uint16_t targetDrainRaw = (drainCurrent * shuntResistor * 4095) / 3.3f;
    // read current voltage across shunt
    analogWrite(currentControlPin, drainCurrentRaw);
    delay(100);

    currentMeasureRaw = getShuntDrop();

    // compare voltage across shunt to expected voltage for given drain current
    while(currentMeasureRaw > targetDrainRaw + margin)
    {
        drainCurrentRaw -= 10;
        analogWrite(currentControlPin, drainCurrentRaw);
        delay(1000);

        Serial.println(drainCurrentRaw);
    }

    while(currentMeasureRaw < targetDrainRaw - margin)
    {
        drainCurrentRaw += 10;
        analogWrite(currentControlPin, drainCurrentRaw);
        delay(1000);

        Serial.println(drainCurrentRaw);
    }

    Serial.print("Drain current raw is ");
    Serial.print(drainCurrentRaw);
    Serial.print(" with ");
    Serial.println(currentMeasureRaw);
}

float getBatteryVoltage()
{
    batteryVoltageRaw = analogRead(batteryVoltagePin);
    batteryVoltage = ((batteryVoltageRaw * 4.2f) / 4095);
    float batteryVoltageMapped = map(batteryVoltage*100, 308, 419, 340, 420);
    return (batteryVoltageMapped / 100);
}

uint16_t batteryCapacityTest()
{
    float batteryCapacity;
    uint16_t counter = 0;

    if(getBatteryVoltage() >= upperVoltage - 0.1f)
    {
        float batteryVoltageBuffer; // battery voltage buffer
        float avgBatteryVoltage;
        float prevBatteryVoltage;

        analogWrite(currentControlPin, drainCurrentRaw);
        
        if(getBatteryVoltage() > upperVoltage)
        {
            Serial.println("Discharging battery to target start voltage - please wait...");

            while(getBatteryVoltage() > upperVoltage)
            {
                Serial.println(getBatteryVoltage());
                delay(1000); // do nothing but waste less cycles checking the voltage level
            }
        }

        Serial.println("Battery capacity test starting - this may take a while...");
        prevBatteryVoltage = getBatteryVoltage();
        
        while(prevBatteryVoltage >= lowerVoltage)
        {
            counter++;
            avgBatteryVoltage += getBatteryVoltage();

            if(counter % 10 == 0)
            {
                batteryVoltageBuffer = avgBatteryVoltage / 10;
                avgBatteryVoltage = 0;

                if(batteryVoltageBuffer < prevBatteryVoltage - 0.01f)
                {
                    prevBatteryVoltage = batteryVoltageBuffer;
                    Serial.println(batteryVoltageBuffer);
                    Serial.println(counter);
                }                
            }

            delay(1000);
        }

        batteryCapacity = (counter * drainCurrent / 3.6f); // counter from s to h; drain current from A to mA
        
        Serial.println("\nBattery capacity test finished");
    }

    else
    {
        Serial.println("charge battery to target voltage");
    }
    analogWrite(currentControlPin, 0); // set current drain to 0

    return batteryCapacity;
}