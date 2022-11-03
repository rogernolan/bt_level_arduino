

#include <MKRIMU.h>
#include <ArduinoBLE.h>

#include "bt_constants.h"

BLEService levelService(LEVEL_SERVICE_UUID); // creating the service
BLEFloatCharacteristic headingCharacteristic(HEADING_CHARACTERISTIC_UUID, BLERead | BLENotify); // characteristic for heading
BLEFloatCharacteristic rollCharacteristic(ROLL_CHARACTERISTIC_UUID, BLERead | BLENotify); // characteristic for Roll
BLEFloatCharacteristic pitchCharacteristic(PITCH_CHARACTERISTIC_UUID, BLERead | BLENotify); // characteristic for pitch

BLEBoolCharacteristic ledSwitcher("2A57", BLERead | BLEWrite); // creating the LED setting characteristic
BLEBoolCharacteristic setZero(SET_ZERO_SERVICE_UUID, BLERead | BLEWrite); // setting this true causes the level to zero.

// global to see if we have the IMU Shield installed.
boolean haveIMU = 0;

// Analog pin the LED is connected to.
#define LED_PIN 2

// epsilon for comparisons between angles to limit amount of BT updates.
#define LEVEL_COMPARISON_EPSILON 0.001
bool significantChange(float from, float to) {
    return (fabs(from - to) > LEVEL_COMPARISON_EPSILON);
}
void setup() {

  Serial.begin(9600);
  while (!Serial);  // loop on serial waiting for it to connect.

  // setup the LED pin
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built in LED
  pinMode(LED_PIN, OUTPUT); // initialize the LED on the bradboard to show the BT written char

  // initialise the IMU
  Serial.println("Initialise IMU!");

  if (IMU.begin()) {
    // initial debug dump of setting etc.
    Serial.print("Euler Angles sample rate = ");
    Serial.print(IMU.eulerAnglesSampleRate());
    Serial.println(" Hz");

    Serial.println();
    Serial.println("Euler Angles in degrees");
    Serial.println("Heading\tRoll\tPitch");

    haveIMU = true;
  } else {
    Serial.println("Failed to initialise IMU!");
  } 

	Serial.println("Starting Bluetooth® Low Energy service!");

  // initialise the BTLE Radio
	  if (BLE.begin()) {
      BLE.setDeviceName(DEVICE_NAME);
      BLE.setLocalName(LOCAL_NAME); 

      BLE.setAdvertisedService(levelService);

      BLEDescriptor headingLabelDescriptor("2901", "heading");    // descriptors are random data attached to all BLE objects. 0x2901 is "User description"
      headingCharacteristic.addDescriptor(headingLabelDescriptor);      
      levelService.addCharacteristic(headingCharacteristic);

      BLEDescriptor rollLabelDescriptor("2901", "roll");
      rollCharacteristic.addDescriptor(rollLabelDescriptor);       
      levelService.addCharacteristic(rollCharacteristic);

      BLEDescriptor pitchLabelDescriptor("2901", "pitch");
      pitchCharacteristic.addDescriptor(pitchLabelDescriptor);       
      levelService.addCharacteristic(pitchCharacteristic);

      BLEDescriptor zeroLabelDescriptor("2901", "Zero readings");
      setZero.addDescriptor(zeroLabelDescriptor); 
      levelService.addCharacteristic(setZero);
      
      levelService.addCharacteristic(ledSwitcher);

      BLE.addService(levelService);  // adding the service. Seems like the ordering of these is important.
	
      //set initial value for characteristics
      ledSwitcher.writeValue(0); 
      headingCharacteristic.writeValue(0.0);
      rollCharacteristic.writeValue(0.0);
      pitchCharacteristic.writeValue(0.0);
      setZero.writeValue(0);

      BLE.advertise();
      Serial.println(" Bluetooth® device active, waiting for connections...");

    } else {

	    Serial.println("starting Bluetooth failed!");
	    while (1);
	  }

    // switch off both LEDs
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PIN, LOW);

}

void loop() {

  BLEDevice central = BLE.central(); // wait for a Bluetooth® Low Energy connection - central is a client
  if (central) {  // if a central is connected to the peripheral
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    digitalWrite(LED_BUILTIN, HIGH);  // Turn on the connection LED
    
    long previousMillis = 0;

    float lastRoll = 0.0;
    float lastPitch = 0.0;
    float lastHeading = 0.0;

    while (central.connected()) { // while a Bluetooth central is connected:

      long currentMillis = millis();


      if (currentMillis - previousMillis >= 200) {    // read every 200 millis and update the BT characteristics

        previousMillis = currentMillis;

        if(haveIMU){ 
          if (IMU.eulerAnglesAvailable()) {
            float heading, roll, pitch;

            IMU.readEulerAngles(heading, roll, pitch);

            // Serial.print(heading);
            // Serial.print('\t');
            // Serial.print(roll);
            // Serial.print('\t');
            // Serial.println(pitch);

            if (significantChange(heading, lastHeading)) {
              lastHeading = heading;
              headingCharacteristic.writeValue(double(heading));
            }

            if (significantChange(roll, lastRoll)) {

              lastRoll = roll;
              rollCharacteristic.writeValue(double(roll));
            }

            if( significantChange(pitch, lastPitch)) {
              lastPitch = pitch;
              pitchCharacteristic.writeValue(double(pitch));
            }
          }
        }
	
        if (ledSwitcher.written()) {
          if (ledSwitcher.value()) {   // any value other than 0
            digitalWrite(LED_PIN, HIGH); // Turn the LED on
          } else {                              // a 0 value
            digitalWrite(LED_PIN, LOW);  // Turn the LED off
          }
        }

      }
    }
    
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());

    digitalWrite(LED_BUILTIN, LOW); // Turn off the connection LED
  }
}




