#include <XBee.h>
#include <Wire.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include "Protocentral_MAX30205.h"
MAX30205 tempSensor;

// cite https://forum.arduino.cc/t/using-millis-for-timing-a-beginners-guide/483573
const unsigned long period = 20000; // 20 second intervals
unsigned long startTime;
unsigned long currTime;

int hRateMeas;
int bloodOxMeas;
float bodyTempMeas;

uint8_t hRateLower = 40; // will likely need defaults
uint8_t hRateUpper = 120;
uint8_t bloodOxLower = 80;
uint8_t bloodOxUpper = 100;
float bodyTempLower = 25; // in Celsius, must be changed
float bodyTempUpper = 35;

int resPin = 4; // CHANGE THESE
int mfioPin = 35;

XBee xbeeBrac = XBee(); // setting up XBee object
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 
bioData body; 

uint8_t payload[] = { 0, 0, 0, 0, 0 }; // {heart rate (whole number), blood ox (whole number?), body temp tens-ones, body temp tenths, emergency status (0 or 1)}
uint8_t updates[] = { 0, 0, 0, 0, 0, 0 };

// SH and SL address of coordinator (receiving)
XBeeAddress64 addrCoord = XBeeAddress64(0x0013a200, 0x4206609b);
ZBTxRequest zbTx = ZBTxRequest(addrCoord, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

void setup() {
   Serial.begin(9600); // serial terminal
   Serial2.begin(9600); // interfacing with Xbee

   Wire.begin();
   startTempSens();
   startBOxSens();
   // receiving/setup of antenna goes BEFORE millis call, to ensure upon startup it starts on 60 seconds
   xbeeBrac.setSerial(Serial2);
   // Transmit packet to coordinator to initiate handshake
   receive(); // receives thresholds if there are any
   startTime = millis(); // gets initial start time
   Serial.println("Clock setup complete");
}

void loop() {
  // Take vital measurements with sensors
    body = bioHub.readBpm();
    hRateMeas = getHeartRate();
    bloodOxMeas = getBloodOx();
    bodyTempMeas = getBodyTemp();
    checkVitals(); // checks vitals, sends emergency flag if one is out of range
  // Timing code for soon to be 60 second intervals
    currTime = millis();
    if (currTime - startTime >= period) {
      Serial.println("20 seconds have elapsed");
      if (transmit(0)) {
        Serial.println("Congratulations! Transmission successful.");
      }
      else {
        Serial.println("Transmission unsuccessful.");
        // shit do we need to make this thing try again
      }
      startTime = currTime; // reset for next iteration
    }
    delay(5000);

}

int transmit(int emergFlag) {
  int txSuccess = 0; // return value, is 0 if XBee success
  payload[4] = (char)emergFlag; // set emergency flag in payload to input
  xbeeBrac.send(zbTx); // sends payload to XBee for transmission

  if (xbeeBrac.readPacket(500)) {
    // got something
    if (xbeeBrac.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbeeBrac.getResponse().getZBTxStatusResponse(txStatus);
      
      if (txStatus.getDeliveryStatus() == SUCCESSFUL) {
        txSuccess = 1; // delivery success!!
      } else {
        // did not work
        Serial.println("Transmission failed.");
      }
    }
  }
  else {
    Serial.println("Transmission failed.");
  }
  return txSuccess;
}

void receiveThres() {
  // updates thresholds if needed. receives data.
  if (receive()) {
      // looks for changes in data. **Assumes unchanged threshold has value 0**
    for (int i = 0; i < sizeof(updates); i++) {
      if (updates[i] != 0) {
        updateThres(updates[i], i);
      }
    }
  }
}

void updateThres(int newValue, int type) {
  // inputs may need to change to uint_8t
  switch (type) {
    case 0:
      hRateLower = newValue;
      Serial.println(hRateLower);
      break;
    case 1:
      hRateUpper = newValue;
      break;
    case 2:
      bloodOxLower = newValue;
      break;
    case 3:
      bloodOxUpper = newValue;
      break;
    case 4:
      bodyTempLower = newValue;
      break;
    case 5:
      bodyTempUpper = newValue;
      break;
    // no default because if the value is invalid nothing should happen or be changed
    
  }
}

void checkVitals() {
  // if measurement is outside thresholds, throw emergency alert (1)
  payload[0] = hRateMeas;
  payload[1] = bloodOxMeas;
  int bodyTempMeasOne = (int)bodyTempMeas;
  int bodyTempMeasTenth = (bodyTempMeas - bodyTempMeasOne) * 10;
  payload[2] = bodyTempMeasOne;
  payload[3] = bodyTempMeasTenth;
  
  if (hRateMeas < hRateLower || hRateMeas > hRateUpper) {
    Serial.println(hRateLower);
    Serial.println(hRateUpper);
    transmit(1);
  }
  else if (bloodOxMeas < bloodOxLower || bloodOxMeas > bloodOxUpper) {
    Serial.println("Emerg 2");
    transmit(1);
  }
  else if (bodyTempMeas < bodyTempLower || bodyTempMeas > bodyTempUpper) {
    Serial.println(bodyTempMeas);
    Serial.println(bodyTempLower);
    Serial.println(bodyTempUpper);
    transmit(1);
  }
}

int receive() {
  int newThres = 0;
  if (xbeeBrac.readPacket(500)) {
    Serial.println("Got something");
    if (xbeeBrac.getResponse().getApiId() == ZB_RX_RESPONSE) {
      Serial.println("Success");
      newThres = 1;
      xbeeBrac.getResponse().getZBRxResponse(rx);
      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
        Serial.println("ACK Moment");
      }
      else {
        Serial.println("No ACK?");
      }
      for (int i = 0; i < rx.getDataLength(); i++) {
        Serial.print((char)rx.getData(i));
        updates[i] = rx.getData(i);
      }
      Serial.println();
    }
    else if (xbeeBrac.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      xbeeBrac.getResponse().getModemStatusResponse(msr);
      if (msr.getStatus() == ASSOCIATED) {
        Serial.println("Associated (good)");
        newThres = 1;
      }
      else if (msr.getStatus() == DISASSOCIATED) {
        Serial.println("Disassociated");
      }
    }
    else {
      Serial.println("Error catch");
    }
  } 
  else {
    Serial.println("No packet sent"); 
  }
  return newThres;
}

void startTempSens() {
  while(!tempSensor.scanAvailableSensors()){
    Serial.println("Couldn't find the temperature sensor, please connect the sensor." );
    delay(1000);
  }
  tempSensor.begin();  
  Serial.println("Temperature sensor on");
}

void startBOxSens() {
  int result = bioHub.begin();
  if (result == 0) // No errors
    Serial.println("Sensor starting...");
  else
    Serial.println("Could not communicate with the sensor!");
 
  Serial.println("Configuring Sensor...."); 
  
  int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings. 
  if(error == 0){ // No errors
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }
  Serial.println("Loading up the buffer with data....");
  delay(5000);
}

float getBodyTemp() {
  return tempSensor.getTemperature();
}

int getHeartRate() {
  return body.heartRate;
}

int getBloodOx() {
  return body.oxygen;
}
