/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Don/Documents/Particle/projects/MEG-Boron/MEG/src/MEG.ino"
/*
 * Project MEG
 * Author: Don Hagen
 * Date: 02Jan2021
 */

//#include <Wire.h> //Needed for I2C
#include <SparkFun_Ublox_Arduino_Library.h>
#include <MMA8452Q.h>
#include <SparkFun_Qwiic_Rfid.h>
#include <Particle.h>

//Globals
void setup();
void loop();
void stolenAlert(bool send);
void enableRelay(String command);
void disableRelay(String command);
int arm(String command);
int silence(String command);
void sentry();
void getGPSData();
void startRFID();
bool startAccel();
bool startGPS();
bool movementDetection();
int sendSMS(String body);
void checkRFID();
int disable(String command);
void cloudRegistration();
void printGPS();
void setCharging(bool enable);
#line 14 "c:/Users/Don/Documents/Particle/projects/MEG-Boron/MEG/src/MEG.ino"
bool armed = true;
int armedInt = 1;
bool stolen = false;
bool sendAlert = true;
int alertInt = 1;
bool on;
long previousMillis = 0;
long interval = 300000; //5 minutes
int disabled = 0;
bool registrationComplete = false;
String status = "";

//GPS stuff
String latString = "";
String lonString = "";
long latitude = 0;
long longitude = 0;
int32_t spd;    
long previousAwakeTime = 0;
long currentAwakeTime = 0;
long awakeDuration = 60000;
int sleepDuration = 60000;
//long GPSCheck = 0;
//long GPSStart = 0;

//RFID stuff
#define RFID_ADDR 0x7D // Default I2C address
String tag;

//Enable offline functionality
SYSTEM_THREAD(ENABLED);

//Objects
SFE_UBLOX_GPS myGPS;
MMA8452Q accel;
Qwiic_Rfid myRfid(0xD7);

void setup() {

  cloudRegistration();

  Serial.begin(115200);
  Wire.begin();

  //turn off charging and charging LED
  //PMIC().disableCharging();
  setCharging(false);

  //power sense pin
  pinMode(D5, INPUT);

  //QWIIC power FET
  //on during startup to allow I2C communication
  pinMode(D4, OUTPUT);
  digitalWrite(D4, HIGH);

  //enable relay FET (NO)
  pinMode(D3, OUTPUT);
  digitalWrite(D3, LOW);

  //disable relay FET (NC)
  pinMode(D2, OUTPUT);
  digitalWrite(D2, LOW);

  startRFID();
  startAccel();
  startGPS();
 
}

void loop()
{
  //turn on QWIIC FET
  digitalWrite(D4, HIGH);

  
  //Serial.println("here");
  while ((currentAwakeTime - previousAwakeTime < awakeDuration) || on)
  {
    getGPSData();
    currentAwakeTime = millis();
    
    //////////for switch debugging only//////////////////////////////////////////////////
    if (digitalRead(D2) == HIGH)
    {
      on = true;
      Serial.println("Vehicle On");
    }
    else
    {
      Serial.println("Vehicle Off ##################################");
      on = false;
    }
    //////////end switch debugging///////////////////////////////////////////////////////

    if (!on && !armed)
    {
      arm("arm");
    }
    if (armed && !stolen)
    {
      sentry();
    }
    if (on && armed)
    {
      checkRFID();
    }
    if (on && !armed)
    {
      Serial.println("Ready to start");
      enableRelay("on");
    }
    if (stolen)
    {
      stolenAlert(sendAlert);
      checkRFID();
    }
    if (disabled == 1)
    {
      Serial.println("Vehicle disabled");
    }
    status= "";
    status = String::format("%2.8f,%3.8f,%d,%d,%d", (latitude / 10000000.0), (longitude / 10000000.0), alertInt, disabled, armedInt);
  }
  
  if (!on && !stolen)
  {

    arm("arm");

    digitalWrite(D4, LOW);
    
    SystemSleepConfiguration config;
    config.mode(SystemSleepMode::STOP)
      .gpio(D5, RISING)
      .duration(sleepDuration)
      .network(NETWORK_INTERFACE_CELLULAR);
    SystemSleepResult sleepResult = System.sleep(config);
    
    //if the app wakes electron up, stay awake for 60s to communicate
    if (sleepResult.wakeupReason() == SystemSleepWakeupReason::BY_NETWORK) {
    // Waken by network activity 
      Particle.publish("status", status, PRIVATE);
      awakeDuration = 60000;
    }
    else
    {
      awakeDuration = 10000;
      //sleepDuration = 60000;
    }
    previousAwakeTime = millis();
    currentAwakeTime = millis();
    
  }
  
}

void stolenAlert(bool send)
{
  if (send)
  {
    long currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {
      previousMillis = currentMillis;
      Serial.println("STOLEN MESSAGE SENT");
      disable("disable");
      String stolenString = "Stolen. Last location: https://www.google.com/maps/search/?api=1&query=";
      stolenString += latString;
      stolenString += ",";
      stolenString += lonString;
      sendSMS(stolenString);
    }
  }
}

void enableRelay(String command)
{
  if (command == "on")
    digitalWrite(D5, HIGH);
  else if (command == "off")
    digitalWrite(D5, LOW);
}

void disableRelay(String command)
{
  if (command == "on")
    digitalWrite(D4, HIGH);
  else if (command == "off")
    digitalWrite(D4, LOW);
}

int arm(String command)
{
  if (command == "disarm")
  {
    Serial.println("Disarmed");
    enableRelay("on");
    disableRelay("off");
    armed = false;
    armedInt = 0;
    //stolen = false;
    return 1;
  }
  if (command == "arm")
  {
    Serial.println("Armed");
    //TODO: set LEDs
    enableRelay("off");
    armed = true;
    armedInt = 1;
    return 2;
  }
  return 0;
}

int silence(String command)
{
  if (command == "send")
  {
    Serial.println("Notifications allowed.");
    sendAlert = true;
    alertInt = 1;
    return 1;
  }
  if (command == "stop")
  {
    Serial.println("Notifications silenced.");
    sendAlert = false;
    alertInt = 0;
    return 2;
  }
  return 0;
}

void sentry()
{
  Serial.println("Guarding...");
  getGPSData();
  if (movementDetection() && (spd > 10))
  {
    stolen = true;
    arm("arm");
    disable("disable");
    Serial.println("STOLEN MESSAGE SENT");
  }
}

void getGPSData()
{
  Serial.println("Getting location................................");
  // fixType 0=no, 3=3D, 4=GNSS+Deadreckoning
  if (myGPS.getFixType() == 0)
    awakeDuration = 35000;

  latitude = myGPS.getLatitude();
  longitude = myGPS.getLongitude();
  spd = myGPS.getGroundSpeed() / 278;

  latString = "";
  lonString = "";

  latString += String(latitude / 10000000.0);
  lonString += String(longitude / 10000000.0);
}

void startRFID()
{
  if (myRfid.begin())
    Serial.println("Ready to scan some tags!");
  else
    Serial.println("Could not communicate with Qwiic RFID!");
  myRfid.clearTags();
}

bool startAccel()
{
  bool success = false;
  if (accel.begin() == false)
  {
    success = false;
    Serial.println(" Accelerometer not Connected. Please check connections.");
  }
  else
  {
    Serial.println("Accelerometer started.");
    success = true;
  }
  return success;
}

bool startGPS()
{
  bool success = false;
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
  }
  else
  {
    Serial.println("GPS started.");
    success = true;
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();        //Save the current settings to flash and BBR
  return success;
}

bool movementDetection()
{
  Serial.println("checking movement");
  bool movement = false;
  if (accel.available())
  { // Wait for new data from accelerometer
    if (accel.readTap() > 0)
    {
      Serial.println("Movement detected.");
      movement = true;
    }
  }
  return movement;
}

int sendSMS(String body)
{
  Particle.publish("twilio_sms", body, PRIVATE);
  return 0;
}

void checkRFID()
{
  Serial.println("Checking RFID");
  tag = myRfid.getTag();
  if (tag != "000000")
  {
    Serial.print("Tag: ");
    Serial.println(tag);
    //delay(5000);
  }

  delay(50);
  if (tag == "8507175142115" || tag == "140891033216") //change this to valid UIDs
  {
    Serial.println("Authorized Access");
    armed = false;
    armedInt = 0;
  }
  else if (tag == "850719716550") //change this to valid UIDs
  {
    Serial.println("Master Key Authorized");
    armed = false;
    armedInt = 0;
    stolen = false;
    disableRelay("off");
    disabled = 0;
  }
  else
  {
    Serial.println(" Access denied");
  }
  //myRfid.clearTags();
}

int disable(String command)
{
  Serial.println("Disabling relay 2");
  if (command == "disable")
  {
    disabled = 1;
    armed = true;
    armedInt = 1;
    disableRelay("on");
    enableRelay("off"); 
    return 1;
  }
  else if (command == "enable")
  {
    disabled = 0;
    disableRelay("off");
    enableRelay("on");
    return 2;
  }
  return 0;
}

void cloudRegistration()
{
  bool latVarSuccess = Particle.variable("latitude", latString);
  bool lonVarSuccess = Particle.variable("longitude", lonString);
  bool alertVarSuccess = Particle.variable("sendAlert", alertInt);
  bool isDisabledVarSuccess = Particle.variable("disabled", disabled);
  bool armedVarSuccess = Particle.variable("armed", armedInt);
  bool armedBoolVarSuccess = Particle.variable("armedBool", armed);
  bool disableFunSuccess = Particle.function("disable", disable);
  bool armFunSuccess = Particle.function("arm", arm);
  bool alertFunSuccess = Particle.function("silence", silence);
  Particle.variable("status", status);
  if (latVarSuccess == false || lonVarSuccess == false || alertVarSuccess == false || isDisabledVarSuccess == false ||
      armedVarSuccess == false || disableFunSuccess == false || armFunSuccess == false || alertFunSuccess == false ||
      armedBoolVarSuccess == false)
  {
    Serial.println("Failed registration.");
  }
  else
    Serial.println("Registration Successful.");
}

/*
Function for debugging GPS data, prints to console
*/
void printGPS()
{
  Serial.print("Latitude (deg): ");
  Serial.println(latString);
  Serial.print("Longitude (deg): ");
  Serial.println(lonString);
  Serial.print("Speed: ");
  Serial.print(spd);
  Serial.println(" km/h");
}

void setCharging(bool enable) {

	PMIC pmic;

	// DisableCharging turns of charging. DisableBATFET completely disconnects the battery.
	if (enable) {
		pmic.enableCharging();
		pmic.enableBATFET();
	}
	else {
		pmic.disableCharging();
		pmic.disableBATFET();
	}

	// Disabling the watchdog is necessary, otherwise it will kick in and turn
	// charing at random times, even when sleeping.

	// This disables both the watchdog and the charge safety timer in
	// Charge Termination/Timer Control Register REG05
	// pmic.disableWatchdog() disables the watchdog, but doesn't disable the
	// charge safety timer, so the red LED will start blinking slowly after
	// 1 hour if you don't do both.
	byte DATA = pmic.readChargeTermRegister();

	if (enable) {
		DATA |= 0b00111000;
	}
	else {
		// 0b11001110 = disable watchdog
		// 0b11000110 = disable watchdog and charge safety timer
		DATA &= 0b11000110;
	}

	// This would be easier if pmic.writeRegister wasn't private (or disable
	// charge safety timer had an exposed method
	Wire1.beginTransmission(PMIC_ADDRESS);
	Wire1.write(CHARGE_TIMER_CONTROL_REGISTER);
	Wire1.write(DATA);
	Wire1.endTransmission(true);
}