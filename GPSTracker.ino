/*-------------------------------------------------------------------
Vehicle Tracker firmware for Electron
(c) Copyright 2016, Philippe Hilger

This code is provided with NO WARRANTIES of any kind (usual
disclaimers apply) for non-commercial applications. If you change it
and start incurring high data usage, blame yourself :)
--------------------------------------------------------------------*/

/*SYSTEM_THREAD(ENABLED);*/

// Getting the library
#include "AssetTracker.h"
#include "math.h"

#define MAX_GPS_POINTS 4 // careful, no more than 255 bytes of data!

#define CNT_ADDR 0
#define LAT_ADDR sizeof(gpsCnt)
#define LON_ADDR LAT_ADDR+sizeof(lat)
#define TS_ADDR LON_ADDR+sizeof(lon)

#define BATTERY_DOWN_THRESHOLD 0.50
#define BATTERY_UP_THRESHOLD 0.70

ApplicationWatchdog wd(60000, System.reset);

int soft_reset = D0;
int soft_mode = D1;
int hard_reset = D2;
int LED7 = D7;

long reset_required = 0;
long mode_required = 0;
volatile bool upgrade_mode = false;

// Set whether you want the device to publish data to the internet by default here.
// 1 will Particle.publish AND Serial.print, 0 will just Serial.print
// Extremely useful for saving data while developing close enough to have a cable plugged in.
// You can also change this remotely using the Particle.function "tmode" defined in setup()
int transmittingData = 1;

// Used to keep track of the last time we published data
long lastGPSPublish = 0;
long lastAccelPublish = 0;
long lastAccelDetected = 0;
long previousAccelDetected = 0;
long lastRead = 0;
long lastActive = 0;

// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
long delayAccelMinutes = 5;
long delayGPSMinutes = 60;
long delayIdleSleepMinutes = 30;
long lastGPSDetected = 0;
long lastGPSRead = 0;
long lastGPSUpdate = 0;
long gpsReadSeconds = 15;


// Threshold to trigger a publish
// 9000 is VERY sensitive, 12000 will still detect small bumps
int accelThreshold = 9000;
int maxAccel = 0;
int maxX=0,
  maxY=0,
  maxZ=0;
String pubAccel = "undefined";

float lat[MAX_GPS_POINTS], lon[MAX_GPS_POINTS];
long ts[MAX_GPS_POINTS];

int gpsCnt = 0;
float sumLat = 0.0,
  sumLon = 0.0,
  lastLat = 0.0,
  lastLon = 0.0,
  avgLat = 0.0,
  avgLon = 0.0;
int latLonCnt;
float gpsThreshold = 0.0001; // 11.1m
float gpsError = 0.1; // 1.1km

int batteryWarning = 1;
long lastBatteryWarning = 0;
bool batteryDown = false; // under down threshold
bool batteryUp = false; // over up threshold
int gpsTrigger = 0;

// Creating an AssetTracker named 't' for us to reference
AssetTracker t = AssetTracker();

// A FuelGauge named 'fuel' for checking on the battery state
FuelGauge fuel;

void checkGPS(void);
void checkBattery(void);

// setup() and loop() are both required. setup() runs once when the device starts
// and is used for registering functions and variables and initializing things
void setup() {

    pinMode(soft_mode,INPUT_PULLUP);
    pinMode(soft_reset,INPUT_PULLUP);
    pinMode(hard_reset,INPUT_PULLUP);
    pinMode(LED7,OUTPUT);

    // Sets up all the necessary AssetTracker bits
    t.begin();

    // Opens up a Serial port so you can listen over USB
    Serial.begin(9600);

    // Enable the GPS module. Defaults to off to save power.
    // Takes 1.5s or so because of delays.
    t.gpsOn();

    // These three functions are useful for remote diagnostics. Read more below.
    Particle.function("tmode", transmitMode);
    Particle.function("batt", batteryStatus);
    Particle.function("gps", gpsPublish);
    Particle.function("aThresh",accelThresholder);
    Particle.function("reset",do_reset);
    lastActive = millis();
    reset_required = millis();
    mode_required = millis();
    /*attachInterrupt(soft_reset, do_reset, RISING);*/

    EEPROM.get(CNT_ADDR,gpsCnt);
    if (gpsCnt<=MAX_GPS_POINTS) {
      EEPROM.get(LAT_ADDR,lat);
      EEPROM.get(LON_ADDR,lon);
      EEPROM.get(TS_ADDR,ts);
    }
}

// loop() runs continuously
void loop() {
    // You'll need to run this every loop to capture the GPS output
    t.updateGPS();

    if (digitalRead(soft_reset) && !digitalRead(soft_mode)) {
      if (millis()-reset_required>200 && millis()-mode_required<2000) {
        if (gpsTrigger == 1) {
          gpsTrigger = 2;
        } else if (gpsTrigger == 3) {
          transmittingData = !transmittingData;
          Serial.println("Triggered No Tracking");
          gpsTrigger = 0;
        }
      }
      digitalWrite(LED7, (millis()-reset_required)%200>100);
      if (millis()-reset_required>3000) {
        pinMode(hard_reset,OUTPUT);
        pinResetFast(hard_reset);
        /*System.reset();*/
      }
    } else {
      reset_required = millis();
    }
    if (digitalRead(soft_mode)) {
      // special handling of mode-reset-mode to publish GPS
      if (millis()-mode_required>200 && millis()-reset_required<2000) {
        if (gpsTrigger == 2) {
          lastGPSPublish = 0;
          gpsTrigger = 0;
          Serial.println("Triggered GPS Publish");
        } else {
          gpsTrigger = 1;
        }
      }
      if (millis()-mode_required>10000) {
        gpsTrigger = 3;
      } else if (millis()-mode_required>3000) {
        gpsTrigger = 0;
        if (digitalRead(soft_reset)) {
          /*attachInterrupt(soft_reset, do_reset, RISING);*/
          upgrade_mode = true;
          digitalWrite(LED7, LOW);
          Cellular.listen();
        } else {
          digitalWrite(LED7, (millis()-mode_required)%100>50);
        }
      } else {
        digitalWrite(LED7, (millis()-mode_required)%500>250);
      }
    } else {
      mode_required = millis();
    }

    if (!digitalRead(soft_reset) && !digitalRead(soft_mode)) {
      // switch led off
      if (transmittingData) {
        digitalWrite(LED7, LOW);
      } else {
        digitalWrite(LED7, millis()%2000>1000 ? HIGH : LOW);
      }
    }

    if (millis()-lastGPSPublish<5000) {
      digitalWrite(LED7, millis()%500<500 ? HIGH : LOW);
    } else if (millis()-lastGPSPublish<6000) {
      digitalWrite(LED7, LOW);
    }

    if (upgrade_mode) {
      digitalWrite(LED7, (millis()-mode_required)%1000>500 ? HIGH : LOW);
    }

    /*if (millis()-lastActive<1000 || upgrade_mode) {
      return;
    }*/

    int newAccel = t.readXYZmagnitude();
    if (newAccel>maxAccel) {
      maxAccel = newAccel;
      maxX=t.readX();
      maxY=t.readY();
      maxZ=t.readZ();
      pubAccel = String::format("{\"t\":%d,\"x\":%d,\"y\":%d,\"z\":%d,\"v\":%.2f,\"c\":%d}",
        Time.now(),maxX,maxY,maxZ,fuel.getVCell(),(int)ceil(fuel.getSoC()+0.5));
      Serial.println("- acceleration: "+String(maxAccel)+" - "+pubAccel);
    }

    // Check if there's been a big acceleration or we published recently
    if(maxAccel > accelThreshold) {
      lastAccelDetected = millis();
    }

    if (lastAccelDetected>0
      && (previousAccelDetected == 0
        ||lastAccelDetected-previousAccelDetected > delayAccelMinutes*60*1000)) {

        previousAccelDetected = lastAccelDetected;
        // reset max
        maxAccel=0;

        // If it's set to transmit AND it's been at least delayMinutes since the last one...
        if (lastAccelPublish == 0
          || millis()-lastAccelPublish >= delayAccelMinutes*60*1000){
            // remember last supposed publish
            lastAccelPublish = millis();
            if (transmittingData) {
              Particle.publish("A", pubAccel, 60, PRIVATE);
            }
        }
    }

    checkGPS();

    if (millis()-lastActive>=1000) {
      lastActive = millis();
    }

    checkBattery();
}

void checkBattery() {
  String batteryAlarm;
  if (fuel.getSoC() <= BATTERY_DOWN_THRESHOLD
    && millis()-lastBatteryWarning > 5*60000) {
      batteryAlarm = String::format("Battery low, v=%.2f (%d%%)",
        fuel.getVCell(), (int)ceil(fuel.getSoC()+0.5));
      if (batteryWarning) {
        Particle.publish("B", batteryAlarm, 60, PRIVATE);
      }
      lastBatteryWarning = millis();
  } else if (fuel.getSoC() >= BATTERY_UP_THRESHOLD
      && lastBatteryWarning>0) {
        batteryAlarm = String::format("Battery back up, v=%.2f (%d%%)",
          fuel.getVCell(), (int)ceil(fuel.getSoC()+0.5));
        if (batteryWarning) {
          Particle.publish("B", batteryAlarm, 60, PRIVATE);
        }
        lastBatteryWarning = 0;
  }
}

void checkGPS() {
  // Dumps the full NMEA sentence to serial in case you're curious
  //Serial.println(t.preNMEA());
  if (t.gpsFix()) {
    float newLat = t.readLatDeg();
    float newLon = t.readLonDeg();

    if (lastGPSPublish == 0
      || sqrt((newLat-lastLat)*(newLat-lastLat)+(newLon-lastLon)*(newLon-lastLon)) > gpsThreshold
      || millis()-lastAccelDetected < 1000) {
      if ((lastGPSPublish == 0)
        || sqrt((newLat-lastLat)*(newLat-lastLat)+(newLon-lastLon)*(newLon-lastLon)) <= gpsError) {

        // update average latLon
        /*sumLat += newLat;
        sumLon += newLon;
        latLonCnt++;
        avgLat = sumLat/latLonCnt;
        avgLon = sumLon/latLonCnt;
        Serial.println(String::format("newAvg: %f|%f",sumLat/latLonCnt,sumLon/latLonCnt));*/

        if (millis()-lastGPSRead >= gpsReadSeconds*1000
          && (millis()-lastAccelDetected <= gpsReadSeconds*1000
          || lastGPSPublish == 0
          || millis()-lastGPSPublish > (delayGPSMinutes*60*1000))) {
          lat[gpsCnt] = newLat; //sumLat/latLonCnt;
          lon[gpsCnt] = newLon; //sumLon/latLonCnt;
          ts[gpsCnt] = Time.now();
          Serial.println(String::format("Adding %f|%f to table[%d]",
            lat[gpsCnt],lon[gpsCnt],gpsCnt));
          gpsCnt++;
          EEPROM.put(CNT_ADDR,gpsCnt);
          EEPROM.put(LAT_ADDR,lat);
          EEPROM.put(LON_ADDR,lon);
          EEPROM.put(TS_ADDR,ts);

          lastGPSRead = millis();
          /*sumLat=sumLon=avgLat=avgLon=0.0;*/
          latLonCnt=0;
        }
        lastLat = newLat;
        lastLon = newLon;
      }
    }
    if (gpsCnt>0
      && (lastGPSPublish == 0
      || gpsCnt >= MAX_GPS_POINTS
      || (millis()-lastGPSPublish) > (delayGPSMinutes*60*1000))) {
        if(transmittingData){
          // Remember when we are supposed to publish (even if we don't)
          lastGPSPublish = millis();
          String pubGPS = "";
          for (int i=0; i<gpsCnt; i++) {
            if (pubGPS!="") {
              pubGPS+=",";
            }
            pubGPS+=String::format("{\"t\":%d,\"l\":%.5f,\"L\":%.5f}",ts[i],lat[i],lon[i]);
          }
          Serial.println("-GPS: ["+pubGPS+"]");
          // Only publish if we're in transmittingData mode 1;
          Particle.publish("G", "["+pubGPS+"]", 60, PRIVATE);
        }
        gpsCnt=0;
      }
  }
}

// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command){
    transmittingData = atoi(command);
    return 1;
}

// Actively ask for a GPS reading if you're impatient. Only publishes if there's
// a GPS fix, otherwise returns '0'
int gpsPublish(String command){
  lastGPSPublish = 0; // force publishing
  checkGPS();
  return t.gpsFix() ? 1 : 0;
}

// Lets you remotely check the battery status by calling the function "batt"
// Triggers a publish with the info (so subscribe or watch the dashboard)
// and also returns a '1' if there's >10% battery left and a '0' if below
int batteryStatus(String command){
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("B",
          String::format("{\"v\":%.2f,\"c\":%.2f}",fuel.getVCell(),fuel.getSoC()),
          60, PRIVATE
    );
    // if there's more than 10% of the battery left, then return 1
    if(fuel.getSoC()>10){ return 1;}
    // if you're running out of battery, return 0
    else { return 0;}
}

// Remotely change the trigger threshold!
int accelThresholder(String command){
    accelThreshold = atoi(command);
    return 1;
}

int do_reset(String command) {
  /*detachInterrupt(soft_reset);*/
  upgrade_mode = false;
  System.reset();
  return 1;
}
