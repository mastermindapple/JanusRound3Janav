void convertCoordinatesToCartesian(float latitude, float longitude);


#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// --- rest of your gps-chip-example.ino code ---
#include "NMEA.h"
#include <HardwareSerial.h>

#include <SoftwareSerial.h>
SoftwareSerial gnssSerial(4, 3); 

#define LEN(arr) ((int)(sizeof(arr) / sizeof(arr)[0]))


enum FlightState { PRELAUNCH, ASCENT, APOGEE, DESCENT, LANDED };
FlightState currentState = PRELAUNCH;

double lastAltitude = 0;
bool reachedApogee = false;

// --- Fake NMEA test sentences ---
// Example 1: 545.4m altitude
const char *testNMEA1 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
// Example 2: higher altitude (simulate ascent)
const char *testNMEA2 = "$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,650.0,M,46.9,M,,*42";
// Example 3: lower altitude (simulate descent)
const char *testNMEA3 = "$GPGGA,123521,4807.038,N,01131.000,E,1,08,0.9,20.0,M,46.9,M,,*4F";

union {
  char bytes[4];
  float valor;
} velocidadeGPS;

float latitude;
float longitude;

// Creates a GPS data connection with sentence type GPRMC
NMEA gps(GPRMC);

TinyGPSPlus gps;

void printState(FlightState state) {
  switch (state) {
    case PRELAUNCH: Serial.print("PRELAUNCH"); break;
    case ASCENT:    Serial.print("ASCENT"); break;
    case APOGEE:    Serial.print("APOGEE"); break;
    case DESCENT:   Serial.print("DESCENT"); break;
    case LANDED:    Serial.print("LANDED"); break;
  }

  // --- State machine logic ---
void updateStateMachine(double currentAltitude) {
  switch (currentState) {
    case PRELAUNCH:
      if (currentAltitude > 10) currentState = ASCENT;
      break;

    case ASCENT:
      if (currentAltitude < lastAltitude) {
        currentState = APOGEE;
        reachedApogee = true;
      }
      break;

    case APOGEE:
      currentState = DESCENT;
      break;

    case DESCENT:
      if (currentAltitude < 15 && reachedApogee) {
        currentState = LANDED;
      }
      break;

    case LANDED:
      // Stay here
      break;
  }

  lastAltitude = currentAltitude;
}

// Feed one NMEA string into TinyGPS++
void feedNMEA(const char *nmea) {
  for (int i = 0; nmea[i]; i++) {
    gps.encode(nmea[i]);
  }
}


enum FlightState { PRELAUNCH, ASCENT, APOGEE, DESCENT, LANDED };
FlightState currentState = PRELAUNCH;

double lastAltitude = 0;
bool reachedApogee = false;

// --- Fake NMEA test sentences ---
// Example 1: 545.4m altitude
const char *testNMEA1 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
// Example 2: higher altitude (simulate ascent)
const char *testNMEA2 = "$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,650.0,M,46.9,M,,*42";
// Example 3: lower altitude (simulate descent)
const char *testNMEA3 = "$GPGGA,123521,4807.038,N,01131.000,E,1,08,0.9,20.0,M,46.9,M,,*4F";



void setup() {
  Serial.begin(9600);
  gnssSerial.begin(4800); // Serial1 is connected to the custom chip
  Serial.println("Data received from GPS Fake:");
}

void convertCoordinatesToCartesian(float latitude, float longitude) {
  // Convert from Degrees to Radians
  float latRadius = latitude * (PI) / 180;
  float lonRadius = longitude * (PI) / 180;

  int earthRadius = 6371; // Radius in km

  float posX = earthRadius * cos(latRadius) * cos(lonRadius);
  float posY = earthRadius * cos(latRadius) * sin(lonRadius);

  Serial.print("        X: ");
  Serial.println(posX);

  Serial.print("        Y: ");
  Serial.println(posY);
}


void loop() {


  // Waits for serial port data
  while (gnssSerial.available()) {
    // Receives data from GPS serial port
    char serialData = gnssSerial.read();
    Serial.print(serialData);

    // Checks if the GPS sentence is valid
    if (gps.decode(serialData)) {
      // Checks if GPS status is 'A'
      if (gps.gprmc_status() == 'A') {
        // Receives GPS speed in km/h
        velocidadeGPS.valor = gps.gprmc_speed(KMPH);
      } else {
        velocidadeGPS.valor = 0;
      }

      latitude = gps.gprmc_latitude();
      longitude = gps.gprmc_longitude();

      // Add line break
      Serial.println();
      Serial.println();

      // Show Latitude
      Serial.print(" Latitude: ");
      Serial.println(latitude, 8);

      // Show Longitude
      Serial.print("Longitude: ");
      Serial.println(longitude, 8);

      // Show Speed ​​in km/h
      Serial.print("    Speed: ");
      Serial.print(velocidadeGPS.valor);
      Serial.println(" Km/h");

      // Converts Geographic Coordinates to Cartesian Plane
      convertCoordinatesToCartesian(latitude, longitude);



      static int step = 0;

  // Feed fake NMEA strings step by step
  if (step == 0) feedNMEA(testNMEA1);
  if (step == 1) feedNMEA(testNMEA2);
  if (step == 2) feedNMEA(testNMEA3);

  if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.time.isUpdated()) {
    // --- Print GNSS data cleanly ---
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
    Serial.print(" | Alt (MSL): "); Serial.print(gps.altitude.meters()); Serial.print(" m");

    // --- Update state machine ---
    updateStateMachine(gps.altitude.meters());

    Serial.print(" | State: "); printState(currentState);
    Serial.println("hello");

    delay(2000);  // Wait so you can read Serial Monitor output
    step++;
    if (step > 2) step = 0; // Loop through fake data
  }
  
    }
  }
}


