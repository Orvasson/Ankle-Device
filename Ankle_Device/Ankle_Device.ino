/*
  For SEN15805(TMP117) (Temperature 0.01 accuracy):
  //    Arduino connections:
  //
  //  |SEN15805 pin label| Pin Function         |Arduino Connection|
  //  |----------------- |:--------------------:|-----------------:|
  //  | GND              | Digital Gnd          |  GND             |
  //  | 3.3V             | Digital VDD          |  MCP1700 1.8V    |
  //  | SDA              | Serial Data          |  A4 (or SDA)     |
  //  | SCL              | Serial Clock         |  A5 (or SCL)     |
  //  | INT              | Interrupt            |  --              |
  Links:
  1)https://github.com/abhaysbharadwaj/ArduinoPedometer
  2)http://librarymanager/All#SparkFun_u-blox_GNSS
*/

#include <Arduino_LSM9DS1.h>
#include <Wire.h> //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h"
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from our sensor

//attempt to store shit in txt file
#include <iostream>   //fixed
//for precision 0.01
#include <iomanip>      // std::setprecision 

//BLE library
#include <ArduinoBLE.h>
// The default address of the device is 0x48 = (GND)

TMP117 sensor; // Initalize sensor for temperature

// The only I2C address for this and all u-Blox GPS products is 0x42
SFE_UBLOX_GPS myGPS;

static const char* lay_down = "Kathistos!";
static const char* not_lay_down = "Oxi Kathistos!";
static const char* stretch_legs = "Tentwmena podia";
//for Temperature Service
//https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.temperature.xml
//Service UUID
#define BLE_Temperature_Service_UUID     "1810"   //from 16-bit UUID NumbersDocument
#define BLE_Steps_Servive_UUID           "1814" // physical activity monitor 1814   183E
#define BLE_GPS_Service_UUID             "1819"  // Location and Navigation

//Characteristics UUID

//Temp Characteristic
#define BLE_Temperature_UUID             "2A66"        //from 16-bit UUID NumbersDocument
//Steps-Meters-Flag Characteristic
#define BLE_Steps_UUID                   "7abdb551-a0f0-444d-8fb0-96356e2d212b"      //from 16-bit UUID NumbersDocument 2B40    2ACF
#define BLE_Meters_UUID                  "e1caf428-19b8-4f36-80e6-9d4b4524cc96"     //random uuid 2701
//GPS Characteristic
#define BLE_latitude_UUID                "2AAE"       //from 16-bit UUID NumbersDocument
#define BLE_longitude_UUID               "2AAF"      //  //from 16-bit UUID NumbersDocument
#define BLE_SIV_UUID                      "f6ef9d90-1c28-4ddb-a31b-ad9680469e50"    // random UUID


/
//----------------------------------------------------------------------------------------------------------------------
// BLE Temperature Measurment && Step Counter && GPS Lat + Long
//----------------------------------------------------------------------------------------------------------------------
BLEService temperatureService( BLE_Temperature_Service_UUID );  // Service for temperature measurements
BLEService stepcounterService( BLE_Steps_Servive_UUID );    // Service for steps
BLEService GPSlatlongService( BLE_GPS_Service_UUID );   //service for gps
//Temperature Characteristic
BLEIntCharacteristic temperatureCharacteristic( BLE_Temperature_UUID , BLERead | BLENotify ); // remote clients will only be able to read this
//Steps Characteristic
BLEIntCharacteristic stepsCharacteristic( BLE_Steps_UUID , BLERead | BLENotify );
//BLEStringCharacteristic layCharacteristic("2A56", BLERead, 17); // String
BLEFloatCharacteristic metersCharacteristic( BLE_Meters_UUID , BLERead | BLENotify );
//GPS Characteristic
BLEIntCharacteristic latCharacteristic( BLE_latitude_UUID, BLERead | BLENotify );
BLEIntCharacteristic longiCharacteristic( BLE_longitude_UUID, BLERead | BLENotify );
BLEIntCharacteristic SIVCharacteristic( BLE_SIV_UUID , BLERead | BLENotify );





#define delays                     25000    //45000
#define TEMP_DELAYS                15000     //4 hours  14400000
#define five_min_delay              2000        //5 minutes   300000
#define one_hour_delay              3600000         //1 hour    3600000

float xavg, yavg, zavg;
int flag = 0, old_steps = 0, lay_flag = 0, stretch_flag = 0, newsteps = 0, meter_flag;
float threshhold = 30.0;
float meter = 0.0;
long timer1, timer2, timer3, timer4, timer5, timer6, lastTime = 0, timer7, timer8;
int valid_gps, satel;
uint32_t steps = 0, steps2 = 0;

//STORED on Ram for BLE
int16_t stored_temp = 0; // value that we sent through BLE
uint32_t longi = 0, lati = 0;


void setup() {

  Serial.begin(9600);
  //while (!Serial);
  // Serial.println("Started");

  if (!IMU.begin()) {
    // Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //  Serial.print("Accelerometer sample rate = ");
  //  Serial.print(IMU.accelerationSampleRate());
  //  Serial.println(" Hz");
  //  Serial.println();

  //TEMPERATURE CODE Init
  Wire.begin();

  Serial.begin(115200);
  // while (!Serial);
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  //Check if TMP117 is working and start the readings
  Serial.println("TMP117 Init for Readings");
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    //   Serial.println("Begin measurements");
  }
  else
  {
    //   Serial.println("Device failed to setup- Freezing code.");
    while (1); // Runs forever
  }

  //END_TEMP//


  //GPS SAM M8Q
  Serial.begin(115200);
  // while (!Serial); //Wait for user to open terminal
  // Serial.println("SparkFun Ublox Example");

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    // Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  //Power Save Mode for GPS
  if (myGPS.powerSaveMode()) { // Defaults to true
    Serial.println(F("Power Save Mode enabled."));
  } else {
    // Serial.println(F("***!!! Power Save Mode FAILED !!!***"));
  }
  //GPS END

  //calibrate for step calculation
  delay(1000);
  calibrated();
  delay(250);

  //BLE
  Serial.begin(9600);    // initialize serial communication
  // while (!Serial);

  if (!BLE.begin()) {   // initialize BLE
    Serial.println("starting BLE failed!");
    while (1);
  }
  //set advertised local name and service
  BLE.setDeviceName("Ankle Device");
  BLE.setLocalName("Ankle Device");  // Set name for connection

  BLE.setAdvertisedService(temperatureService); // Advertise service
  BLE.setAdvertisedService(stepcounterService); // Advertise service
  BLE.setAdvertisedService(GPSlatlongService); // Advertise service

  //BLE and characteristics for temperatureService
  temperatureService.addCharacteristic(temperatureCharacteristic); // Add characteristic to service
  //BLE and characteristics for stepcounterService
  stepcounterService.addCharacteristic( stepsCharacteristic );
  // stepcounterService.addCharacteristic( layCharacteristic );
  stepcounterService.addCharacteristic( metersCharacteristic );
  //BLE and characteristics for GPS
  GPSlatlongService.addCharacteristic( latCharacteristic );
  GPSlatlongService.addCharacteristic( longiCharacteristic );
  GPSlatlongService.addCharacteristic( SIVCharacteristic );

  //add Services
  //  BLE.addService( temperatureService );
  BLE.addService( stepcounterService ) ;
  BLE.addService( GPSlatlongService );
  BLE.addService( temperatureService );

  BLE.advertise();  // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  // Serial.println("Waiting for connections...");

  timer1 = millis();
  timer2 = millis();
  timer3 = millis();
  timer4 = millis();
  timer5 = millis();
  timer6 = millis();
  timer7 = millis();
  timer8 = millis();
}

void loop() {

  BLEDevice central = BLE.central();                    //Waits for BLE Central device to connect
  delay(1000);
  if (central)
  {
    while (central.connected()) {
      getsteps(); // check if new steps made
      if (newsteps > 4) {
        newsteps = 0 ;
      }
      
      steps = newsteps + steps;
      if ( steps != steps2) {
        steps2 = steps;
        meter_flag = 1;
      }
      Serial.println("Ekane tosa vimata : ");
      Serial.println(steps);
      stepsCharacteristic.writeValue(steps2);
      if ( steps2 >= 10) {
        if ( ( (steps2 % 10) == 0 ) && (meter_flag == 1) ) {
          timer5 = millis();
          meter = meter + 7.62 ;
          meter_flag = 0;
         
            metersCharacteristic.writeValue( meter);
          
        }
      }
      // Call function positionGPS to get latitude and longitude
      positionGPS();
      if (  valid_gps >= 1 )  {
        latCharacteristic.writeValue( lati );
        longiCharacteristic.writeValue( longi );
        SIVCharacteristic.writeValue( satel );
      }
      //Check if timer is on to get the temperature reading
      timer3 = millis();
      if ( timer3 > timer4 + 5000 ) { // TEMP_DELAY
        temperatures();
        //set the value for charasteristics
        temperatureCharacteristic.writeValue( stored_temp); //temperature value
        timer4 = millis();
      }
      //calibrate again every 5 minute
      timer1 = millis();
      if ( timer1 > timer2 + five_min_delay ) {
        calibrated();
        timer2 = millis();
      }
    }
  }
  Serial.println("Not yet connected");
}

void getsteps() {

  float totavg = 0;
  float totvect[20] = {0};
  float totave[20] = {0};
  float xaccl[20] = {0};
  float yaccl[20] = {0};
  float zaccl[20] = {0};
  float x, y, z;
  newsteps = 0;
  for (int j = 0; j < 20; j++) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      xaccl[j] = x * 100;
      delay(1);
      yaccl[j] = y * 100;
      delay(1);
      zaccl[j] = z * 100;


      totvect[j] = sqrt(((xaccl[j] - xavg) * (xaccl[j] - xavg)) + ((yaccl[j] - yavg) * (yaccl[j] - yavg)) + ((zaccl[j] - zavg) * (zaccl[j] - zavg)));
      if (j > 2) {
        totave[j] = (totvect[j] + totvect[j - 1]) / 2 ;
        totavg = totave[j] + totavg;
        delay(80);
        //get the correct threshhold depending on the calibration
        if ( (j == 3) || (j == 9) || (j == 12)  || (j == 15) || (j == 18) ) {
          //check threshhold range
          if ( ( totave[j - 3] < 16) && ( totave[j - 2] < 16) && ( totave[j - 1] < 16) && ( totave[j] < 16) ) {
            threshhold = 20.0;
            if (j == 3) {
              newsteps = 0;
            } else if ( ( j == 9 ) && ( newsteps > 3 ) ) {
              newsteps = newsteps - 3;
            } else if ( ( j == 12 ) && ( newsteps > 3 ) ) {
              newsteps = newsteps - 3;
            } else if ( ( j == 15 ) && ( newsteps > 3 ) ) {
              newsteps = newsteps - 3;
            } else if ( ( j == 18 ) && ( newsteps > 3 ) ) {
              newsteps = newsteps - 3;
            }
          } else {
            threshhold = 27.0;
          }
        }
        //check if laying down or if leg stretched while sitting (driving/clutch)
        if (j > 3) {
          //bool values of raw data
          bool Xcheck = ( ( xaccl[j - 3] < -40 ) && ( xaccl[j - 3] > -105) );
          bool Xcheck2 = ( ( xaccl[j - 2] < -47 ) && ( xaccl[j - 2] > -97) );
          bool Xcheck3 = ( ( xaccl[j - 1] < -47 ) && ( xaccl[j - 1] > -97) );
          bool Xcheck4 = ( ( xaccl[j] < -47 ) && ( xaccl[j] > -97) );
          bool YandZcheck = ( ( yaccl[j - 3] < -57 ) && ( zaccl[j - 3] < 28 ) );
          bool YandZcheck2 = ( ( yaccl[j - 2] < -50 ) && ( zaccl[j - 2] < 25 ) );
          bool YandZcheck3 = ( ( yaccl[j - 1] < -50 ) && ( zaccl[j - 1] < 25 ) );
          bool YandZcheck4 = ( ( yaccl[j] < -50 ) && ( zaccl[j] < 25 ) );
          //lay down
          if ( ( xaccl[j - 3] > yaccl[j - 3] ) && ( xaccl[j - 2] > yaccl[j - 2] ) && ( xaccl[j - 1] > yaccl[j - 1] ) && ( xaccl[j] > yaccl[j] ) && ( yaccl[j - 1] < -45 ) && ( yaccl[j] < -45 ) ) {
            lay_flag = 1;
            stretch_flag = 0;
           
            delay(5);
            //  return;
          } else {
            lay_flag = 0;
           
            delay(10);
          }
          //stretched leg
          if ( (Xcheck) && (Xcheck2) && (Xcheck3) && (Xcheck4) && (YandZcheck) && (YandZcheck2) && (YandZcheck3) && (YandZcheck4) ) { //sitting with stretched legs, various positions
            stretch_flag = 1;
            lay_flag = 0;
           
            delay(5);
            //  return;
          } else {
            stretch_flag = 0;
          }

        }
        //         Serial.println("to totave einai: ");
        //          Serial.println(totave[j]);
        //calculate if we made new steps
        if ( (j > 4) && (stretch_flag == 0) && (lay_flag == 0) ) {  // && (stretch_flag = 0) && (lay_flag = 0)
          // values for sudden movements of leg while sitting
          float XminusY = abs( xaccl[j - 2] - yaccl[j - 2] );
          float XminusY2 = abs( xaccl[j - 1] - yaccl[j - 1] );
          float XminusY3 = abs( xaccl[j] - yaccl[j] );
          float ZminusY = abs( zaccl[j] - yaccl[j] );
          bool Xunder185 = ( !(xaccl[j] < -180) || !( xaccl[j - 1] < -180) || !( xaccl[j - 2] < -180) || !( xaccl[j - 3] < -180) || !(xaccl[j - 4] < -180) || !(xaccl[j - 5] < -180) || !(xaccl[j - 6] < -180) );


          if ( ( totave[j] > threshhold ) && ( flag == 0 ) && (yaccl[j - 2] > yaccl[j - 1]) && (yaccl[j - 1] > yaccl[j]) && (zaccl[j - 2] > zaccl[j - 1])  ) //  && (zaccl[j-1] > zaccl[j])
          {
            if ( !(XminusY < 45) && !(XminusY2 < 45) && !(XminusY3 < 45) && !(ZminusY < 35) ) {
              if ( Xunder185 ) {
                newsteps = newsteps + 1;
                flag = 1;
                old_steps = 1;
                delay(8);
              }
            }
          } else if ( ( totave[j] > threshhold ) && ( flag == 0 ) && (yaccl[j - 2] > yaccl[j - 1]) && (yaccl[j - 1] > yaccl[j]) && (zaccl[j - 2] < zaccl[j - 1])  ) {
            if ( !(XminusY < 45) && !(XminusY2 < 45) && !(XminusY3 < 45) && !(ZminusY < 35) ) {
              if ( Xunder185 ) {
                newsteps = newsteps + 1;
                flag = 1;
                old_steps = 1;
                delay(8);
              }
            }
          } else if ( ( totave[j] > threshhold ) && ( flag == 0 ) && (yaccl[j - 2] < yaccl[j - 1]) && (yaccl[j - 1] < yaccl[j]) && (zaccl[j - 2] > zaccl[j - 1])  ) {
            if ( !(XminusY < 45) && !(XminusY2 < 45) && !(XminusY3 < 45) && !(ZminusY < 35) ) {
              if ( Xunder185 ) {
                newsteps = newsteps + 1;
                flag = 1;
                old_steps = 1;
                delay(8);
              }
            }
          } else if ( ( totave[j] > threshhold ) && ( flag == 1 ) ) {
            old_steps = 0;
          }
          if (totave[j] < threshhold  && flag == 1)
          {
            flag = 0;
            old_steps = 0;
          }

        }
      }
    }
  }
 
  totavg = (totavg / 19.0);
 
  if (totavg > 31.50) {
    newsteps = 0;
  }
}


void calibrated() {

  float xval[40] = {0};
  float yval[40] = {0};
  float zval[40] = {0};
  float x, y, z;
  float x1, y1, z1;
  float sum = 0;
  float sum1 = 0;
  float sum2 = 0;

  for (int j = 0; j < 40; j++) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      //      Serial.println("To " + x" to " + y " to " + z);
      //      Serial.print();
      //x values
      x1 = x * 100;
      xval[j] = float(x1);
      sum = xval[j] + sum;
      delay(10);
      //y values
      y1 = y * 100;
      xval[j] = float(y1);
      sum1 = xval[j] + sum1;
      delay(10);
      //z values
      z1 = z * 100;
      zval[j] = float(z1);
      sum2 = zval[j] + sum2;

    }
  }
  xavg = sum / 40.0;
  yavg = sum1 / 40.0;
  zavg = sum2 / 40.0;
  //
 
}


void positionGPS() {

  if (millis() - lastTime > 45000)
  {
    lastTime = millis(); //Update the timer

    long latitude = myGPS.getLatitude();
    //    Serial.print(F("Lat: "));
    //    Serial.print(latitude);

    //We check if the patient changed latitude
    if ( abs(latitude - lati) >= 90 ) {
      lati = latitude ;
    }
    long  longitude = myGPS.getLongitude();
    //    Serial.print(F(" Long: "));
    //    Serial.print(longitude);
    //    Serial.print(F(" (degrees * 10^-7)"));

    //We check if the patient changed longitude
    if ( abs(longitude - longi) >= 90 ) {
      longi = longitude;
    }

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    if ( SIV > 2 ) {
      valid_gps = 1;
      satel = SIV;
    } else {
      valid_gps = 0;
    }

    //   Serial.println();
  }
}

void temperatures() {

  //  sensor.setOneShotMode(); // Sets mode register value to be 0b11

  delay(200);

  float sum = 0;
  float tempCelcius = 0.0; //the printed value of temperature
  float thermo[10];

  //Temp prints

  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    // float tempC = sensor.readTempC();
    for (int iota = 1; iota < 11; iota++)
    {
      float tempC = sensor.readTempC();
      thermo[iota] = tempC;
      sum = sum + thermo[iota];
      tempCelcius = (sum / iota);
    }
    // Print temperature in °C

    //   Serial.print("Temperature in Celsius after 10 readings: ");
    //   Serial.println(tempCelcius, 4);

    // Keep precision of 0.01 to send in BLE
    std::cout << std::setprecision(3) << tempCelcius << '\n';
    std::cout << std::fixed;
    //   Serial.println(tempCelcius);

    //store in global variable
    stored_temp = (tempCelcius * (100));

    sensor.setShutdownMode(); // Sets mode register value to be 0b01

  }

}
