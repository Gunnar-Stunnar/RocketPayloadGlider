/*
  Venus GPS with TinyGPS++
  A glider Guidance System
  Original by Jeff Dunker
  by William H and Gunnar E
*/
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>
#include <Narcoleptic.h>

Servo PitchServo;  // create servo object to control a servo
Servo YawServo;

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define GPSBaud 9600
#define ConsoleBaud 4800  // Argent Radiosheild also uses
#define XbeeBaud 4800

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

// The serial connection to the GPS device
SoftwareSerial venus(10, 11);

// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;
#define TX_ON 12
  /*Highlands Ranch test coordinates
   * Lat:  39.5455
   * Long: -105.0103
   */
  /*Launch site coordinates
   * Lat:  38.532141
   * Long: -104.907401
   */
#define DST_LAT 39.5455
#define DST_LNG -105.0103

/* This example shows a basic framework for how you might
   use course and distance to guide a person (or a drone)
   to a destination.  This destination is the DST Tower.
   Change it as required.

   The easiest way to get the lat/long coordinate is to
   right-click the destination in Google Maps (maps.google.com),
   and choose "What's here?".  This puts the exact values in the
   search box.
*/
#define BUFFERSIZE 255
char buff[BUFFERSIZE];    // Incoming data buffer

SFE_BMP180 pressure; // You will need to create an SFE_BMP180 object, here called "pressure":
double baseline; // baseline pressure
////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
  //Highlands ranch declination : -8.367
  //Fort Carson declination :     -8.167
#define DECLINATION -8.167 // Declination (degrees).
int LDR_Pin = 0; //analog pin 0 //AAA collide with radioshield?

boolean imudata = false,GPSdata = false,barometerdata=false;

//state of the code
int codeState;

//Pathfinding points & other variables related to pathfinding
float points[56]; //the array containing all
int goingToPoint;
int destinationXY[2];
float a;
float b;
float o;
int scalar;

#define radToDegrees (180/3.14159)

void setup()
{
  codeState=0;
  pinMode(TX_ON, OUTPUT);
  PitchServo.attach(5);  // attaches the servo on pin 5 to the servo object
  YawServo.attach(6);  // attaches the servo on pin 6 to the servo object
  Serial.begin(ConsoleBaud);  // RadioSheild runs at 4800 baud using Ardiuno 0 & 1
  venus.begin(GPSBaud);
  XBee.begin(XbeeBaud);
  int status = pressure.begin();
  if (status == 0)
  { //AAA barometer failed to start
    Serial.println("Barometer failed to start\nResetting...\n");
    arduinoReset();
  }
  strcpy(buff, "000000z0000.00N/0000.00Wg/A=000000");
  Serial.print("MAB0BX-11\r\n");
  delay(10);
  Serial.print("PAPRS,RELAY*,WIDE\r\n");
  delay(10);
  Serial.print("L200");
  delay(10);
  pinMode(TX_ON, OUTPUT);
  delay(30);

  baseline = getPressure();  // Get the baseline pressure:
  Serial.print("Baseline pressure: ");
  Serial.println(baseline);
  //Serial.println(" mb");

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  { //AAA imu failed to start
    Serial.println("IMU failed to start\nResetting...\n");
    arduinoReset();

    //Issue with reset we can't keep track of the value of times we reset
  }
  a=0.8;
  b=0.5+a/4;
  goingToPoint=0;
  scalar=5280;
}

void loop()
{
  switch(codeState)
  {
    //Everthing a-ok
    case 0: //detect launch
      
      break;
    case 1: //detect eject
      
      break;
    case 2: //detect glide
      
      break;
    case 3: //detect wind direction
      
      break;
    case 4: //begin flight
      
      break;
    case 5: //end of flight
      
      break;
    case 6: //landed on ground
      
      break;
    }
  
  
  
  
  
  
  
  int LDRReading = analogRead(LDR_Pin);
  //  Serial.println(LDRReading);
  if (LDRReading > 512) {
    // Serial.println("Dark"
  } //AAA end of second case
  
  double alt = 0, P;
  P = getPressure();  // Get a new pressure reading:
  alt = pressure.altitude(P, baseline);
  
  printSensorStatus(alt);
  
  
  
  
  // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (venus.available() > 0)
    gps.encode(venus.read());

  // Every 5 seconds, do an update.
  if (millis() - lastUpdateTime >= 5000)
  {
    lastUpdateTime = millis();
    //   Serial.println();

    // Establish our current status
    double distanceToDestination = TinyGPSPlus::distanceBetween(
                                     gps.location.lat(), gps.location.lng(), DST_LAT, DST_LNG);
    double courseToDestination = TinyGPSPlus::courseTo(
                                   gps.location.lat(), gps.location.lng(), DST_LAT, DST_LNG);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

    // debug //AAA not needed on final
    /*
      Serial.print("DEBUG: Course2Dest: ");
      Serial.print(courseToDestination);
      Serial.print("  CurCourse: ");
      Serial.print(gps.course.deg());
      Serial.print("  Dir2Dest: ");
      Serial.print(directionToDestination);
      Serial.print("  RelCourse: ");
      Serial.print(courseChangeNeeded);
      Serial.print("  CurSpd: ");
      Serial.println(gps.speed.kmph());
        if (gps.location.isUpdated() || gps.altitude.isUpdated())
        {
          Serial.print("Location: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print(",");
          Serial.print(gps.location.lng(), 6);
          Serial.print("  Altitude: ");
          Serial.println(gps.altitude.meters());
        }

      // Within 20 meters of destination?  We're here!
      if (distanceToDestination <= 20.0)
      {
      Serial.println("CONGRATULATIONS: You've arrived!");
      exit(1);
      }
    */
    Transmit();
    /* /AAA not needed, may want distance though
      Serial.print("DISTANCE: ");
      Serial.print(distanceToDestination);
      Serial.println(" meters to go.");
      Serial.print("INSTRUCTION: ");

      // Standing still? Just indicate which direction to go.
      if (gps.speed.kmph() < 2.0)
      {
      Serial.print("Head ");
      Serial.print(directionToDestination);
      Serial.println(".");
      return;
      }

      if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
      Serial.println("Keep on straight ahead!");
      else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345)
      Serial.println("Veer slightly to the left.");
      else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)
      Serial.println("Veer slightly to the right.");
      else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)
      Serial.println("Turn to the left.");
      else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)
      Serial.println("Turn to the right.");
      else
      Serial.println("Turn completely around.");
    */
  }
  int Pval = map(Pval, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  PitchServo.write(Pval);                  // sets the servo position according to the scaled value
  int Yval = map(Yval, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  YawServo.write(Yval);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}

//void setRollValue

void detectLaunch(char fails)
{
  switch(fails)
  {
    case 'b': //barometer failed
      
      break;
    case 'i': //imu failed
      
      break;
    default: //everything worked
      
      break;
  }
}

void detectEject()
{
  
}

void detectGlide(char fails)
{
  switch(fails)
  {
    case 'c': //everything failed
      
      break;
    case 'i': //imu failed
      
      break;
    default: //everything worked
      
      break;
  }
}

void detectWindDirection(char fails) //if not able to be find wind, imu down, sets expected wind direction
{
  switch(fails)
  {
    case 'c': //everthing failed
    case 'i': //imu failed
      o=0; //AAA expected wind direction
      break;
    default: //everything worked
      //narcoleptic sleep?/sleep 
      //o=gyro angle (may be backwards?)
      break;
  }
}

void beginFligh(char fails)
{
  switch(fails)
  {
    case 'c': //everything failed
      
      break;
    case 'b': //barometer failed
      
      break;
    case 'i': //imu failed
      
      break;
    case 'g': //gps failed
      
      break;
    default: //everything worked
      
      break;
  }
}

void endFlight(char fails)
{
  switch(fails)
  {
    case 'c': //everything failed
      
      break;
    case 'b': //barometer failed
      
      break;
    case 'i': //imu failed
      
      break;
    case 'g': //gps failed
      
      break;
    default: //everything worked
      
      break;
  }
}

void landed() //transmit data, keep servos from moving
{
  
}

void printSensorStatus(double alt)
{
    //AAA necessity of these during final flight?
  //print IMU data
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"

    //AAA may be good to print during test, not needed during flight if too many items are being printed
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  
  //Print the altitude
  Serial.print("Relative altitude: ");
  if (alt >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(alt, 3);
  Serial.println(" meters");
  
  //add space under to differentiate between cycles
  Serial.println();
  delay(PRINT_SPEED);
}

//collect data, in different scenerios
void collect_data(){
  


  
}


void Transmit()
{
  float latit, longit;
  digitalWrite(TX_ON, HIGH);
  delay(30);
  //print call sign
  Serial.print("AB0BX-11:>APRS:@");
  //print time
  Serial.print(gps.time.hour());
  Serial.print(gps.time.minute());
  Serial.print(gps.time.second());
  Serial.print("h");
  //print location
  latit = gps.location.lat();
  if (latit > 0) {
    Serial.print(gps.location.lat(), 6);
    Serial.print("N");
  } else {
    Serial.print(-gps.location.lat(), 6);
    Serial.print("S");
  }
  Serial.print("/");
  longit = gps.location.lng();
  if (longit > 0) {
    Serial.print(gps.location.lng(), 6);
    Serial.print("W");
  } else {
    Serial.print(-gps.location.lng(), 6);
    Serial.print("E");
  }
  Serial.print("g");
  //print other gps data
  Serial.print(gps.course.deg());
  Serial.print("/");
  Serial.print(gps.speed.mps());
  Serial.print("/A=");
  Serial.println(gps.altitude.meters());
  Serial.print("\r\n");  // send CR/LF transmit packet
  delay(5000);
  digitalWrite(TX_ON, LOW);
  delay(5000);
}

void xbee()
{
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    XBee.write(buff);
  }
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.write(XBee.read());
  }
}

double getPressure()
{
  char status;
  double T, P, a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    //    Serial.print("Temperature is ");
    //   Serial.print(T, 2);
    //    Serial.println(" degrees C");
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
      }
    }
  }
}

void printGyro()
{ //AAA review necessity
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();

  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  //  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  //  Serial.print(imu.calcGyro(imu.gx), 2);
  //  Serial.print(", ");
  //  Serial.print(imu.calcGyro(imu.gy), 2);
  //  Serial.print(", ");
  //  Serial.print(imu.calcGyro(imu.gz), 2);
  //  Serial.println(" deg/s");
#elif defined PRINT_RAW
  //  Serial.print(imu.gx);
  //  Serial.print(", ");
  //  Serial.print(imu.gy);
  //  Serial.print(", ");
  //  Serial.println(imu.gz);
#endif
}

void printAccel()
{ //AAA review necessity
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  //  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  //  Serial.print(imu.calcAccel(imu.ax), 2);
  //  Serial.print(", ");
  //  Serial.print(imu.calcAccel(imu.ay), 2);
  //  Serial.print(", ");
  //  Serial.print(imu.calcAccel(imu.az), 2);
  //  Serial.println(" g");
#elif defined PRINT_RAW
  //  Serial.print(imu.ax);
  //  Serial.print(", ");
  //  Serial.print(imu.ay);
  //  Serial.print(", ");
  //  Serial.println(imu.az);
#endif

}

void printMag()
{ //AAA review necessity
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();

  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  //  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  //  Serial.print(imu.calcMag(imu.mx), 2);
  //  Serial.print(", ");
  //  Serial.print(imu.calcMag(imu.my), 2);
  //  Serial.print(", ");
  //  Serial.print(imu.calcMag(imu.mz), 2);
  //  Serial.println(" gauss");
#elif defined PRINT_RAW
  //  Serial.print(imu.mx);
  //  Serial.print(", ");
  //  Serial.print(imu.my);
  //  Serial.print(", ");
  //  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{ //AAA currently useless
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

void arduinoReset()
{
  asm volatile ("  jmp 0");
}

void createPathfindingPoints()
{  
  int quarterOfPoints=7;
  //create point objects
  //points=new float[quarterOfPoints*8];
  //quarter 0 and 2 move in the same direction, however, 1 and 3 move in the opposite direction
  for(int i=0;i<quarterOfPoints;i++) {
    float c=sqrt(a*a-b*b);
    float x=1;
    float y1=tan((90/(quarterOfPoints-i)-90/quarterOfPoints)/radToDegrees);
    float x_1=(float)((a*b)/sqrt(pow(a,2)*pow(y1,2)+pow(b,2)*pow(x,2))*x); //
    float y_1=(float)((a*b)/sqrt(pow(a,2)*pow(y1,2)+pow(b,2)*pow(x,2))*y1);
    //set 1st and 2nd quarters.
    points[i*2] = (x_1+c)*cos(o)-y_1*sin(o);
    points[i*2+1] = y_1*cos(o)+(x_1+c)*sin(o);
    points[quarterOfPoints*4+i*2] =(-x_1+c)*cos(o)+y_1*sin(o);
    points[quarterOfPoints*4+i*2+1] =-y_1*cos(o)+(-x_1+c)*sin(o);
    float x_2;
    float y_2;
    //shift values over by one
    if(i!=0) {
      float y2=tan((90/(i+0)-90/quarterOfPoints)/radToDegrees);
      x_2=-(float)((a*b)/sqrt(pow(a,2)*pow(y2,2)+pow(b,2)*pow(x,2))*x); //
      y_2=(float)((a*b)/sqrt(pow(a,2)*pow(y2,2)+pow(b,2)*pow(x,2))*y2);
    } else {
      x_2=0; //?
      y_2=b;
    }
    //set 1st and 3rd quarters
    points[quarterOfPoints*2+i*2] = (x_2+c)*cos(o)-y_2*sin(o);
    points[quarterOfPoints*2+i*2+1] = y_2*cos(o)+(x_2+c)*sin(o);
    points[quarterOfPoints*6+i*2] = (-x_2+c)*cos(o)+y_2*sin(o);
    points[quarterOfPoints*6+i*2+1] = -y_2*cos(o)+(-x_2+c)*sin(o);
  }
  if(goingToPoint*2>=56)
    goingToPoint=0;
  destinationXY[0]=points[goingToPoint*2]*scalar;
  destinationXY[1]=points[goingToPoint*2+1]*scalar;
}

/* todo
 *  clean code
 *    review //AAA comments
 *    review printed data and discuss importance
 *  add printAll function
 *  add reset function
 *  reWrite Pval and Yval
 *  add sensor startup failure messages
 *  add flight plans
 *  define the max and the finite ranges
 *  add case structure
 *  light detector
 *    what is light and dark?
 *  define servo movement
 *    add default values
 *  boot system
 *    detect liftoff{fast accel and large change in alt}
 *    is out of rocket{light sensor sees high for good amount of time}
 *    is flying{ fowr accel > side accel && down accel > side accel over time}
 *  
 *  /
 */
