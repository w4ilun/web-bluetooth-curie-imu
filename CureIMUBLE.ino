#include <CurieBLE.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

BLEPeripheral blePeripheral;
BLEService imuService("BEEF"); //give your custom service an ID
BLECharacteristic imuChar("FEED", BLERead | BLENotify, 20); //give your custom characteristic an ID, properties, and length

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup(){
    Serial.begin(9600);

    // instantiate BLE peripheral
    blePeripheral.setLocalName("CurieIMU"); //broadcast device name
    blePeripheral.setAdvertisedServiceUuid(imuService.uuid());  // add the service UUID
    blePeripheral.addAttribute(imuService);   // add your custom service
    blePeripheral.addAttribute(imuChar); // add your custom characteristic
    blePeripheral.begin();
    Serial.println("Bluetooth device active, waiting for connections...");

    // start the IMU and filter
    CurieIMU.begin();
    CurieIMU.setGyroRate(25);
    CurieIMU.setAccelerometerRate(25);
    filter.begin(25);

    // set the accelerometer range to 2G
    CurieIMU.setAccelerometerRange(2);
    // set the gyroscope range to 250 degrees/second
    CurieIMU.setGyroRange(250);

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
}

void loop() {
    int aix, aiy, aiz;
    int gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, heading;
    unsigned long microsNow;
    String str = "";

    // check if it's time to read data and update the filter
    microsNow = micros();
    if(microsNow - microsPrevious >= microsPerReading){

        // read raw data from CurieIMU
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

        // convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);

        // update the filter, which computes orientation
        filter.updateIMU(gx, gy, gz, ax, ay, az);

        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();

        str = str + roll + "," + pitch + "," + heading;
        Serial.println(str);

        BLECentral central = blePeripheral.central();
        if(central){ // if a central is connected to peripheral
            const unsigned char imuCharArray[20] = {
                str[0],str[1],str[2],str[3],str[4],
                str[5],str[6],str[7],str[8],str[9],
                str[10],str[11],str[12],str[13],str[14],
                str[15],str[16],str[17],str[18],str[19]
            };
            imuChar.setValue(imuCharArray, 20); //notify central with new data
        }

        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
    }
}

float convertRawAcceleration(int aRaw){
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw){
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}