//----------------------------------------------------------------------------------------------
//Board Library : Seeed nRF52 mbed-enable Borads 2.9.1
//Board Select  : Seeed nRF52 mbed-enable Borads / Seeed XIAO BLE - nRF52840
//----------------------------------------------------------------------------------------------
//note: To work around a bug in analogRead() that reads the battery voltage, the patch in the link below is required
//https://forum.seeedstudio.com/t/xiao-ble-sense-mbed-2-7-2-battery-charge-and-voltage-monitor-analogread-p0-31-does-not-work/266438/31
//**********************************************************************************************    
//2023/04/19

#include <Wire.h>
// #include <ArduinoBLE.h>
#include <MAX30105.h>

//definition of pins and constants
// #define HICHG       P0_02    //charging current　HIGH(Open):50mA LOW:100mA
// #define CHG         P0_03    //charge indicator Discharge:LED OFF　Charge:LED ON
// #define VBAT_ENABLE P0_28    //battery voltage readable　HIGH(Open):disable　LOW:enable
// #define VBAT_READ   P0_31    //battery voltage ADC input
// #define VBAT_LOWER    3.5    //battery voltage lower limit
#define LOOPTIMER      30    //loop timer mS(min30)
#define LOOP_COUNT      8    //send data every LOOP_COUNT times, number of bytes of waveform data
#define dataNum   6+LOOP_COUNT  //bytes of data : hrValue 2, spo2Value 2, Vbatt 2, waveformData[LOOP_COUNT] (6+LOOP_COUNT bytes)

//Definition of MAX30105 constants
#define MASK_MIN      300    //saturation value of peak detection mask period
#define MASK_MAX     1000
#define MASK_DETECT  0.62    //section without peak detection aveINTERVAL * MASK_DETECT
#define N_irAve         4    //moving average number of ir sensor value
#define N_redAve        4    //moving average number of red sensor value
#define N_hrAve         4    //moving average number of heart rate
#define N_spo2Ave       4    //moving average number of SpO2
#define N_intervalAve  16    //moving average number of peak-to-peak intervals

//variable definitions
int32_t previous_time = millis();     //previous peak detection time
uint32_t measureTime = millis();      //data acquisition time
uint32_t irValue;                     //(moving averaged) ir sensor data
uint32_t redValue;                    //(moving averaged) red sensor data
int32_t previous_irValue = 0;         //previous irValue
int32_t previous_redValue = 0;        //previous redValue
int32_t previous_diff = 0;            //previous difference
int32_t pulseInterval = 0;            //peak interval
int32_t previous_pulseInterval = 0;   //previous peak interval
int32_t min_irValue = 9999999;        //max and min irValue during one pulse section
int32_t max_irValue = 0;
int32_t min_redValue = 9999999;       //max and min redValue during one pulse section
int32_t max_redValue = 0;
int32_t irValueDC;                    //DC component of irValue
int32_t irValueDCmin;                 //min DC component of irValue
int32_t redValueDC;                   //DC component of redValue
int32_t irValueAC;                    //AC component of irValue
int32_t redValueAC;                   //AC component of redValue
double spo2Value;                     //SpO2　saturation oxygen level
double hrValue;                       //heart rate
int32_t intervalMask;                 //period to detect peak
uint32_t irBuff[N_irAve + 1];         //buffer for moving average function
uint32_t redBuff[N_redAve + 1];       
int avgHR[N_hrAve + 1];               
int avgSPO2[N_spo2Ave + 1];           
int avgINTERVAL[N_intervalAve + 1];   
uint32_t startTime;                   //loop start time
float Vbatt;                          //battery voltage
union unionData {                     //for data type conversion
  uint16_t  dataBuff16[dataNum/2];
  uint8_t   dataBuff8[dataNum];
};
union unionData ud;
bool LED_OnOff;                       //for LED blinking
int m = 0;                            //for loop count

//instances of the class
MAX30105 MAX30102;      //[Important Note] The names of sensors are reversed in hardware and library

//custom Services and Characteristics EXAMPLE
// #define myUUID(val) ("3917ab4f-" val "-47c5-b442-be7df6e1616f")
// BLEService        poxService(myUUID("0000"));
// BLECharacteristic dataCharacteristic(myUUID("0010"), BLERead | BLENotify, dataNum);

#define AR_INTERNAL2V4 3
//************************************************************************************************************ setup()
void setup()
{     
  //Serial port initialization
  Serial.begin(9600);
  delay(1000);

  //IO pin settings
  pinMode(LED_RED, OUTPUT);         //writing data:ON  battery voltage low:ON
  pinMode(LED_GREEN, OUTPUT);       //connecting to Central:BLINK
  pinMode(LED_BLUE, OUTPUT);        //error status:BLINK
  // pinMode(HICHG, OUTPUT);
  // pinMode(CHG, INPUT);
  // pinMode(VBAT_ENABLE, OUTPUT);
  // pinMode(VBAT_READ, INPUT);

  digitalWrite(LED_RED, HIGH);      //ON:LOW OFF:HIGH
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  // digitalWrite(HICHG, LOW);         //charging current　100mA
  // digitalWrite(VBAT_ENABLE, LOW);   //battery voltage read enable

  //initialization of ADC
  analogReference((eAnalogReference)3);  //Vref=2.4V
  // analogReference(2.4);
  // analogAcquisitionTime(AT_10_US);  //default 10uS  3,5,10,15,20,40
  // analogAcquisitionTime(10);
  analogReadResolution(12);         //default 10 bit   
  
  //initialization of MAX30102
  Serial.println("Initialization of MAX30102");
  while (!MAX30102.begin(Wire, 400000UL)) {;} //set 400kHz
  byte ledBrightness = 30;  //0=0mA to 255=50mA
  byte sampleAverage = 4;   //1, 2, 4, 8, 16, 32 (when sampleRate=400, peak detection fails if less than 4)
  byte ledMode = 2;         //1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400;     //50, 100, 200, 400, 800, 1000, 1600, 3200 (unstable above 800)
  int pulseWidth = 411;     //69, 118, 215, 411 (any setting 69~411 is acceptable)
  int adcRange = 4096;      //2048, 4096, 8192, 16384 (if set 2048, may be overscale)
  
  MAX30102.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  MAX30102.enableAFULL();           //FIFO interrupt enabled  
  MAX30102.setFIFOAlmostFull(3);    //interrupt occurs at 32-3=29
  MAX30102.enableFIFORollover();    //when the FIFO is full, it continues to fill with new data
  
  //getting initial values [Important Note] the names of sensors are reversed in hardware and library
  previous_irValue =  MAX30102.getRed();
  previous_time = millis();

  //initialization and configuration of BLE classes
  // if (!BLE.begin())
  // {
  //   Serial.println("starting BLE module failed!");
  //   errorBlink(4);                                  //[4] BLE initialization error
  // }

  // //set the local name peripheral advertises
  // BLE.setLocalName("myPeripheral_2");  // <----------------------------------------------
  // //set the device name peripheral advertises  (Default "Arduino")
  // BLE.setDeviceName("XIAO nRF52840");
  // //set the UUID for the service this peripheral advertises
  // BLE.setAdvertisedService(poxService);
  // //add the characteristic to the service  
  // poxService.addCharacteristic(dataCharacteristic);
  // //add service  
  // BLE.addService(poxService);

  // //start advertising
  // BLE.setAdvertisingInterval(160);    //0.625mS*160=100mS
  // BLE.setConnectionInterval(6, 3200); //1.25mS*6=7.5mS, 1.25mS*3200=4sec
  // BLE.advertise();

  Serial.println("End of initialization");
}

//**************************************************************************************************** loop() start
void loop()
{ 
  //connect to Central
  // Serial.println("Connecting to Central ........");
  // BLEDevice central = BLE.central();

  // if (central) {    //connected to Central?  
  if(1) {
    Serial.println("Connected to Central");
    // while(central.connected()) {  
    while(1) {   
      LED_OnOff = !LED_OnOff;                           //connection indicator BLINK
      digitalWrite(LED_GREEN, (LED_OnOff ? LOW: HIGH));
    
      //detect peak to determine heart rate and SpO2
      startTime = millis();
  
      //acquisition of data from sensors 
      while(MAX30102.available() == 0) {
        MAX30102.check();   //if there is new data, write it in sense structure and update head and tail
      }                      
      //[Important Note] the names of sensors are reversed in hardware and library
      irValue = movingAveL(MAX30102.getFIFORed(), irBuff, N_irAve);    //moving average ir data
      redValue = movingAveL(MAX30102.getFIFOIR(), redBuff, N_redAve);  //moving average red data
    
      MAX30102.nextSample();    //update FIFO tail

      //data acquisition time
      measureTime = millis();
  
      //update max and min values to find AC components
      if (irValue < min_irValue) min_irValue = irValue;
      if (irValue > max_irValue) max_irValue = irValue;
      if (redValue < min_redValue) min_redValue = redValue;
      if (redValue > max_redValue) max_redValue = redValue;

      //pulse indicator　parameters have been adjusted for ease of viewing
      float dispData = 0.125 + 0.75 * ((float)irValue - (float)irValueDCmin) / (float)irValueAC; 
      byte pwmData = map(255 * dispData, 0, 255, 255, 0);
      analogWrite(LED_RED, pwmData);

      //Mask value by multiplying the moving average of peak intervals by a factor
      //peak detection range is limited to other than MASK_MIN to MASK_MAX to avoid false detection
      int tempI = movingAveI((int)previous_pulseInterval, avgINTERVAL, N_intervalAve);
      intervalMask = constrain(tempI, MASK_MIN, MASK_MAX) * MASK_DETECT;

      //peak is defined as the point where the diff(the change in irValue) goes from positive to negative
      //detect peaks from the time past the mask period
      int32_t diff = irValue - previous_irValue;
      if ( (previous_diff > 0) && (diff <= 0) && ((measureTime - previous_time) > intervalMask) ) {
        //obtain the AC and DC components of irValue and redValue
        //the DC component adopts the value at the peak
        irValueAC = max_irValue - min_irValue;
        redValueAC = max_redValue - min_redValue;
        irValueDC = max_irValue;
        redValueDC = max_redValue;
        irValueDCmin = min_irValue;

        //calculate peak interval
        pulseInterval = millis() - previous_time;
        previous_time = millis();

        //calculate heart rate per minute from peak interval
        //multiply by 16 and divide by 16 to retain decimals
        hrValue =  (double)movingAveI(60000 * 16 / pulseInterval, avgHR, N_hrAve) / 16.0;
    
        //R = (AC_RED / DC_RED) / (AC_IR / DC_IR)
        double red_div = double(redValueAC) / double(redValueDC);
        double ir_div = double(irValueAC) / double(irValueDC);
        double R = red_div / ir_div;

        //calculate SpO2   spo2 = -45.060*R^2 + 30.354*R + 94.845 (spo2_algorithm.cpp)
        //multiply by 16 and divide by 16 to retain decimals
        spo2Value = (double)movingAveI((-45.060 * R * R + 30.354 * R + 94.845) * 16.0, avgSPO2, N_spo2Ave) / 16.0;
    
        //initialization for next max and min detection
        min_irValue = 9999999; max_irValue = 0;
        min_redValue = 9999999; max_redValue = 0;   
      }
        
      //previous value for peak detection
      previous_irValue = irValue; previous_diff = diff;
      previous_pulseInterval = pulseInterval; 

      //averages the battery voltage 32 times
      // int Vadc = 0;
      // for (int i = 0; i < 32; i++) {
      //   Vadc = Vadc + analogRead(VBAT_READ);
      // }
      // Vbatt = 2.961 * 2.4 * (Vadc / 32) / 4096 * 1.0196;   //sample:correction value=1.0196, Vref=2.4, attenuation ratio=2.961
      // ud.dataBuff16[2] = Vbatt * 1000;
      // if(Vbatt < VBAT_LOWER) {          //low battery voltage warning
      //   digitalWrite(LED_RED, LOW);     //red LED ON
      // }

      //preparing data for transmission
      ud.dataBuff16[0] = hrValue * 100;
      ud.dataBuff16[1] = spo2Value * 100;
      ud.dataBuff16[2] = Vbatt * 1000; 
      ud.dataBuff8[m + 6] = uint8_t(100 * dispData);
      Serial.print(m); Serial.print(" "); Serial.println(ud.dataBuff8[m + 6]);

      //write to Characteristic every LOOP_COUNT times to reduce traffic
      // m++;
      // if(m >= LOOP_COUNT) {      
      //   digitalWrite(LED_BLUE, LOW);   
      //   dataCharacteristic.writeValue(ud.dataBuff8, dataNum);
      //   digitalWrite(LED_BLUE, HIGH);
      //   m = 0;
      // }

      //loop time adjustment
      while(millis() - startTime < LOOPTIMER);     
    } //while connected

    //once disconnected from Central, from the beginning of the loop
    digitalWrite(LED_GREEN, HIGH);
    // Serial.print("Disconnected from central: ");
    // Serial.println(central.address());
  } //if connect    
} //loop()



//************************************************************************************************ movingAveI()
// moving average(int)　n=interval+1 sum=buff[0]
int movingAveI(int indata, int* buff, int n) {
  for(int i = n; i >=2; i--) {
    buff[i] = buff[i-1];
  }
  buff[1] = indata;
  buff[0] = 0;
  for(int i = 1; i <= n; i++) {
    buff[0] = buff[0] + buff[i];
  }
  int ave = buff[0] / n;
  return(ave); 
}

//************************************************************************************************** movingAveL()
// moving average(uint32_t)　n=interval+1 sum=buff[0]
uint32_t movingAveL(uint32_t indata, uint32_t* buff, int n) {
  for(int i = n; i >=2; i--) {
    buff[i] = buff[i-1];
  }
  buff[1] = indata;
  buff[0] = 0;
  for(int i = 1; i <= n; i++) {
    buff[0] = buff[0] + buff[i];
  }
  uint32_t ave = buff[0] / n;
  return(ave); 
}

//************************************************************************************************** movingAveF()
// moving average(float)　n=interval+1 sum=buff[0]
float movingAveF(float indata, float* buff, int n) {
  for(int i = n; i >=2; i--) {
    buff[i] = buff[i-1];
  }
  buff[1] = indata;
  buff[0] = 0;
  for(int i = 1; i <= n; i++) {
    buff[0] = buff[0] + buff[i];
  }
  float ave = buff[0] / n;
  return(ave); 
}

//************************************************************************************************* errorBlink()
//repeat blinking a specified number of times
void errorBlink(int errorNumber)
{
  while(1) {
    for(int i = 1; i <= errorNumber; i++) {
      digitalWrite(LED_BLUE, LOW);
      delayMicroseconds(300000);
      digitalWrite(LED_BLUE, HIGH);
      delayMicroseconds(300000);
    }
    delayMicroseconds(500000);
  } 
}
