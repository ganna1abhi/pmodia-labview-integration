// By Dr. Arda Gozen and Abhishek Gannarapu (abhishek.gannarapu@wsu.edu)
// Gain factors calculated at different frequencies starting from 2000 Hz to 100 kHz 
// in steps of 2000 Hz
// A 1.5 Mohm Resistor and 10 pF Capacitor (RC) was used as a reference impedance source

#include <WireIA.h>
#include <inttypes.h>
#include <Wire.h>
#include <EEPROM.h>

#include "Wire.h"
#include "Complex.h"

// Defining all the register events
#define button 2
#define SLAVE_ADDR 0x0D
#define ADDR_PTR 0xB0           

#define START_FREQ_R1 0x82
#define START_FREQ_R2 0x83
#define START_FREQ_R3 0x84

#define FREG_INCRE_R1 0x85
#define FREG_INCRE_R2 0x86
#define FREG_INCRE_R3 0x87

#define NUM_INCRE_R1 0x88
#define NUM_INCRE_R2 0x89

#define NUM_SCYCLES_R1 0x8A
#define NUM_SCYCLES_R2 0x8B

#define RE_DATA_R1 0x94
#define RE_DATA_R2 0x95

#define IMG_DATA_R1 0x96
#define IMG_DATA_R2 0x97

#define TEMP_R1 0x92
#define TEMP_R2 0x93

#define CTRL_REG 0x80
#define STATUS_REG 0x8F
#define pi 3.14159265358

int data;
int statuscheck;
const float MCLK = 16.776 * pow(10, 6); // AD5933 Internal Clock Speed 16.776 MHz
const float start_freq = 2 * pow(10, 3); // Set start freq, < 100Khz
const float incre_freq = 2 * pow(10, 3); // Set freq increment
const int incre_num = 50; // Set number of increments; < 511
float re;
float img;
float mag; 
float ph; 
//float gn_mag; 
//float gn_ph; 
float cal_mag; 
float cal_ph;
float dev_freq;
const float re_act = 98.875*pow(10,3); // actual real value of the resistor 
const float img_act = -417.48; // actual img value of the resistor 
const float mag_act = sqrt(pow(double(re_act),2)+pow(double(img_act),2)); // actual magitude
const float ph_act = (180.0/3.1415926)*atan(double(img_act)/double(re_act)); // actual phase angle
//ph_act = (180.0/3.1415926)*ph_act; // angle in degrees 

const float gn_mag[] = {
2.01676933647889e-10, 2.01075896245039e-10, 2.02156749829603e-10, 2.02865126394114e-10, 2.03880200885129e-10,
2.04569424353690e-10, 2.05442615763733e-10, 2.06210063740696e-10, 2.06735177014616e-10, 2.06728935086957e-10,
2.07422575115978e-10, 2.07872485518808e-10, 2.08291032237997e-10, 2.09156621954237e-10, 2.09512309596226e-10,
2.10435572286198e-10, 2.10985801047195e-10, 2.11625778859015e-10, 2.11922193697383e-10, 2.11862711483692e-10,
2.12767926798854e-10, 2.13680470743593e-10, 2.14425516345066e-10, 2.14932132756597e-10, 2.15692163638429e-10,
2.16889636729696e-10, 2.17838930172185e-10, 2.19850520559645e-10, 2.23181823975115e-10, 2.26510167932770e-10,
2.31135767962348e-10, 2.35766382974399e-10, 2.40777746794554e-10, 2.45906454664768e-10, 2.51148149301368e-10,
2.56529139594743e-10, 2.61856135586265e-10, 2.67507186793794e-10, 2.72900258632719e-10, 2.78355005904209e-10,
2.84335612005172e-10, 2.90287089194992e-10, 2.96142256708266e-10, 3.02062991022894e-10, 3.07890332582480e-10,
3.13632546133231e-10, 3.19401388529402e-10, 3.25065730924158e-10, 3.30586692957278e-10, 3.36109573695980e-10
};

const float gn_ph[] = {
-1.51778679847932, -1.50070096785381, -1.45146628794921, -1.40782279138970, -1.36454353180326, -1.32494953251332,
-1.28197906980682, -1.24161064427691, -1.20165116076679, -1.16370221104284, -1.12249524394826, -1.08293005578720,
-1.04267862769066, -1.00282442808124, -0.962877762938166, -0.922429028437587, -0.882403104207715, -0.842678065256900,
-0.803381427181115, -0.764265685978069, -0.724877414735710, -0.685094432473163, -0.645422905219333, -0.605818769314204,
-0.566622498529320, -0.525738167518921, -0.484800653020244, -0.441475913506845, -0.402438287180483, -0.363639408863913,
-0.324354385030797, -0.284660914312864, -0.244119542316058, -0.204067286929258, -0.163181157071979, -0.123067163492273,
-0.0830839545697892, -0.0439266971456112, -0.00432889396542535, 0.0349102808102506, 0.0742409322479309,
0.113586654213806, 0.152573280032845, 0.192048198088574, 0.231095099786608, 0.270856077191696, 0.309908909139402,
0.349068438320551, 0.388200906243044, 0.428420633017557
};

float mag_un; 
float ph_un; 
float re_un;
float img_un;
int cnt = 0; 
Complex temp(0,0);
Complex c_dev(0,0); 
Complex c_sht(0,0);

IA testIA;
void setup() {
  Wire.begin(9);
  Serial.begin(9600);
 // Serial.println("came here");
  writeData(CTRL_REG, 0x06);
  delay(1000);
}

void loop() {
  delay(10);
  writeData(START_FREQ_R1, getFrequency(start_freq, 1));
  writeData(START_FREQ_R2, getFrequency(start_freq, 2));  
  writeData(START_FREQ_R3, getFrequency(start_freq, 3));
  delay(10);
  // Increment by 2 kHz
  writeData(FREG_INCRE_R1, getFrequency(incre_freq, 1));
  writeData(FREG_INCRE_R2, getFrequency(incre_freq, 2));
  writeData(FREG_INCRE_R3, getFrequency(incre_freq, 3));
  delay(10);
  // Points in frequency sweep (100), max 511
  writeData(NUM_INCRE_R1, (incre_num & 0x001F00) >> 0x08 );
  writeData(NUM_INCRE_R2, (incre_num & 0x0000FF));
  delay(10);
  //Serial.println(readData(NUM_SCYCLES_R2));
  delay(100);
  // Setting device to Standby
  writeData(CTRL_REG, 0xB0);
  delay(10);
  writeData(CTRL_REG, 0x10);
  delay(10);
  writeData(NUM_SCYCLES_R1, 511);
  delay(10);
  writeData(NUM_SCYCLES_R2, 0xFF);
  delay(10);
  //  //status checking using polling - D2 needs to be high when done

  //  //Start the frequency sweep
  writeData(CTRL_REG, 0x20);
  int swp=1;
  while (swp==1 && cnt < 50) {
    byte scheck = readData(0x8F);
    while (scheck != 0b00000010) {    //Status check to see if device is ready
      
      //Serial.println(scheck, BIN);
      scheck = byte(readData(0x8F)) & 0b00000010;
    }
      //delay(2000);
      byte R1 = readData(RE_DATA_R1);
      byte R2 = readData(RE_DATA_R2);
      re = (R1 << 8) | R2;
      //Serial.print(re);
      if (re > pow(2,15)) {
        re = re - pow(2,16); 
      }
     // Serial.print(PI);
      R1  = readData(IMG_DATA_R1);
      R2  = readData(IMG_DATA_R2);
      img = (R1 << 8) | R2;
      //Serial.println(img);
      if (img > pow(2,15)) {
        img = img - pow(2,16);
      }
      mag = sqrt(pow(double(re),2)+pow(double(img),2));
      
      // four quadrant phase angle check 
      
      if ( re > 0 && img > 0){
        ph = atan(double(img)/double(re));
      }
      else if (re < 0 && img > 0){
        ph = pi + atan(double(img)/double(re));
      }
      else if (re < 0 && img < 0){
        ph = pi + atan(double(img)/double(re));
      }
      else{
        ph = 2*pi + atan(double(img)/double(re));
      }
            
      mag_un = 1/(gn_mag[cnt]*mag); 
      ph_un = ph - gn_ph[cnt]; 
      re_un = mag_un*cos(ph_un); 
      img_un = -mag_un*sin(ph_un);

      //Serial.println(ph); 
     // Serial.print(re); Serial.print("\t"); Serial.println(img);
      Serial.print(re_un); Serial.print("\t"); Serial.println(img_un);
      if (byte(readData(0x8F)) & 0b00000100) {
        swp=0;
        }
      else{
        writeData(CTRL_REG, 0x30);
       // Serial.print("actual mag: ");Serial.println(dev_freq);
        delay(1000);
        }
        
      cnt = cnt + 1; 
    }
  Serial.println("----------------");
  cnt = 0; 
  delay(100);
  delay(10);
//  //Power down
  writeData(CTRL_REG,0xA0);
}

void writeData(int addr, int data) {

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
  delay(1);
}


int readData(int addr) {
  int data;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(ADDR_PTR);
  Wire.write(addr);
  Wire.endTransmission();

  delay(1);

  Wire.requestFrom(SLAVE_ADDR, 1);

  if (Wire.available() >= 1) {
    data = Wire.read();
  }
  else {
    data = -1;
  }
  delay(1);
  return data;
}

byte getFrequency(float freq, int n) {
  long val = long((freq / (MCLK / 4)) * pow(2, 27));
  byte code;

  switch (n) {
    case 1:
      code = (val & 0xFF0000) >> 0x10;
      break;

    case 2:
      code = (val & 0x00FF00) >> 0x08;
      break;

    case 3:
      code = (val & 0x0000FF);
      break;

    default:
      code = 0;
  }

  return code;
}
