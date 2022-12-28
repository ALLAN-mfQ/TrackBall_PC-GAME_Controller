

#include "Keypad_Matrix.h"
#include "Keyboard.h"
#include "Mouse.h"
#include "ADNS2083.h"
#include "Encoder.h"
#include <Wire.h>

#include <Kalman.h> 
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define ENCODER_DO_NOT_USE_INTERRUPTS
#define KEY_SPACE 0x20
#define SCLK 5                            //マウスセンサーピン
#define SDIO 4                            //
#define _Config 0x06

#define SENS_Y 0.5 //ジャイロ縦軸感度
#define DRIFT_X -1.5 //ドリフト補正
#define DRIFT_Y -3

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

static char on_gyro =0;
static char on_gyro2 =0;
ADNS2083 Optical1 = ADNS2083(SCLK, SDIO);

const int xAxis = A3;         // joystick X axis
const int yAxis = A2;         // joystick Y axis

// parameters for reading the joystick:
int range = 50;               // output range of X or Y movement
int threshold = range / 10;    // resting threshold
int center = range / 2;       // resting position value
static char jbf =0;

volatile static char cpi=2;
volatile static char cpi_set=0;
signed char x = 0;                        
signed char y = 0;                        
static signed char r = 0;
Encoder myEnc(1,0);    //ロータリーエンコーダーに使う端子。

const byte ROWS = 5;    //キーマトリクスの端子数
const byte COLS = 5;

//ボタン設定
/*******************************************************
                                5way_sw
                  w               6           L1            R1                      
                  ↑               ↑          [24]          [22]                           
               a← ● →d        9← (10) →7      L2            R2                                
joystick_button _/↓               ↓          [23]          [21]                           
                  s               8                                                                  
                          
              [11]        [15]         [16]         back_L    back_R                                            
         [14]      [12]          [19]       [17]      [5]      [25]                                             
              [13]        [20]         [18]                                                                     
                                                                                                           
                    
    {joystick_button  ,9   ,17  ,15  ,21}
    {sw3(option) ,10  ,16  ,14  ,22}
    {sw2(option) ,7    ,19  ,11  ,23}
    {sw1(PCIset) ,8    ,20  ,12  ,24}
    {5  ,6   ,18  ,13  ,25}
*********************************************************/
#define _gyro '8' //gyro on/off
#define _CPI '9'  //CPI set
#define _SW3 '6'  //option button sw3
#define _SW2 '7'  //option button sw2

const char keys[ROWS][COLS] = {
 
  {_gyro,'c','2',MOUSE_MIDDLE,MOUSE_LEFT},
  {_SW3,KEY_ESC,'1','r','e'},
  {_SW2, 'j','4','t',KEY_SPACE},
  {_CPI, 'b','5','z','q'},
  {MOUSE_RIGHT, 'm', '3','x','f'},

};

const byte rowPins[ROWS] = {10, 16, 14,15,18}; //ダイオード側connect to the row pinouts of the keypad
const byte colPins[COLS] = {6, 7, 8,9,19}; //connect to the column pinouts of the keypad

  // Create the Keypad
Keypad_Matrix kpd = Keypad_Matrix( makeKeymap (keys), rowPins, colPins, ROWS, COLS );




void setup() 
{ 
  Serial.begin (115200);
  Wire.begin();

 #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
 delay(100);
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();


  Optical1.begin();                       // Resync (not really necessary?)
  writeRegister(_Config,0x02);     //CPI設定  800 0 / 1000 0x01 / 1200 0x02 /1600 0x03
  Keyboard.begin();


  kpd.begin ();
  kpd.setKeyDownHandler (keyDown);
  kpd.setKeyUpHandler   (keyUp);
  

  Mouse.begin();
  
 delay(100);
}


void loop()
{

/* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
 // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX/131); Serial.print("\t");
  Serial.print(gyroY/131); Serial.print("\t");
  Serial.print(gyroZ/131); Serial.print("\t");

  Serial.print("\t");
#endif

if(on_gyro == 1)
Mouse.move((-((gyroZ+gyroX)/2)/131)DRIFT_X, ((-gyroY/131)DRIFT_Y)*SENS_Y,0);

//ホイール　ロータリーエンコーダー
  r = myEnc.read();
 if(r <= -1)
 r= -1;
 else if(r >= 1)
 r= 1;
 myEnc.write(0);
 //
   
  kpd.scan ();  //キーボード
   
//マウス
    x = Optical1.dx();  
    y = Optical1.dy(); 
  //x =~ (Optical1.dx() - 1);                   // 方向を反転する場合
  //y =~ (Optical1.dy() - 1); 
                     
   Optical1.mo();         //Motion_Status読み取り    
   Mouse.move(x, y, r);  




         //CPI変更SW
          if(cpi_set==1){
            delay(500);
            Serial.print ("_:");
           if(cpi==0){writeRegister(_Config,0x00); Serial.print ("CPI-800");}
           if(cpi==1){writeRegister(_Config,0x01); Serial.print ("CPI-1000");}
           if(cpi==2){writeRegister(_Config,0x02); Serial.print ("CPI-1200");}
           if(cpi==3){writeRegister(_Config,0x03); Serial.print ("CPI-1600");}     
            cpi_set=0;
            cpi ++;
            if(cpi > 3) cpi =0;
         }
         //
         
 // read and scale the two axes:

   int xReading = readAxis(A3);
   int yReading = readAxis(A2);
   
          if(xReading > 8){
      Keyboard.press('d');
         }
      
      else if(xReading < -8){
      Keyboard.press('a');
      }
    
      if(yReading > 8){
      Keyboard.press('s');
         }
      else if(yReading < -8){
      Keyboard.press('w');
      }

if(xReading == 0) {
       Keyboard.release('a');
       Keyboard.release('d');
}

if(yReading == 0) {
       Keyboard.release('w');
       Keyboard.release('s');
}


 
 }//main end


//レジスタ書き込み関数
void writeRegister(uint8_t address, uint8_t data)
{
  int i = 7;
  
  // Set MSB high, to indicate write operation:
  address |= 0x80;
  
  // Write the address:
  pinMode (SDIO, OUTPUT);
  for (; i>=0; i--)
  {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, address & (1 << i));
    digitalWrite (SCLK, HIGH);
  }
  
  // Write the data:
  for (i=7; i>=0; i--)
  {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, data & (1 << i));
    digitalWrite (SCLK, HIGH);
  }
}

///////////////////////////////////////
//キー検出関数
void keyDown (const char which)
  {
     Serial.print (F("Key down: "));
     Serial.println (which);
if(which == _CPI) cpi_set =1;

if(which == _gyro){
  if(on_gyro2 == 0) {on_gyro =1; on_gyro2 =1;}
  else if(on_gyro2 == 1) {on_gyro =0; on_gyro2 =0;}
}

if(which != MOUSE_LEFT && which != MOUSE_RIGHT && which != MOUSE_MIDDLE && which !=_CPI && which != _gyro)
  Keyboard.press(which);


  
    if(which == MOUSE_LEFT)
     if (!Mouse.isPressed(MOUSE_LEFT))
    Mouse.press(MOUSE_LEFT);

    if(which == MOUSE_RIGHT)
     if (!Mouse.isPressed(MOUSE_RIGHT))
    Mouse.press(MOUSE_RIGHT);

    if(which == MOUSE_MIDDLE)
     if (!Mouse.isPressed(MOUSE_MIDDLE))
    Mouse.press(MOUSE_MIDDLE);

  }

void keyUp (const char which)
  {
  Serial.print (F("Key up: "));
  Serial.println (which);

  if(which != MOUSE_LEFT && which != MOUSE_RIGHT && which != MOUSE_MIDDLE && which != _CPI && which != _gyro){
  Keyboard.release(which);
  }
  if(which == MOUSE_LEFT)
    if (Mouse.isPressed(MOUSE_LEFT))
    Mouse.release(MOUSE_LEFT);

    if(which == MOUSE_RIGHT)
    if (Mouse.isPressed(MOUSE_RIGHT))
    Mouse.release(MOUSE_RIGHT);

    if(which == MOUSE_MIDDLE)
    if (Mouse.isPressed(MOUSE_MIDDLE))
    Mouse.release(MOUSE_MIDDLE);
  }
  

int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}
