
#include "Keypad_Matrix.h"
#include "Keyboard.h"
#include "Mouse.h"
#include "ADNS2083.h"
#include "Encoder.h"

#define ENCODER_DO_NOT_USE_INTERRUPTS
#define KEY_SPACE 0x20
#define SCLK 4                            //マウスセンサーピン
#define SDIO 3                            //
#define _Config 0x06

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
Encoder myEnc(0,2);    //ロータリーエンコーダーに使う端子。

const byte ROWS = 5;    //キーマトリクスの端子数
const byte COLS = 5;

//ボタン設定
/*******************************************************
                                5way_sw
                  w                6          L1           R1                      
                  ↑               ↑          [21]          [23]                           
              a← ● →d      9← (10) →7      L2            R2                                
joystick_button _/↓               ↓          [22]          [24]                           
                  s                8                                                                  
                          
              [11]        [15]         [16]         back_L    back_R                                            
         [14]      [12]          [19]       [17]      [5]      [25]                                             
              [13]        [20]         [18]                                                                     
                                                                                                           
                    
    {joystick_button  ,9   ,17  ,15  ,21}
    {*  ,10  ,16  ,14  ,22}
    {*  ,7   ,19  ,11  ,23}
    {*  ,8   ,20  ,12  ,24}
    {5  ,6   ,18  ,13  ,25}
*********************************************************/

const char keys[ROWS][COLS] = {
 
  {KEY_LEFT_CTRL,'c','2',MOUSE_MIDDLE,'q'},
  {'0',KEY_ESC,'1','r',KEY_SPACE},
  {'0', 'j','4','t','e'},
  {'0', 'b','5','z',MOUSE_LEFT},
  {MOUSE_RIGHT, 'm', '3','x','f'},

};

const byte rowPins[ROWS] = {10, 16, 14,15,18}; //ダイオード側connect to the row pinouts of the keypad
const byte colPins[COLS] = {5,6, 7, 8,9}; //connect to the column pinouts of the keypad

  // Create the Keypad
Keypad_Matrix kpd = Keypad_Matrix( makeKeymap (keys), rowPins, colPins, ROWS, COLS );




void setup() 
{ 
  Serial.begin (9600);
  
  Optical1.begin();                       // Resync (not really necessary?)
  writeRegister(_Config,0x02);     //CPI設定  800 0 / 1000 0x01 / 1200 0x02 /1600 0x03
  Keyboard.begin();
  Mouse.begin();

  kpd.begin ();
  kpd.setKeyDownHandler (keyDown);
  kpd.setKeyUpHandler   (keyUp);
  

  pinMode(1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(1),CPI,FALLING);

 delay(100);
}


void loop()
{
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
           if(cpi==0)writeRegister(_Config,0x00);
           if(cpi==1)writeRegister(_Config,0x01);
           if(cpi==2)writeRegister(_Config,0x02);
           if(cpi==3)writeRegister(_Config,0x03);           
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
if(which != MOUSE_LEFT && which != MOUSE_RIGHT && which != MOUSE_MIDDLE){
 
    Keyboard.press(which);
}  
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

  if(which != MOUSE_LEFT && which != MOUSE_RIGHT && which != MOUSE_MIDDLE){
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
  void gaia(void){
     Keyboard.write('e');
     delay(2000);
    Mouse.click(MOUSE_RIGHT);
     delay(3000);
     delay(3000);
      
    }


void CPI(void){
    
if (digitalRead(1) == LOW && cpi_set == 0){
if(cpi==0){
 Serial.print ("CPI-800");
  }     
 else if(cpi==1){
 Serial.print ("CPI-1000");
  }
  else if(cpi==2){
 Serial.print ("CPI-1200");
  }
  else if(cpi==3){
 Serial.print ("CPI-1600");
  }  
 }
cpi_set = 1;
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
