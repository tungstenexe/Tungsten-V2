/********************************************************************************************************
* Nerf Stryfe - Tungsten V2
*
* Description
* program for Nerf Stryfe with single shot, burst mode and full auto. 
* Adjustable ROF, ammo capacity, burst per shot. Ammo counter. Combo mode
*
* created  16 Jun 2017
* modified 12 Sep 2017
* by TungstenEXE
*
* If you find my code useful, do support me by subscribing my YouTube Channel, thanks.
*
* My YouTube Channel Link - Nerf related
* https://www.youtube.com/channel/UCDMDwz4Hf8E0R0khyh40SxQ
* 
* Board used      - Arduino Nano
* Pusher Motor    - MTB Rhino Motor 
* FlyWheel Motors - MTB 180 Hellcat Motor
* ESC used        - Hobbywing Quicrun 60A 2S-3S Waterproof Brushed ESC for 1/10
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Bounce2.h>
#include <Servo.h>

#include <SPI.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Adafruit-GFX library found here :
// https://github.com/adafruit/Adafruit-GFX-Library
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Adafruit_SSD1306 library found here :
// https://github.com/adafruit/Adafruit_SSD1306
// you might need to comment away the line "#define SSD1306_128_32" and uncomment the line
// "#define SSD1306_128_64" so that the display works for OLED screen size 128 by 64
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>

/*///////////////////////////////////////////////////////////////////////////////////////////////////////
 * The following are the setting for the ESC used, for other ESC, just chnage setting according to
 * ESC Spec
 *///////////////////////////////////////////////////////////////////////////////////////////////////////
#define THROTTLE_PUSHER_MIN         1500
#define THROTTLE_PUSHER_MAX         2000
#define THROTTLE_PUSHER_BAKE        1000
#define THROTTLE_PUSHER_RETURN      1700
#define THROTTLE_PUSHER_REDUCE      1600
// End of Pusher ESC setting ///////////////////////////////////////////////////////////////////////////////////
#define THROTTLE_MOTORS_MIN         1500
#define THROTTLE_MOTORS_MAX         2000
#define THROTTLE_MOTORS_BAKE        1000
// End of Motor ESC setting ///////////////////////////////////////////////////////////////////////////////////

// The colors for for my wiring reference only you can use your own color
#define PIN_DARTCOUNT               2    // PIN listening to dart count (Brown)
#define PIN_PUSHERESC               3    // PIN to control Pusher ESC, normally the white wire from ESC (Grey)
#define PIN_OLED_RESET              4    // for OLED
#define PIN_MOTORSESC               5    // PIN to control DC Flywheel Motors ESC, normally the white wire from ESC (Green)
//#define PIN_REV                   6    // PIN listening to rev press event for testing only, unnecessary for implementation

#define PIN_DARTTRIGGER             9    // PIN listening to trigger pull event (Purple)
#define PIN_DARTRESET               10   // PIN listening to reset counter event (Orange)
#define PIN_MODEFIRE                11   // PIN listening to change in mode of fire event (White)
#define PIN_SETTING                 12   // PIN listening to setting limit (Green)

#define PIN_VOLTREAD                A1   // PIN to receive voltage reading from battery (Yellow)
// Note                             A4      are used by the OLED SDA (Blue)
// Note                             A5      are used by the OLED SCL (Yellow)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Of PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AMMO_UPPER_LIMIT            35   // Maxmimum ammo configurable
#define AMMO_LOWER_LIMIT            6    // Minimum  ammo configurable

#define BURST_UPPER_LIMIT           5    // Maxmimum burst configurable
#define BURST_LOWER_LIMIT           2    // Minimum burst configurable

#define MODE_SINGLE                 0    // Integer constant to indicate firing single shot
#define MODE_BURST                  1    // Integer constant to indicate firing burst
#define MODE_AUTO                   2    // Integer constant to indicate firing full auto
#define NUM_OF_MODE                 3    // Number of mode available

#define MODE_ROF_STANDARD           0    // Integer constant to indicate standard rate of fire 
#define MODE_ROF_HIGH               1    // Integer constant to indicate highest rate of fire 
#define NUM_OF_MODE_ROF             2    // Number of ROF available

#define RETURN_DELAY_ONESHOT        15   // For single shot, Increase this if the pusher stop too soon for your setup, 
                                         // decrease otherwise
#define RETURN_DELAY_MULTISHOT      10   // For multi shots, Increase this if the pusher stop too soon for your setup, 
                                         // decrease otherwise
#define REV_UP_DELAY                150  // Increase/decrease this to control the flywheel rev-up time (in milliseconds) 

#define BATTERY_MIN                 10.8 // Minimum voltage of battery for rev to operate
#define BATTERY_MIN_3DIGIT          108  // Minimum voltage of battery for rev to operate
#define BATTERY_MAX_3DIGIT          123  // Maximun 

int     rofLimitArr    []         = {50, 100}; // number are in percentage, 50%, 100%
String  rofLimitStrArr []         = {"     <<<<<<>>>>>>", "<<<<<<<<<<<>>>>>>>>>>>"}; 
int     modeROFSetted             = MODE_ROF_STANDARD;   // track the ROF setted
int     modeROFSelected           = MODE_ROF_STANDARD;   // track the ROF selected

int     ammoLimitArrSize          = 5;                 
int     ammoLimitArr      []      = {6, 12, 15, 18, 22}; // 
int     ammoLimitSelected         = 3;                   // 
int     ammoLimit                 = ammoLimitArr[ammoLimitSelected]; // default set as 18 dart mag
int     burstLimit                = 3;                   // default set as 3 darts per burst

int     modeFire                  = MODE_SINGLE;         // track the mode of fire, Single, Burst or Auto, Single by default
int     dartToBeFire              = 0;                   // track amount of dart(s) to fire when trigger pulled, 0 by default
int     dartLeft                  = ammoLimit;           // track amount of dart in mag, same default value as of ammoLimit
float   currentVoltage            = 99.0;
int     returnDelay               = RETURN_DELAY_ONESHOT;

boolean       batteryLow          = false;       // track battery status
boolean       isFiring            = false;       // track if blaster firing         
boolean       magOut              = false;       // track is there a mag 
boolean       isPusherOut         = false;       // track is the pusher out, if true, pusher need to return
boolean       tungstenV2Mode      = false;       // true when V2 mode is selected
boolean       v2ModeFullAuto      = false;       // true when V2 mode is on and is on full auto firing

unsigned long pusherTimerStart    = 0;           // track how long had the pusher being out
const    long pusherOutThreshold  = 90;          // threshold for deciding should the pusher return module kick in.
                                                 // If the estimated shot per second is 10, it means each shot took around
                                                 // 100 millisecond, which mean pusherOutThreshold should be setted 
                                                 // slightly about 100 millisecond
Servo         pusherESC;
Servo         motorsESC;

Adafruit_SSD1306 display(PIN_OLED_RESET);

// Declare and Instantiate Bounce objects
Bounce btnTrigger       = Bounce(); 
Bounce btnDartCount     = Bounce(); 
Bounce btnModeFire      = Bounce();
Bounce btnDartReset     = Bounce();
Bounce btnSetting       = Bounce();
//Bounce btnRev           = Bounce(); // for testing only, unnecessary for implementation

void setup() { // initilze  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // INPUT PINs setup
  // Note: Most input pins will be using internal pull-up resistor. A fall in signal indicate button pressed.
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  
  pinMode(PIN_DARTCOUNT, INPUT_PULLUP);   // PULLUP
  btnDartCount.attach(PIN_DARTCOUNT);
  btnDartCount.interval(5);

  /*// For testing only, unnecessary for implementation
  pinMode(PIN_REV,INPUT_PULLUP);          // PULLUP
  btnRev.attach(PIN_REV);
  btnRev.interval(5);
  */
  
  pinMode(PIN_DARTTRIGGER,INPUT_PULLUP);  // PULLUP
  btnTrigger.attach(PIN_DARTTRIGGER);
  btnTrigger.interval(5);

  pinMode(PIN_MODEFIRE,INPUT_PULLUP);     // PULLUP
  btnModeFire.attach(PIN_MODEFIRE);
  btnModeFire.interval(5);

  pinMode(PIN_DARTRESET, INPUT_PULLUP);   // PULLUP
  btnDartReset.attach(PIN_DARTRESET);
  btnDartReset.interval(5);

  pinMode(PIN_SETTING,INPUT_PULLUP);      // PULLUP
  btnSetting.attach(PIN_SETTING);
  btnSetting.interval(5);

  pinMode(PIN_VOLTREAD, INPUT);           // Not using PULLUP analog read 0 to 1023

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // OUTPUT PINs setup
  ///////////////////////////////////////////////////////////////////////////////////////////////////////  

  pusherESC.attach(PIN_PUSHERESC);
  pusherESC.writeMicroseconds(THROTTLE_PUSHER_MIN);
  delay(2000);
  
  motorsESC.attach(PIN_MOTORSESC);
  motorsESC.writeMicroseconds(THROTTLE_MOTORS_MIN);
  delay(2000);

  magOut = (digitalRead(PIN_DARTRESET)==HIGH);
  readVoltage();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  //delay(2000);
  display.clearDisplay();
  if (magOut) {
    dartLeft     = 0;
    updateMagOutDisplay(); 
  } else {
    updateSettingDisplay();
  }
}

void loop() { // Main Loop  
  if (!batteryLow) {  
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Update all buttons
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    btnDartCount.update();
    btnTrigger.update();
    btnModeFire.update();
    btnDartReset.update();
    btnSetting.update();
    //btnRev.update(); // For testing only, unnecessary for implementation
  
    /* // For testing only, unnecessary for implementation 
    if (btnRev.fell()) {              // pressed
      motorsESC.writeMicroseconds(THROTTLE_MOTORS_MAX);
    } else if (btnRev.rose()) {       // released
      motorsESC.writeMicroseconds(THROTTLE_MOTORS_BAKE);
    }  
    */
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Mag Out
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnDartReset.fell()) { // pressed, there is a Mag in the blaster
      dartLeft = ammoLimit;
      magOut   = false;
      if (tungstenV2Mode) {
        updateV2ModeDisplay();
      } else {
        updateSettingDisplay();
      }
    } else if (btnDartReset.rose()) { // No Mag in the blaster      
      shutdownSys();
      magOut = true;
      updateMagOutDisplay();
    }
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Trigger Pull/Release
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnTrigger.fell()) {               // pull
      if (magOut) {
        if (ammoLimit < AMMO_UPPER_LIMIT) {
          ammoLimit++;      
          updateMagOutDisplay();
        }
      } else if (tungstenV2Mode) {
        if (digitalRead(PIN_MODEFIRE) == HIGH) {
          triggerPressedHandle(MODE_SINGLE);
          v2ModeFullAuto = false;
        } else {
          triggerPressedHandle(MODE_AUTO);
          v2ModeFullAuto = true;
        }      
      } else {
        triggerPressedHandle(modeFire);
      }        
    } else if (btnTrigger.rose()) {        // released
      triggerReleasedHandle();
    }
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Dart Count switch
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnDartCount.rose()) {        // counter switch released, pusher is extending towards dart 
      shotFiredHandle();              // for firing
    } else if (btnDartCount.fell()) { // pusher is returning from firing dart
      shotFiredReturnHandle();
    }
  
    pusherReturnCheck();
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Firing Mode change: Single Shot, Burst, Full Auto
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnModeFire.fell()) { // pressed
      if (magOut) {
        if (ammoLimit > AMMO_LOWER_LIMIT) {
          ammoLimit--;      
          updateMagOutDisplay();
        }
      } else if (tungstenV2Mode) { 
        if (digitalRead(PIN_DARTTRIGGER) == HIGH) {
          triggerPressedHandle(MODE_BURST);
          v2ModeFullAuto = false;
        } else {
          triggerPressedHandle(MODE_AUTO);
          v2ModeFullAuto = true;
        }      
      } else {
        modeFire = ++modeFire % NUM_OF_MODE; // next mode
        modeROFSetted = ((modeFire == MODE_SINGLE) || (modeFire == MODE_BURST)) ? MODE_ROF_STANDARD : modeROFSelected;    
        // For Single/Burst Shot mode, the ROF will be setted to Standard only    
        updateSettingDisplay();
      }
    }
  
    if (btnSetting.fell()) {
      settingPressedHandle();
    }
  } else {
    // Battery is Low
    // Stop all Motors just in case.
    shutdownSys();    
    updateBatteryLowDisplay();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiredHandle() {
  if (dartLeft > 0) {

    // to refresh the whole display is too time consuming
    // therefore just overwrite the old dart count by switching to background color
    // write the current count on the screen, this remove it from the screen
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.setCursor(90,18);
    display.print(dartLeft);
    
    dartLeft--;     // decrease dart count
    dartToBeFire--;    

    if (dartToBeFire == 0) {
        delay(returnDelay);
        pusherESC.writeMicroseconds(THROTTLE_PUSHER_BAKE);
        isFiring = false;
    }

    // switch back to white text and write the new count
    display.setTextColor(WHITE);
    display.setCursor(90,18);
    display.print(dartLeft);
    display.display();
    
  } 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredReturnHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiredReturnHandle() {
    if (dartLeft <= 0 || dartToBeFire <= 0) {   
      pusherESC.writeMicroseconds(THROTTLE_PUSHER_BAKE); // make sure pusher motor stops
      motorsESC.writeMicroseconds(THROTTLE_MOTORS_BAKE); // stop flywheels

      if (isPusherOut) { // the pusher return module is active, deactivate it 
        isPusherOut = false;
        pusherTimerStart = 0;
      }
          
      dartToBeFire = 0;
      if (magOut) {
        updateMagOutDisplay();
      } else if (tungstenV2Mode) {
        updateV2ModeDisplay();
      } else {
        updateSettingDisplay();
      }     
    } else if (dartToBeFire == 1) {
        pusherESC.writeMicroseconds(THROTTLE_PUSHER_REDUCE);        
    }    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerPressedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerPressedHandle(int caseModeFire) {  
  if (dartLeft > 0){
    switch(caseModeFire) {
      case MODE_SINGLE: dartToBeFire++; break;
      case MODE_BURST : dartToBeFire += burstLimit; 
          if (dartToBeFire > dartLeft) {
            dartToBeFire = dartLeft;
          }
        break;
      case MODE_AUTO  : dartToBeFire = dartLeft;
    }

    if (dartToBeFire == 1) {
      returnDelay = RETURN_DELAY_ONESHOT;
    } else {
      returnDelay = RETURN_DELAY_MULTISHOT;
    }
       
    int pusherThrottle = map(rofLimitArr[modeROFSetted] , 0, 100, THROTTLE_PUSHER_MIN, THROTTLE_PUSHER_MAX);

    // Rev flywheels
    motorsESC.writeMicroseconds(THROTTLE_MOTORS_MAX);
    delay(REV_UP_DELAY);    

    // Start Firing
    pusherESC.writeMicroseconds(pusherThrottle);    
    isFiring = true;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerReleasedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerReleasedHandle() {  
  if ((modeFire == MODE_AUTO || v2ModeFullAuto) && isFiring) {
      v2ModeFullAuto = false; // exit v2ModeFullAuto if it is on 
      if (dartToBeFire > 2) {
        pusherESC.writeMicroseconds(THROTTLE_PUSHER_RETURN);
        dartToBeFire = 2;    // fire off 2 last shot
      }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: readVoltage
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void readVoltage() {
  int voltagePinValue = (int) (analogRead(PIN_VOLTREAD) / 4.176);    
  float newVoltage    = (voltagePinValue / 10.0); 

  if (!batteryLow) {
    currentVoltage = (newVoltage < currentVoltage) ? newVoltage : currentVoltage;      
  } else {
    currentVoltage = (newVoltage > BATTERY_MIN) ? newVoltage : currentVoltage;  
  }
  batteryLow = (currentVoltage <= BATTERY_MIN);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: settingPressedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void settingPressedHandle() {
  if (magOut) {
    ammoLimitSelected = ++ammoLimitSelected % ammoLimitArrSize;
    ammoLimit         = ammoLimitArr[ammoLimitSelected];
    updateMagOutDisplay();
  } else {  
    switch (modeFire) {
      case MODE_SINGLE :
        tungstenV2Mode = !tungstenV2Mode; // switch between V2 mode and normal mode
        break;
      case MODE_BURST :
          burstLimit++;
          if (burstLimit > BURST_UPPER_LIMIT){
            burstLimit = BURST_LOWER_LIMIT;
          }
        break;
      case MODE_AUTO   :
          modeROFSelected = ++modeROFSelected % NUM_OF_MODE_ROF;
          modeROFSetted   = modeROFSelected;
        break;  
    }
    if (tungstenV2Mode) {
      updateV2ModeDisplay();
    } else {
      updateSettingDisplay();
    }
  }  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: pusherReturnCheck
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void pusherReturnCheck() {
  if (isPusherOut) {
    if ((pusherTimerStart != 0) && ((millis() - pusherTimerStart) >= (pusherOutThreshold + returnDelay))) {
      motorsESC.writeMicroseconds(THROTTLE_MOTORS_MAX);      
      pusherESC.writeMicroseconds(THROTTLE_PUSHER_REDUCE);
      pusherTimerStart = 0;
    }
  } else {
    if (!isFiring && digitalRead(PIN_DARTCOUNT) == HIGH) {
        isPusherOut = true;
        pusherTimerStart = millis();
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateSettingDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSettingDisplay() {
  readVoltage();
  int numOfCircle = 1;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
  
  display.setCursor(0,19);
  display.println("Ammo Count");  
  
  display.setCursor(0,32);
  switch(modeFire) {
      case MODE_SINGLE: 
          display.println("Single Shot");  
        break;
      case MODE_BURST : 
          numOfCircle = burstLimit;
          display.print(burstLimit);          
          display.println(" Rounds Burst");  
        break;
      case MODE_AUTO  : 
          numOfCircle = 10;
          display.println("Full Auto");  
        break;
    }
    
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSetted]);  
  
  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateLimitDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateMagOutDisplay() {
  readVoltage();
  
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
  
  display.setCursor(0,19);
  display.println("MAG OUT - SET");  
      
  display.setCursor(0,57);
  display.println("<*-*-*-*-*^*-*-*-*-*>");  

  display.setTextSize(2);
  display.setCursor(0,32);
  display.println("-AMMO-");  
  
  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(ammoLimit);  

  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateV2ModeDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateV2ModeDisplay() {
  readVoltage();
  int numOfCircle = burstLimit;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
      
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSelected]);  

  display.setTextSize(2);
  display.setCursor(0,21);
  display.println("<-V2->");  

  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateBatteryLowDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateBatteryLowDisplay() {
  readVoltage();
  if (batteryLow) {
    display.clearDisplay();
        
    display.setTextSize(2);
    display.setCursor(0,14);
    display.println("+-------+");  
    display.println("|BattLow|=");  
    display.println("+-------+");  
  
    display.display();
  } else {
    if (magOut) {
      updateMagOutDisplay();
    } else if (tungstenV2Mode) {
      updateV2ModeDisplay();
    } else {
      updateSettingDisplay();
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shutdown
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shutdownSys() {
  dartToBeFire = 0;
  pusherESC.writeMicroseconds(THROTTLE_PUSHER_BAKE);
  motorsESC.writeMicroseconds(THROTTLE_MOTORS_BAKE);
  isFiring     = false;
}



