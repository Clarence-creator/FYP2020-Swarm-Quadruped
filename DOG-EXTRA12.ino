//================================自己加入的PS2X控制部分======================================
//#define PS2_INPUT
#ifdef PS2_INPUT
  #include <PS2X_lib.h>   // Install the PS2X library Forked here: https://github.com/Toglefritz/Arduino-PS2X
  PS2X ps2x; // create PS2 Controller Class
  #define PS2_DAT      14
  #define PS2_CMD      15
  #define PS2_SEL      16
  #define PS2_CLK      17
  byte gamepadtype = 0;
  int error = 0; 
  byte vibrate = 0;
  //#define pressures   false
  //#define rumble      false
#endif
//=====================================================================

  String btn_cmd = "";
//================================自己加入蓝牙5控制部分==========================
#define BLE_INPUT
#ifdef  BLE_INPUT
    //串口通信
    #include <SoftwareSerial.h>
    //#include <ArduinoJson.h>
    #define BLE_TX       15 //A1   接BLU  RX
    #define BLE_RX       16 //A2   接BLU  TX
    #define BLEbaud  9600 
    SoftwareSerial BLE(BLE_RX, BLE_TX);   //RX, TX
    //SoftwareSerial BLE(4, 3);   //RX4 接蓝牙tx, TX3 接蓝牙rx
#endif

//#define MP3_PLAY
#ifdef MP3_PLAY
    #include "DFRobotDFPlayerMini.h"
    //#define MP3_TX       16   //A2  接MP3  RX
    //#define MP3_RX       17   //A3  接MP3 TX
    //SoftwareSerial MP3(MP3_RX, MP3_TX);  
    SoftwareSerial MP3(17, 16); 
    DFRobotDFPlayerMini myDFPlayer;
#endif

#define MAIN_SKETCH
#include "WriteInstinct/EZCat.h"
#ifdef GYRO
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#endif

#define PACKET_SIZE 42
#define OVERFLOW_THRESHOLD 128

//#if OVERFLOW_THRESHOLD>1024-1024%PACKET_SIZE-1   // when using (1024-1024%PACKET_SIZE) as the overflow resetThreshold, the packet buffer may be broken
// and the reading will be unpredictable. it should be replaced with previous reading to avoid jumping
#define FIX_OVERFLOW
//#endif
#define HISTORY 2
int8_t lag = 0;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprLag[HISTORY][2];

#ifdef GYRO
MPU6050 mpu;
#endif

#define OUTPUT_READABLE_YAWPITCHROLL
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[PACKET_SIZE]; // FIFO storage buffer

#ifdef GYRO
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
#endif
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/******************
// https://brainy-bits.com/blogs/tutorials/ir-remote-arduino
//-----( Declare objects )-----
//#define IR_RECIEVER A0
#ifdef IR_RECIEVER
#include <IRremote.h>
IRrecv irrecv(IR_RECIEVER);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
String translateIR() // takes action based on IR code received
// describing Remote IR codes.
{
  switch (results.value) { //why the program is printing the IR key twice? //Rz's note
    //IR signal    key on IR remote       //abbreviation of gaits   //gait/posture names
    case 0xFFA25D: PTLF(" CH-");          return (F("sit"));
    case 0xFF629D: PTLF(" CH");           return (F("d"));          //shutdown all servos
    case 0xFFE21D: PTLF(" CH+");          return (F("balance"));    //neutral standing
    case 0xFF22DD: PTLF(" |<<");          return (F("rc"));         //recover (turtle roll )
    case 0xFF02FD: PTLF(" >>|");          return (F("pu"));         //push up
    case 0xFFC23D: PTLF(" >||");          return (F("str"));        //stretch
    case 0xFFE01F: PTLF(" -");            return (F("buttUp"));     //butt up
    case 0xFFA857: PTLF(" +");            return (F("ly"));         //lay down crawling
    case 0xFF906F: PTLF(" EQ");           return (F("pee"));        //stand on three feet
    case 0xFF0DF2: PTLF(" 0");            return (F("trL"));        //trot left
    case 0xFF9867: PTLF(" 100+");         return (F("tr"));         //trot fast/run
    case 0xFFB04F: PTLF(" 200+");         return (F("trR"));        //trot right
    case 0xFF10EF: PTLF(" 1");            return (F("crL"));        //crawl left
    case 0xFF11EE: PTLF(" 2");            return (F("cr"));         //crawl fast
    case 0xFF12ED: PTLF(" 3");            return (F("crR"));        //crawl right
    case 0xFF14EB: PTLF(" 4");            return (F("bkL"));        // back left
    case 0xFF15EA: PTLF(" 5");            return (F("bk"));         //back
    case 0xFF16E9: PTLF(" 6");            return (F("bkR"));        //back right
    case 0xFF18E7: PTLF(" 7");            return (F("calib"));      //calibration posture
    case 0xFF19E6: PTLF(" 8");            return (F("zero"));       //customed skill
    case 0xFF1AE5: PTLF(" 9");            return (F("zero"));       //customed skill
    case 0xFFFFFFFF: return (""); //Serial.println(" REPEAT");

    default: {
        Serial.println(results.value, HEX);
      }
      return ("");                      //Serial.println("null");
  }// End Case
  //delay(100); // Do not get immediate repeat //no need because the main loop is slow

  // The control could be organized in another way, such as:
  // forward/backward to change the gaits corresponding to different speeds.
  // left/right key for turning left and right
  // number keys for different postures or behaviors
}
#endif
*********************/

char token;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN];
char *newCmd = new char[CMD_LEN];
byte newCmdIdx = 0;
byte hold = 0;

uint8_t timer = 0;
//#define SKIP 1
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstWalkingJoint;
byte jointIdx = 0;


unsigned long usedTime = 0;
#ifdef GYRO
void checkBodyMotion()  {
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) ;
  if (mpuInterrupt || fifoCount >= packetSize)
  {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    //PTL(fifoCount);
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount > OVERFLOW_THRESHOLD) { //1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)

      // -- RzLi --
#ifdef FIX_OVERFLOW
      PTLF("FIFO overflow! Using last reading!");
      lag = (lag - 1 + HISTORY) % HISTORY;
#endif
      // --
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef MPU_YAW180
      ypr[2] = -ypr[2];
#else
      ypr[1] = -ypr[1] ;
#endif
#endif
      /*PT(ypr[1] * degPerRad);
        PTF("\t");
        PTL(ypr[2] * degPerRad);*/
      // overflow is detected after the ypr is read. it's necessary to keep a lag recrod of previous reading.  -- RzLi --
#ifdef FIX_OVERFLOW
      for (byte g = 0; g < 2; g++) {
        yprLag[lag][g] = ypr[g + 1] * degPerRad;
        ypr[g + 1] = yprLag[(lag - 1 + HISTORY) % HISTORY][g] * radPerDeg;
      }
      lag = (lag + 1) % HISTORY;
#endif
      // --
      //deal with accidents
      if (fabs(ypr[1])*degPerRad > LARGE_PITCH) {
        PT(ypr[1] * degPerRad);
        PTF("\t");
        PTL(ypr[2] * degPerRad);
        if (!hold) {
          token = 'k';
          strcpy(newCmd, ypr[1]*degPerRad > LARGE_PITCH ? "lifted" : "dropped");
          newCmdIdx = 1;
        }
        hold = 10;
      }
      // recover
      else if (hold) {
        if (hold == 10) {
          token = 'k';
          strcpy(newCmd, "balance");
          newCmdIdx = 1;
        }
        hold --;
        if (!hold) {
          char temp[CMD_LEN];
          strcpy(temp, newCmd);
          strcpy(newCmd, lastCmd);
          strcpy(lastCmd, temp);
          newCmdIdx = 1;
          
          #ifdef MP3_PLAY
            myDFPlayer.play(4);
          #endif
        }
      }
      //calculate deviation
      for (byte i = 0; i < 2; i++) {
        RollPitchDeviation[i] = ypr[2 - i] * degPerRad - motion.expectedRollPitch[i];
        RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i], 0);//filter out small angles
      }
    }
  }
}
#endif //GYRO
bool auto_mode = false;
unsigned long cur_time;

void setup() {
  pinMode(BUZZER, OUTPUT);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#ifdef GYRO
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000);
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
#endif

  Serial.begin(57600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  PTLF("\n* 12DOF EXTRA-DOG Starting *");



#ifdef GYRO
  //PTLF("Connecting MPU6050...");//临时屏蔽
  mpu.initialize();
  //do
  {
    delay(1000);
    // verify connection
    //PTLF("Testing connections...");//临时屏蔽
    PTL(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));//sometimes it shows "failed" but is ok to bypass.
  } //while (!mpu.testConnection());

  // load and configure the DMP
  do {
    //PTLF("Initializing DMP...");//临时屏蔽
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity

    for (byte i = 0; i < 4; i++) {
      PT(EEPROMReadInt(MPUCALIB + 4 + i * 2));
      PT(" ");
    }
    PTL();
    mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
    mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
    mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
    mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      //PTLF("Enabling DMP...");  //临时屏蔽
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      //PTLF("Enabling interrupt detection"); //临时屏蔽
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      PTLF("DMP ready!");
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      PTLF("DMP failed (code ");
      PT(devStatus);
      PTLF(")");
      PTL();
    }
  } while (devStatus);
#endif
  //opening music
#if WalkingDOF == 8
  //pinMode(BUZZER, OUTPUT);
  playMelody(MELODY);
#endif
/***********临时屏蔽**********红外部分
#ifdef IR_RECIEVER
  //IR
  {
    //PTLF("IR Receiver Button Decode");
    irrecv.enableIRIn(); // Start the receiver
  }
#endif
*****************************/

  assignSkillAddressToOnboardEeprom();
  //PTL();

  // servo
  {
    //pwm.begin();
    //pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates
    initServos();
    delay(200);

    //meow();
    strcpy(lastCmd, "rest");
    motion.loadBySkillName("rest");
    for (int8_t i = DOF - 1; i >= 0; i--) {
      pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
      //servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
      //PTL(pulseToAngle(calibratedDuty0[i]));
      calibratedPWM(i, motion.dutyAngles[i]);
      delay(100);
    }
    //randomSeed(analogRead(0));//use the fluctuation of voltage caused by servos as entropy pool
    shutServos();
    token = 'd';
  }
/********************
#ifndef IR_RECIEVER
  pinMode(A0, OUTPUT);
  digitalWrite(A0, 0);
  pinMode(A0, INPUT);
  if (digitalRead(A0)) {
    auto_mode = true;
  }
#endif
*********************/
  //beep(30);//临时屏蔽
/**************************
#ifdef ULTRA_SOUND
  pinMode(TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO, INPUT); // Sets the echoPin as an Input
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER, HIGH);
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(10);
#endif
******************************/
  int t = 0;
  int minDist, maxDist;
  while (0) {//disabled for now. needs virtual threading to reduce lag in motion.
    calibratedPWM(0, -10 * cos(t++*M_PI / 360));
    calibratedPWM(1, 10 * sin(t++ * 2 * M_PI / 360));
/***************************************
#ifdef ULTRA_SOUND
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(BUZZER, HIGH);
    digitalWrite(BUZZER, LOW);
#endif

#ifdef ULTRA_SOUND
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(ECHO, HIGH, farTime);
    // Calculating the distance
    int distance = duration * 0.034 / 2; // 10^-6 * 34000 cm/s
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance == 0 ? LONGEST_DISTANCE : distance);
    Serial.println(" cm");
#endif
*************************************/
  }
//  meow(); //临时屏蔽

#ifdef MP3_PLAY
  MP3.begin(9600);
  myDFPlayer.begin(MP3);
  myDFPlayer.volume(30);
  myDFPlayer.play(1);
#endif

#ifdef  BLE_INPUT
  //=================================蓝牙初始化======================
    //定义蓝牙软串口通信
    BLE.begin(BLEbaud);
    //while(BLE.read()>= 0){}
      //while (!BLE);
  // wait for ready
  //while (BLE.available() && BLE.read()); // empty buffer
  
    //PTLF("Init Bluetooth...");//临时屏蔽
  //==================================================================
#endif



//======================================ps2手柄初始化========================
#ifdef PS2_INPUT
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins
//error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
 //error = ps2x.config_gamepad(9,7,6,8, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
/*
       PTLF("PS2 INIT:");
      PT(error);
      PTLF(",gamepad type:");
      PT(gamepadtype);
      PTL();
*/
   if(error == 0){
     PTLF("Found Controller");
   }
  else if(error == 1)
   PTLF("No controller found");
   
  else if(error == 2)
   PTLF("Controller found but not accepting commands");
   
  else if(error == 3)
   PTLF("Controller refusing to enter Pressures mode");

  gamepadtype=ps2x.readType();  
     switch(gamepadtype) {
       case 0:
        PTLF("Unknown type");
       break;
       case 1:
        PTLF("DualShock Found");
       break;
     }
      
#endif
  pinMode(BUZZER, OUTPUT);
}

float pulseToAngle(float pulse) {
  return map(pulse, SERVOMIN, SERVOMAX, 0, 180);
}

void loop() {
  //cur_time = millis();
  newCmd[0] = '\0';
  newCmdIdx = 0;
  // MPU block
#ifdef GYRO
  checkBodyMotion();
#endif
  // accident block
  //...
  //...
  //for obstacle avoidance and auto recovery

  // input block



#ifdef PS2_INPUT
  if(gamepadtype != 2){ // Controller
    ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
    // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
     
     if (ps2x.ButtonPressed(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
        btn_cmd ="wkF";
      }      
     if(ps2x.ButtonPressed(PSB_PAD_RIGHT)){
        btn_cmd ="wkR";
      }
     if(ps2x.ButtonPressed(PSB_PAD_LEFT)){
        btn_cmd ="wkL";
      }
     if(ps2x.ButtonPressed(PSB_PAD_DOWN)){
        btn_cmd ="bk";
      }
      
     if(ps2x.ButtonPressed(PSB_L1)){//squer
        btn_cmd ="balance";
        }
     if(ps2x.ButtonPressed(PSB_L2)){//cross
        btn_cmd ="vt";
        }
    if(ps2x.ButtonPressed(PSB_R1)){//Triangle 
        btn_cmd ="sit";
     }
     if(ps2x.ButtonPressed(PSB_R2)){    //Circle just pressed
        btn_cmd ="d";
      }
      
    if(ps2x.ButtonPressed(PSB_SELECT)){
      //Serial.println("Select is being held");
      btn_cmd="";
      }
         
         
 }

#endif


#ifdef  BLE_INPUT
//===============================================自己加入的蓝牙读取数据====================================
  //BLE.listen();
  if(BLE.available()>0){
    if(BLE.findUntil(":\"","}")){
      btn_cmd=BLE.readStringUntil('\r\n');
      //int ei=btn_cmd.lastIndexOf("\"}");
      btn_cmd=btn_cmd.substring(0,btn_cmd.lastIndexOf("\"}"));
      PTL(btn_cmd);
    }
  }
  //while(BLE.read()>= 0){}
#endif



  
    if ( btn_cmd !="" && btn_cmd != "state"){     // btn_cmd != NULL &&

      strcpy(newCmd, btn_cmd.c_str());
      //PTL(newCmd);
      //btn_cmd="";
      if (!strcmp(newCmd, "d")){
        token = 'd';
        auto_mode = false;
      } else if (!strcmp(newCmd, "show")) {
        char **bList = new char*[10];
        /*bList[0] = "rc1";
        bList[1] = "rc2";
        bList[2] = "rc3";
        bList[3] = "rc4";
        bList[4] = "rc5";
        bList[5] = "rc6";
        bList[6] = "rc7";
        bList[7] = "rc8";
        bList[8] = "rc9";
        bList[9] = "rc10";
        */
        bList[0] = "rest";
        bList[1] = "buttUp";
        bList[2] = "cd1";
        bList[3] = "cd2";
        bList[4] = "pee1";
        bList[5] = "pee";
        bList[6] = "str";
        bList[7] = "pu1";
        bList[8] = "pu2";
        bList[9] = "zero";
        float speedRatio[10] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
        int pause[10] = {200, 500, 500, 500, 200, 200, 200, 500, 500, 500};
        behavior( 10, bList, speedRatio, pause);
        strcpy(newCmd, "rest");
        delete []bList;
      } else if (!strcmp(newCmd, "pu")) {
        char **bList = new char*[2];
        bList[0] = "pu1";
        bList[1] = "pu2";
        float speedRatio[2] = {2, 2};
        int pause[2] = {0, 0};
        for (byte i = 0; i < 3; i++)
          behavior(2, bList, speedRatio, pause);
        strcpy(newCmd, "rest");
        //meow();
        delete []bList;
      } else if (!strcmp(newCmd, "auto")) {
            auto_mode=true;
            #ifdef MP3_PLAY
              myDFPlayer.play(4); 
            #endif
            token = 'c';
      } else if (!strcmp(newCmd, "c")) {
            token = 'c';
      } else
        token = 'k';
      newCmdIdx = 2;
    }
 btn_cmd="";


/*//临时频闭
#ifdef IR_RECIEVER
  if (irrecv.decode(&results)) {
    if (translateIR() != "") {
      strcpy(newCmd, translateIR().c_str());
      if (!strcmp(newCmd, "d"))
        token = 'd';
      else if (!strcmp(newCmd, "rc")) {
        char **bList = new char*[10];
        bList[0] = "rc1";
        bList[1] = "rc2";
        bList[2] = "rc3";
        bList[3] = "rc4";
        bList[4] = "rc5";
        bList[5] = "rc6";
        bList[6] = "rc7";
        bList[7] = "rc8";
        bList[8] = "rc9";
        bList[9] = "rc10";
        float speedRatio[10] = {2, 2, 2, 10, 5, 10, 5, 5, 5, 2};
        int pause[10] = {500, 500, 500, 0, 0, 0, 0, 0, 500, 100};
        behavior( 10, bList, speedRatio, pause);
        strcpy(newCmd, "rest");

      } else if (!strcmp(newCmd, "pu")) {
        char **bList = new char*[2];
        bList[0] = "pu1";
        bList[1] = "pu2";
        float speedRatio[2] = {2, 2};
        int pause[2] = {0, 0};
        for (byte i = 0; i < 3; i++)
          behavior(2, bList, speedRatio, pause);
        strcpy(newCmd, "rest");
        meow();

      } else
        token = 'k';
      newCmdIdx = 2;
    }
    irrecv.resume(); // receive the next value
  }
#endif
*/

  if ( Serial.available() > 0) {
    token = Serial.read();
    newCmdIdx = 3;
  }
/*
  if (auto_mode) {
    static char *movements[] = {"balance", "cr", "tr", "cr", "ly", "sleep","balance"};
    //static char *movements[] = {"balance", "vt","rest"};
    static short timeouts[] {2000, 2000, 5000, 5000, 2000, 2000, 2000,2000};
    static unsigned long old_time = cur_time + timeouts[0];
    static int c = 0;

    if (cur_time - old_time >= timeouts[c] ) {
      old_time = cur_time;
      token = 'k';
      newCmdIdx = 4 ;

      c = c % (sizeof(movements) / 2);

      strcpy(newCmd, movements[c++]);
      //PTL(newCmd);
      //PTL(timeouts[c]);
      //strcpy(newCmd, "balance");
      //if (c == sizeof(movements) / 2)
         // auto_mode = false;
      //PTL(c);
    }
  }

*/
  if (newCmdIdx) {
    //PTL(token);
    beep(newCmdIdx * 4);
    // this block handles argumentless tokens
    if (token == 'd' ) {
      motion.loadBySkillName("rest");
      transform( motion.dutyAngles);
      PTLF("shut down servos");
      shutServos();
    }    /* //临时频闭
    else if (token == 's') {
      PTLF("save calibration");
      saveCalib(servoCalibs);
    }

    else if (token == 'a') {
      PTLF("abort calibration");
      for (byte i = 0; i < DOF; i++) {
        servoCalibs[i] = servoCalib( i);
      }
    }
    */
    // this block handles array like arguments
    /*  //临时频闭
      else if (token == 'l' ) {
      byte len = Serial.read();
      PT(len);
      //
      //PTLF("receiving 16 angle list in binary [ byte, ..., byte ] ");
      char *inBuffer = new char[len];
      //
      for (byte i = 0; i < len; i++) {
        inBuffer[i] = Serial.read();
        PT(inBuffer[i]);
        PTL();
      }
      if (len == DOF)
        allCalibratedPWM(inBuffer);
      else
        for (byte i = 0; i < len / 2; i++)
          calibratedPWM(inBuffer[i * 2], inBuffer[i * 2 + 1]);
      //          Serial.readBytes(inBuffer, DOF);
      //          //allCalibratedPWM(dutyAng+1);
      //          delay(200);
      //
      delete [] inBuffer;
    }*/
    else if (token == 'c' || token == 'm') {
      int8_t target[2] = {};
      String inBuffer = Serial.readStringUntil('\n');
      byte inLen = 0;
      strcpy(newCmd, inBuffer.c_str());
      char *pch;
      pch = strtok (newCmd, " ,");
      for (byte c = 0; pch != NULL; c++)
      {
        target[c] = atoi(pch);
        pch = strtok (NULL, " ,");
        inLen++;
      }

      if (token == 'c') {
        //PTLF("calibrating [ targetIdx, angle ]: ");
        if (strcmp(lastCmd, "c")) { //first time entering the calibration function
          motion.loadBySkillName("calib");
          transform( motion.dutyAngles);
        }
        if (inLen == 2)
          servoCalibs[target[0]] = target[1];
        PTL();
        for (byte i = 0; i < DOF; i++) {
          PT(i);
          PT(",\t");
        }
        PTL();
        printList(servoCalibs);
        yield();
      } else if (token == 'm') {
        //SPF("moving [ targetIdx, angle ]: ");
        motion.dutyAngles[target[0]] = target[1];
      }
      PT(token);
      printList(target, 2);

      int duty = SERVOMIN + PWM_RANGE / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + motion.dutyAngles[target[0]]) * pulsePerDegree[target[0]] * rotationDirection(target[0]);
      //pwm.setPWM(pin(target[0]), 0,  duty);
      writeServoMicroseconds(target[0], duty);
    }  else if (Serial.available() > 0) {
      String inBuffer = Serial.readStringUntil('\n');
      strcpy(newCmd, inBuffer.c_str());
    }

    while (Serial.available() && Serial.read()); //flush the remaining serial buffer in case the commands are parsed incorrectly
    //check above
    if (strcmp(newCmd, "") && strcmp(newCmd, lastCmd) ) {
      //      PT("compare lastCmd ");
      //      PT(lastCmd);
      //      PT(" with newCmd ");
      //      PT(token);
      //      PT(newCmd);
      //      PT("\n");
//      if (token == 'w') {}; //some words for undefined behaviors//临时屏蔽

      if (token == 'k') { //validating key
        motion.loadBySkillName(newCmd);
        //motion.info();
        timer = 0;
        if (strcmp(newCmd, "balance") && strcmp(newCmd, "lifted") && strcmp(newCmd, "dropped") )
          strcpy(lastCmd, newCmd);
        // if posture, start jointIdx from 0
        // if gait, walking DOF = 8, start jointIdx from 8
        //          walking DOF = 12, start jointIdx from 4
        firstWalkingJoint = (motion.period == 1) ? 0 : DOF - WalkingDOF;
        postureOrWalkingFactor = (motion.period == 1 ? 1 : POSTURE_WALKING_FACTOR);
        jointIdx = firstWalkingJoint;
        transform( motion.dutyAngles,  1, firstWalkingJoint);

        if (!strcmp(newCmd, "rest")) {
          shutServos();
          token = 'd';
        }
      } else {
        lastCmd[0] = token;
        memset(lastCmd + 1, '\0', CMD_LEN - 1);
      }
    }

            #ifdef MP3_PLAY
              myDFPlayer.play(2); 
            #endif  
            
  }

  //motion block
  {
    if (token == 'k') {
/*
          jointIdx =
#ifdef HEAD  //start from head
            0;
#else
  #ifdef TAIL
              2;
    #else
              4;  //DOF - WALKING_DOF;//临时频闭,当频闭了头部和尾部后，直接从肩部 4 开始读数组                                 
  #endif

#endif
*/

/*
#ifndef HEAD  //skip head
      if (jointIdx < 2)
        jointIdx = 2;
#endif

#ifndef TAIL  //skip tail
      if (jointIdx < 4)
        jointIdx = 4;
#endif
*/

      if (jointIdx < firstWalkingJoint && motion.period > 1) {
        calibratedPWM(jointIdx, 0
#ifdef GYRO
                      + adjust(jointIdx)
#endif
                     );
      }
      else if (jointIdx >= firstWalkingJoint) {
        int dutyIdx = timer * WalkingDOF + jointIdx - firstWalkingJoint;
        calibratedPWM(jointIdx, motion.dutyAngles[dutyIdx]
#ifdef GYRO
                      + adjust(jointIdx)
#endif
                     );
      }
      jointIdx++;

      if (jointIdx == DOF) {
        jointIdx = 0;
#ifdef SKIP
        if (updateFrame++ == SKIP) {
          updateFrame = 0;
#endif
          timer = (timer + 1) % motion.period;
#ifdef SKIP
        }
#endif
      }
    }
  } // motion block ends
}
