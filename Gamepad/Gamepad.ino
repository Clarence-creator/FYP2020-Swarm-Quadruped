#include "U8glib.h"
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI 



#define __DEBUG__
//#define __RF24__

// Baud rate must match the setting of UART modules(HC-05,HM-10 and etc),
// set module to master mode if requires
#define BAUD 57600

#define Console Serial
//#define BlueTooth Serial


    #include <SoftwareSerial.h>
//#undef BlueTooth
    #define BLE_TX       10 //接蓝牙RX
    #define BLE_RX       11 //接蓝牙TX
    #define BLEbaud  9600 
    SoftwareSerial BlueTooth(BLE_RX, BLE_TX);   //RX, TX
    //SoftwareSerial BlueTooth(11, 10); //11 接蓝牙 TX，  10接蓝牙RX


#ifdef __RF24__
/*
  library  - https://github.com/tmrh20/RF24/
  tutorial - https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE  2   // ce pin
#define CSN 3   // csn pin
const char const address[]  = {0x01, 0xff, 0xff, 0xff, 0xff, 0xff}; // change if needs
RF24 radio(CE, CSN);
#endif

// joystick pins
#define VRX_PIN   A0
#define VRY_PIN   A1
#define SW_PIN    A2

//
#define BEEPER_PIN A3

// pins for matrix switches
#define MATRIX_ROW_START 6 // pin 6 to 9
#define MATRIX_NROW 4
#define MATRIX_COL_START 2 // pin 2 to 5
#define MATRIX_NCOL 4

const char *walks1[] {
  "wkR", "wkL", "wk", "bk",
};

const char *walks2[] {
  "trR", "trL", "tr", "bk",
};

const char *walks3[] {
  "crR", "crL", "cr", "bk",
};

const char *walks4[] {
  "bkR", "bkL",  "tr", "bk",
};

const char **walkings[] {
  walks1, walks2, walks3, walks4,
};

char **walkptr;

const char *row1[] {
  "str", "sit", "buttUp", "pee",
};

const char *row2[] {
  "c","rest", "zero", "pu",
};

// disabled
const char *row3[] {
  "d", "balance", "vt",  "show", "dropped",
};

char *cmdptr,prev_cmd[24] = "rest";
boolean is_gait = true;

char **instincts[] {
  walkings[0], row1, row2, row3
};






void setup() {


   // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  u8g.setFontPosTop();

  
  
  Console.begin(BAUD);
  while (!Console); // wait for serial port to connect. Needed for native USB port only
#ifdef __DEBUG__
  Console.println("started!");
  BlueTooth.begin(BLEbaud);
#endif

  //
  //pinMode(SW_PIN, INPUT);
  //digitalWrite(SW_PIN, HIGH);
  pinMode(SW_PIN, INPUT_PULLUP); //注意 Z 轴输入一定要上拉，不然电平不稳。
  //
#ifdef __RF24__
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
#endif
  tone(BEEPER_PIN, 300);
  delay(100);
  noTone(BEEPER_PIN);





}

void loop() {
  boolean sw1 = false;
  static long cmdPeriod = 0;
  int longClick = 0;
  int joy_x;
  int joy_y;
  int row = -1, col = -1;
  char token = 'k';

  int matrix = scanmatrix(&longClick);
  if (matrix >= 0) {
    row = (matrix) / 4;
    col = matrix % 4;
    if (row == 0) {
      instincts[0] = walkings[col];
      col = 2;
    } else {
      cmdptr = instincts[row][col];
      is_gait = false;
      goto __send;
    }
  }

  joy_x = analogRead(VRY_PIN);
  joy_y = analogRead(VRX_PIN);
  /*
  //碰到有问题的加
  joy_x = map(analogRead(VRY_PIN), 0, 1023, 0, 255);
  joy_y = map(analogRead(VRX_PIN), 0, 1023, 0, 255);
*/
  if (joy_x < 300) {
    col = 0;
    is_gait = true;
  } else if (joy_x > 900) {
    col = 1;
    is_gait = true;
  } else if (joy_y < 300) {
    col = 2;
    is_gait = true;
  } else if (joy_y > 900) {
    col = 3;
    is_gait = true;
  } else {
    if (is_gait)
      cmdptr = "balance";
  }
  sw1 = digitalRead(SW_PIN);
  if (!sw1) {
    // disable servo
    token = 'd';
    cmdptr = "d";  //临时修改， 源代码：cmdptr = "";  是因为主板上蓝牙对接RX,TX， 
  }

  if (col >= 0) {
    walkptr = instincts[0];
    cmdptr = walkptr[col];
  }

__send:
  if (millis() >= cmdPeriod && strcmp(cmdptr, prev_cmd) != 0) {
    //BlueTooth.write(token); //临时屏蔽，因为不是机器人主板上蓝牙对接主板的RX,TX，需要通过内部解释
#ifdef __RF24__
    radio.write(&token, 1);
#endif
    if (cmdptr != nullptr) {
      BlueTooth.print("{\"btn-dir\":\"");
      BlueTooth.print(cmdptr);
      BlueTooth.println("\"}");
      //BlueTooth.write(cmd, strlen(cmd));
#ifdef __RF24__
      radio.write(cmd, strlen(cmd));
#endif
      strcpy(prev_cmd, cmdptr);
#ifdef __DEBUG__
      //Console.write(token);
      Console.println(cmdptr);
      /*//查看输出字符串
      Console.print("{\"btn-dir\":\"");
      Console.print(cmdptr);
      Console.print("\"}");
      Console.println();
      */

    
     // picture loop
      u8g.firstPage();  
      do {
        //draw();
        //u8g.setFont(u8g_font_unifont);
        u8g.setFont(u8g_font_9x15);
          u8g.setPrintPos(0, 15); 
        //u8g.setFont(u8g_font_osb21);
            u8g.print("CMD: ");
            u8g.print(cmdptr);
            u8g.println("");
      } while( u8g.nextPage() );
  
#endif
    }
    cmdPeriod = millis() + 50;
  }

}

int scanmatrix(int *longClick) {
  static int priorMatrix = -1;
  static long curMatrixStartTime = 0;

  *longClick = 0;
  // we will energize row lines then read column lines
  // first set all rows to high impedance mode
  for (int row = 0; row < MATRIX_NROW; row++) {
    pinMode(MATRIX_ROW_START + row, INPUT);
  }
  // set all columns to pullup inputs
  for (int col = 0; col < MATRIX_NCOL; col++) {
    pinMode(MATRIX_COL_START + col, INPUT_PULLUP);
  }

  // read each row/column combo until we find one that is active
  for (int row = 0; row < MATRIX_NROW; row++) {
    // set only the row we're looking at output low
    pinMode(MATRIX_ROW_START + row, OUTPUT);
    digitalWrite(MATRIX_ROW_START + row, LOW);

    for (int col = 0; col < MATRIX_NCOL; col++) {
      delayMicroseconds(100);
      if (digitalRead(MATRIX_COL_START + col) != LOW) {
        continue;
      }
      // we found the first pushed button
      int curmatrix = row * MATRIX_NROW + col;
      //
      int clicktime = millis() - curMatrixStartTime;
      if (curmatrix != priorMatrix) {
        curMatrixStartTime = millis();
        priorMatrix = curmatrix;
        *longClick = 0;
      } else if (clicktime > 500) {
        // User has been holding down the same button continuously for a long time
        if (clicktime > 3000) {
          *longClick = 2;
        } else {
          *longClick = 1;
        }

      }
      return curmatrix;
    }
    pinMode(MATRIX_ROW_START + row, INPUT); // set back to high impedance
    //delay(1);
  }

  return -1;
}




void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( 0, 22, "Hello World!");
}
