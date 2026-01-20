#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct DreamFocuserCommand
{
    char M = 'M';
    char k;
    unsigned char a;
    unsigned char b;
    unsigned char c;
    unsigned char d;
    unsigned char addr = '\0';
    unsigned char z;
};

#define DIR_PIN 2
#define STEP_PIN 4
#define MOTOR_INTERFACE 1
#define MS1 10
#define MS2 11
#define MS3 14
#define START_POSITION 30000

// registers
int curTemp = 20;
int curHumidity = 50;
bool running = false;
long destPos = START_POSITION;

AccelStepper accelStepper(MOTOR_INTERFACE, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  
  accelStepper.setMaxSpeed(1000);
  accelStepper.setAcceleration(50);
  accelStepper.setSpeed(200);
  accelStepper.setCurrentPosition(START_POSITION);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
}

unsigned char calcChecksum(DreamFocuserCommand *cmd) {
    unsigned char z;

    // calculate checksum
    z = (cmd->M + cmd->k + cmd->a + cmd->b + cmd->c + cmd->d + cmd->addr) & 0xff;
    return z;
}

void returnLong(long num, DreamFocuserCommand *cmd) {
  long dig = num;
  cmd->d = (char)(dig & 0xff);
  dig = dig >> 8;
  cmd->c = (char)(dig & 0xff);
  dig = dig >> 8;
  cmd->b = (char)(dig & 0xff);
  dig = dig >> 8;
  cmd->a = (char)(dig & 0xff);
}

long readLong(DreamFocuserCommand *cmd) {
  long ret = 0;

  ret = ret | cmd->a;
  ret = ret << 8;
  ret = ret | cmd->b;
  ret = ret << 8;
  ret = ret | cmd->c;
  ret = ret << 8;
  ret = ret | cmd->d;

  return ret;
}

void isCalibrated(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;
  ret.a = '\0';
  ret.b = '\0';
  ret.c = '\0';
  ret.d = '\0';
  ret.addr = '\0';
  ret.k = 'W';

  ret.z = calcChecksum(&ret);
  Serial.write((char *)&ret,8);
}

void isMoving(DreamFocuserCommand *cmd) {
  running = accelStepper.isRunning();

  DreamFocuserCommand ret;
  ret.a = '\0';
  ret.b = '\0';
  ret.c = '\0';
  if (running) {
    ret.d = '\1';

  } else {
    ret.d = '\0';
  }
  ret.addr = '\0';
  ret.k = 'I';

  ret.z = calcChecksum(&ret);
  Serial.write((char *)&ret,8);
}

void getPosition(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;
  long dig = accelStepper.currentPosition();
  ret.d = (unsigned char)(dig & 0xff);
  dig = dig >> 8;
  ret.c = (unsigned char)(dig & 0xff);
  dig = dig >> 8;
  ret.b = (unsigned char)(dig & 0xff);
  dig = dig >> 8;
  ret.a = (unsigned char)(dig & 0xff);
  ret.addr = '\0';
  ret.k = 'P';

  if (!accelStepper.isRunning()) {
    destPos = dig;
  }

  ret.z = calcChecksum(&ret);
  Serial.write((char *)&ret,8);
}

void readMemory(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;
  if (cmd->addr == 3) { // Max Position
    returnLong(65535,&ret);
  } else {
    returnLong(0,&ret);
  }
  ret.k = 'A';
  ret.addr = 0;
  ret.z = calcChecksum(&ret);
  Serial.write((char *)&ret,8);

}

void getTemperature(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;
  long retHumidity = curHumidity * 10;
  retHumidity = retHumidity << 16;
  long retTemp = curTemp * 10;
  returnLong(retHumidity + retTemp,&ret);
  ret.k = 'T';
  ret.addr = 0;
  ret.z = calcChecksum(&ret);
  Serial.write((char *)&ret,8);

}

void move(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;

  long position = readLong(cmd);
  accelStepper.moveTo(position);
  accelStepper.run();
  destPos = position;

  Serial.write((char *)cmd,8);

}

void stop(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;
  accelStepper.stop();

  destPos = accelStepper.currentPosition();

  Serial.write((char *)cmd,8);

}

void sync(DreamFocuserCommand *cmd) {
  DreamFocuserCommand ret;
  if (accelStepper.isRunning()) {
    accelStepper.stop();
  }

  long newPos = readLong(cmd);
  accelStepper.setCurrentPosition(newPos);
  destPos = accelStepper.currentPosition();

  Serial.write((char *)cmd,8);

}

void processCommand(DreamFocuserCommand *cmd) {
  switch(cmd->k) 
  {
  case 'M': 
    /* MMabcd0z - set position x
      response - MMabcd0z */
    move(cmd);
    break;
  case 'H': 
    /* MH00000z - stop
      response - MH00000z */
    stop(cmd);
    break;
  case 'P': 
    /* MP00000z - read position
      response - MPabcd0z  */
    getPosition(cmd);
    break;
  case 'I':
    /* MI00000z - is moving
      response - MI000d0z - d = 1: yes, 0: no */
    isMoving(cmd);
    break;
  case 'T':
    getTemperature(cmd);
    /* MT00000z - read temperature
      response - MT00cd0z - temperature = ((c<<8)|d)/16.0 */
    break;
  case 'A':
    readMemory(cmd);
    /* MA0000nz - read memory dword - n = address
      response - MAabcd0z(?) */
    break;
  case 'B':
    /* MBabcdnz - write memory dword - abcd = content, n = address
      response - MBabcd0z(?) */
    break;
  case 'C':
    /* MC0000nz - read memory word - n = address
      response - */
    break;
  case 'D':
    /* MDab00nz - write memory word - ab = content, n = address
      response - */
    break;
  case 'R':
    /* MR000d0z - move with speed d & 0b1111111 (0 - 127), direction d >> 7 (1 up, 0 down)
      response - MR000d0z - */
    break;
  case 'W':
    isCalibrated(cmd);
    /* MW00000z - is calibrated
      response - MW000d0z - d = 1: yes (absolute mode), 0: no (relative mode) */
    break;
  case 'Z':
    sync(cmd);
    /* MZabcd0z - calibrate toposition x
      response - MZabcd0z */
    break;
  case 'G':
    /* MG00000z - park
      response - MG00000z */
    break;

  default:
    break;
  }
}

void showStats() {
  char printBuf[10];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.print(" Cur Pos: ");
  display.println(ltoa(accelStepper.currentPosition(),printBuf,10));
  display.print("Dest Pos: ");
  display.println(ltoa(destPos,printBuf,10));
  

  display.display();
}

void loop() {
  DreamFocuserCommand cmd;
  showStats();

  // Check if any data is available in the serial receive buffer
  if (Serial.available() >0) {
    int read = Serial.readBytes((char *)&cmd,sizeof(DreamFocuserCommand));
    if (read == 8) {
      processCommand(&cmd);
    }
  }
}
