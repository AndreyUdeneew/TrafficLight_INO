#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// #define IRQ_PIN 3
// #define XSHUT_PIN 28
// Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

String cmd, CMDcur;
uint8_t programNumber;

volatile int counter = 0;

uint8_t strobeInput = 6;
uint8_t UV_LED = 7;
uint8_t GREEN_LED = 10;
uint8_t RED_LED = 8;
uint8_t IR_LED = 9;

uint8_t SW_pin = 27;
uint8_t VAR_X_pin = 28;
uint8_t VAR_Y_pin = 29;

uint8_t nMotorsSleep = 18;

uint8_t nFLT1 = 17;
uint8_t nEnl = 11;
uint8_t stepPin_1 = 13;
uint8_t dirPin_1 = 12;
uint8_t k1 = 20;
uint8_t k2 = 21;
uint8_t k3 = 22;
// uint8_t k4 = 23;
uint8_t therapyPin = 23;
uint8_t k5 = 24;
uint8_t k6 = 25;

uint8_t nFLT2 = 19;
uint8_t nEn2 = 14;
uint8_t stepPin_2 = 15;
uint8_t dirPin_2 = 26;
uint8_t solenoid_DIR = 3;
uint8_t solenoid_ON = 2;

volatile uint8_t PWM_GREEN = 255;
volatile uint8_t PWM_UV = 255;
volatile uint8_t PWM_Red = 255;
volatile uint8_t PWM_IR = 126;
volatile uint8_t PWM_IR_min = 1;

uint8_t mode;
uint8_t actualFilter = 0;

uint16_t VAR_X = 0;
uint16_t VAR_Y = 0;

int zoomPosition;
int focusPosition;
int zoomTargetPosition;
int focusTargetPsition;
int maxFocusSteps = 2300;
int maxZoomSteps = 2400;
int zoomOptimal = 1162;

int fastLag = 500;
int TIMER_INTERVAL_MS = 500;
int optimalZoom = 1560;
int deltaFocusThresh = 1;

uint8_t autofocusState = 0;

int16_t distance;

volatile uint8_t M[4][2];
volatile uint8_t M0[4][2]{
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 }
};
volatile uint8_t M1[4][2];
volatile uint8_t M2[4][2];
volatile uint8_t M3[4][2];
volatile uint8_t M4[4][2];
volatile uint8_t M5[4][2];
volatile uint8_t M6[4][2];
volatile uint8_t M7[4][2];

uint8_t focusCorrected = 0;


void modesCacheRefresh() {
  // Serial.println("cache refreshing");
  M1[0][0] = 1;  // Red, Green, R/G, R-G
  M1[0][1] = 0;
  M1[1][0] = 0;
  M1[1][1] = 0;
  M1[2][0] = 0;
  M1[2][1] = 0;
  M1[3][0] = 0;
  M1[3][1] = 0;

  M2[0][0] = 0;  //RLED
  M2[0][1] = 0;
  M2[1][0] = 1;
  M2[1][1] = 0;
  M2[2][0] = 0;
  M2[2][1] = 0;
  M2[3][0] = 0;
  M2[3][1] = 0;

  M3[0][0] = 1;  //RLED + UV_LED
  M3[0][1] = 0;
  M3[1][0] = 1;
  M3[1][1] = 0;
  M3[2][0] = 0;
  M3[2][1] = 0;
  M3[3][0] = 0;
  M3[3][1] = 0;

  M4[0][0] = 1;  //oxygenation IR LEDs must be mounted instead of UV LEDs.
  M4[0][1] = 0;
  M4[1][0] = 0;
  M4[1][1] = 1;
  M4[2][0] = 0;
  M4[2][1] = 0;
  M4[3][0] = 0;
  M4[3][1] = 0;

  M5[0][0] = 0;  // G_LED
  M5[0][1] = 0;
  M5[1][0] = 0;
  M5[1][1] = 0;
  M5[2][0] = 1;
  M5[2][1] = 0;
  M5[3][0] = 0;
  M5[3][1] = 0;

  M6[0][0] = 1;  //Tripple: red, green and UV LEDs.
  M6[0][1] = 0;
  M6[1][0] = 1;
  M6[1][1] = 0;
  M6[2][0] = 1;
  M6[2][1] = 0;
  M6[3][0] = 0;
  M6[3][1] = 0;

  M7[0][0] = 0;  // ICG mode IR LEDs must be mounted instead of GREEN LEDs.
  M7[0][1] = 0;
  M7[1][0] = 0;
  M7[1][1] = 0;
  M7[2][0] = 0;  //No White Light
  M7[2][1] = 0;  //No White Light
  M7[3][0] = 1;
  M7[3][1] = 0;
}

void autofocusToggle() {
  if (autofocusState == 1) {
    lox.startRangeContinuous();
  }
  if (autofocusState == 0) {
    lox.stopRangeContinuous();
  }
}


void setup() {

  modesCacheRefresh();


  pinMode(stepPin_1, OUTPUT);
  pinMode(dirPin_1, OUTPUT);
  pinMode(stepPin_2, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(nMotorsSleep, OUTPUT);
  pinMode(nEnl, OUTPUT);
  pinMode(nEn2, OUTPUT);
  pinMode(nFLT1, INPUT);
  pinMode(nFLT2, INPUT);
  pinMode(k1, INPUT);
  pinMode(k2, INPUT);
  pinMode(k3, INPUT);
  // pinMode(k4, INPUT);
  pinMode(therapyPin, OUTPUT);
  pinMode(k5, INPUT);
  pinMode(k6, OUTPUT);
  pinMode(solenoid_DIR, OUTPUT);
  pinMode(solenoid_ON, OUTPUT);

  //  pinMode(strpbeInput, INPUT_PULLUP); /// Our camera strobe in HIGH - Acquiring, LOW - not acquiring
  pinMode(UV_LED, OUTPUT);     // UV LED
  pinMode(RED_LED, OUTPUT);    // UV LED
  pinMode(GREEN_LED, OUTPUT);  // GREEN LED
  pinMode(IR_LED, OUTPUT);     // GREEN LED
  // pinMode(3, OUTPUT);          // For migalka test
  pinMode(SW_pin, INPUT_PULLUP);

  digitalWrite(GREEN_LED, PWM_IR_min);
  delay(10);
  digitalWrite(UV_LED, PWM_IR_min);   // 4 correct work of interrpt
  digitalWrite(RED_LED, PWM_IR_min);  // 4 correct work of interrpt
  digitalWrite(IR_LED, PWM_IR_min);   // 4 correct work of interrpt
  digitalWrite(UV_LED, PWM_IR_min);   // 4 correct work of interrpt
  //digitalWrite(UV_LED, HIGH);// 4 correct work of interrpt
  //digitalWrite(RED_LED, HIGH);// 4 correct work of interrpt
  Serial.begin(115200);
  Serial.setTimeout(100);
  //  pinMode(strobeInput,INPUT);
  //  attachInterrupt(strobeInput, Strobe_Input_Handler, RISING); // 4 ARDUINO
  attachInterrupt(digitalPinToInterrupt(strobeInput), Strobe_Input_HandlerRise, CHANGE);  // 4 Rpi Pico
                                                                                          // attachInterrupt(digitalPinToInterrupt(k1), Strobe_Input_HandlerFall, FALLING);  // 4 Rpi Pico
  pinMode(strobeInput, INPUT_PULLUP);                                                     // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  // pinMode(strobeInput, INPUT);  // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  // pinMode(k1, INPUT);           // 4 Rpi Pico pull_up must be after the attachinterrupt. It's a bug.
  digitalWrite(solenoid_DIR, LOW);
  digitalWrite(solenoid_ON, LOW);
  motorsCalibration();

  // Serial.println("Adafruit VL53L0X test.");

  //   if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1)) {
  //     Serial.print(F("Starting ITimer1 OK, millis() = "));
  //     Serial.println(millis());

  // #if (TIMER_INTERRUPT_DEBUG > 1)
  //     Serial.print(F("OutputPin1 = "));
  //     Serial.print(outputPin1);
  // #endif
  //   } else
  //     Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  // zoomNsteps(1, maxZoomSteps, fastLag);  // correct N of steps
  // zoomNsteps(0, maxZoomSteps, fastLag);
  // zoomPosition = 0;
  // // maxZoomSteps -= zoomOptimal;
  focusNsteps(1, maxFocusSteps, fastLag);  // correct N of steps dir 1 - to the closest zoom
  focusNsteps(0, maxFocusSteps, fastLag);  // correct N of steps dir 1 - to the closest zoom
  // zoomNsteps(1, optimalZoom, fastLag);
  focusPosition = 0;

  if (!lox.begin()) {
    // Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  // if (autofocusState == 0) {
  // lox.stopRangeContinuous();
  // }
    lox.startRangeContinuous();
    delay(100);
    focusCorrection();
    lox.stopRangeContinuous();
}

void motorsCalibration() {
  digitalWrite(nEnl, LOW);
  digitalWrite(dirPin_1, HIGH);

  digitalWrite(nEn2, LOW);
  digitalWrite(dirPin_2, LOW);

  digitalWrite(nMotorsSleep, LOW);
  //  Motor1 - focus(?)
  //  Motor2 - fzoom(?)
  //    for (int x = 0; x < 65536; x++) {
  //      digitalWrite(stepPin_1, HIGH);
  //      digitalWrite(stepPin_2, HIGH);
  //      delay(5);  // ms (Note : 1000ms = 1sec)
  //      digitalWrite(stepPin_1, LOW);
  //      digitalWrite(stepPin_2, LOW);
  //      delay(5); // ms (Note : 1000ms = 1sec)
  //      Serial.println(x);
  //}
}

void Strobe_Input_HandlerRise() {
  // if(digitalRead(strobeInput) == HIGH)
  // {
  if (counter == 4) {
    counter = 0;
  }
  if (counter == 1) {
    //  analogWrite(UV_LED, PWM_UV);
    //  analogWrite(RED_LED, 0);
    digitalWrite(UV_LED, M[0][0]);
    digitalWrite(RED_LED, M[1][0]);
    digitalWrite(GREEN_LED, M[2][0]);
    digitalWrite(IR_LED, M[3][0]);
    // analogWrite(RED_LED, M[1][0]);
    // analogWrite(GREEN_LED, M[2][0]);
    // analogWrite(IR_LED, M[3][0]);

    // lastTimer2 = millis();
    // digitalWrite(UV_LED, M[0][0]);
    // digitalWrite(RED_LED, M[1][0]);
    // digitalWrite(GREEN_LED, M[2][0]);
    // digitalWrite(IR_LED, M[3][0]);
  }
  if (counter == 0) {
    //   //    analogWrite(UV_LED, 0);
    //   //    analogWrite(RED_LED, PWM_Red);
    // digitalWrite(UV_LED, M[0][1]);
    // digitalWrite(RED_LED, M[1][1]);
    // digitalWrite(GREEN_LED, M[2][1]);
    // //  digitalWrite(IR_LED, LOW);
    // digitalWrite(UV_LED, 0);
    // digitalWrite(RED_LED, 0);
    // digitalWrite(GREEN_LED, 0);
    // digitalWrite(IR_LED, M[3][1]);
    digitalWrite(UV_LED, M[0][1]);
    digitalWrite(RED_LED, M[1][1]);
    digitalWrite(GREEN_LED, M[2][1]);
    digitalWrite(IR_LED, M[3][1]);
  }
  // }
  counter += 1;  // + синхр.
  // else
  // {
  //       analogWrite(UV_LED, 0);
  //   analogWrite(RED_LED,0);
  //   analogWrite(GREEN_LED, 0);
  //   analogWrite(IR_LED, 0);
  // }
}

// void Strobe_Input_HandlerFall() {
//   analogWrite(UV_LED, 0);
//   // analogWrite(RED_LED, 0);
//   // analogWrite(GREEN_LED, 0);
//   // analogWrite(IR_LED, 0);
// }

void waiting_4_command() {
  int PWM_VAL, PWM_VALH, PWM_VALL, PWM_VALlowest;
  cmd = "";
  if (Serial.available()) {
    //    cmd = Serial.readStringUntil('\n');
    cmd = Serial.readString();
    cmd.trim();
  }


  if (cmd.substring(0, 2) == "UV") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_UV = PWM_VAL;
    modesCacheRefresh();
    //        analogWrite(UV_LED, PWM_UV);
    // Serial.println("UV has been changed, modes cache was refreshed");
    // Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "WH") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_GREEN = PWM_VAL;
    modesCacheRefresh();
    analogWrite(GREEN_LED, PWM_GREEN);
    // Serial.println("WH has been changed, modes cache was refreshed");
    // Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "RE") {
    PWM_VALH = cmd[2] - '0';
    PWM_VALL = cmd[3] - '0';
    PWM_VALlowest = cmd[4] - '0';
    PWM_VAL = (PWM_VALH * 100) + (PWM_VALL * 10) + (PWM_VALlowest * 1);
    PWM_Red = PWM_VAL;
    modesCacheRefresh();
    //    analogWrite(RED_LED, PWM_Red);
    // Serial.println("RE has been changed, modes cache was refreshed");
    // Serial.println(PWM_VAL);
  }

  if (cmd.substring(0, 2) == "FC") {
    actualFilter = cmd[2] - '0';
    filterChange(actualFilter);
  }

  if (cmd.substring(0, 5) == "DIST?") {
    distanceMeas();
  }

  if (cmd.substring(0, 3) == "OFF") {
    zoomNsteps(0, maxZoomSteps, fastLag);    // correct N of steps
    focusNsteps(0, maxFocusSteps, fastLag);  // correct N of steps dir 1 - to the closest zoom
  }

  if (cmd.substring(0, 4) == "ZOOM") {
    uint8_t dir;
    if (cmd[4] == '+') {
      dir = 1;
    } else {
      dir = 0;
    }
    zoomNsteps(dir, 100, fastLag);  // correct N of steps
  }

  if (cmd.substring(0, 5) == "FOCUS") {
    uint8_t dir;
    if (cmd[5] == '+') {
      dir = 1;
    } else {
      dir = 0;
    }
    focusNsteps(dir, 100, fastLag);  // correct N of steps
  }

  if (cmd.substring(0, 5) == "AFOFF") {
    autofocusState = 0;
    autofocusToggle();
  }
  if (cmd.substring(0, 5) == "AFON") {
    autofocusState = 1;
    autofocusToggle();
  }
  if (cmd.substring(0, 3) == "AF") {
    lox.startRangeContinuous();
    delay(100);
    focusCorrection();
    lox.stopRangeContinuous();
  }

  if (cmd.substring(0, 4) == "T_ON") {
    digitalWrite(therapyPin, HIGH);
  }

  if (cmd.substring(0, 5) == "T_OFF") {
    digitalWrite(therapyPin, LOW);
  }

  if (cmd.substring(0, 1) == "M") {
    mode = cmd[1] - '0';
    // Serial.println("mode has been changed");
    // Serial.println(mode);
    if (mode == 1) {

      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M1[i][j];
    }
    if (mode == 2) {

      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M2[i][j];
    }
    if (mode == 3) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M3[i][j];
    }
    if (mode == 4) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M4[i][j];
    }
    if (mode == 5) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M5[i][j];
    }
    if (mode == 6) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M6[i][j];
    }
    if (mode == 7) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M7[i][j];
    }
    if (mode == 0) {
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 2; j++)
          M[i][j] = M0[i][j];
    }
  }
}

int distanceMeas(void) {

  if (lox.isRangeComplete()) {
    Serial.print("Distance in mm: ");
    distance = lox.readRange();
    Serial.println(distance);
  }

  // if (vl53.dataReady()) {
  //   // new measurement for the taking!
  //   distance = vl53.distance();
  //   if (distance == -1) {
  //     // something went wrong!
  //     Serial.print(F("Couldn't get distance: "));
  //     Serial.println(vl53.vl_status);
  //     return 0;
  //   }
  // Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.println(" mm");
  // Serial.print("ZOOM ");
  // Serial.println(zoomPosition);
  Serial.print("FOCUS ");
  Serial.println(focusPosition);

  // data is read out, time for another reading!
  // vl53.clearInterrupt();
  // }
  return distance;
}

void filterChange(uint8_t actualFilter) {
  // Serial.println("filter changing");
  // Serial.println(actualFilter);
  digitalWrite(solenoid_ON, HIGH);
  delay(5);
  if (actualFilter == 0) {
    digitalWrite(solenoid_DIR, HIGH);
    delay(5);
  } else {
    digitalWrite(solenoid_DIR, LOW);
    delay(5);
  }
  digitalWrite(solenoid_ON, LOW);
}

void zoom(uint8_t dir, uint8_t lag) {
  // Serial.println("filter switching");
  // Serial.println(actualFilter);
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t zoomCount = 0;
  if (dir == 1) {  //dir 1 - to the closest zoom
    digitalWrite(dirPin_2, HIGH);
    while (analogRead(VAR_Y_pin) >= 767) {
      digitalWrite(stepPin_2, HIGH);
      delay(lag);
      digitalWrite(stepPin_2, LOW);
      delay(lag);
      zoomCount += 1;
      zoomPosition += 1;
      if (zoomPosition >= maxZoomSteps) {
        zoomPosition = maxZoomSteps;
      }
      // Serial.println(zoomPosition);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_2, LOW);
    while (analogRead(VAR_Y_pin) <= 256) {
      digitalWrite(stepPin_2, HIGH);
      delay(lag);
      digitalWrite(stepPin_2, LOW);
      delay(lag);
      zoomCount += 1;
      zoomPosition -= 1;
      if (zoomPosition <= 0) {
        zoomPosition = 0;
      }
      // Serial.println(zoomPosition);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void zoomNsteps(uint8_t dir, int nSteps, int lag) {
  // Serial.println("filter switching");
  // Serial.println(actualFilter);
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t zoomCount = 0;
  if (dir == 1) {  //dir 1 - to the closest zoom
    digitalWrite(dirPin_2, HIGH);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_2, HIGH);
      // delay(lag);
      delayMicroseconds(lag);
      digitalWrite(stepPin_2, LOW);
      // delay(lag);
      delayMicroseconds(lag);
      zoomCount += 1;
      zoomPosition += 1;
      if (zoomPosition >= maxZoomSteps) {
        zoomPosition = maxZoomSteps;
      }
      // Serial.println(zoomCount);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_2, LOW);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_2, HIGH);
      // delay(lag);
      delayMicroseconds(lag);
      digitalWrite(stepPin_2, LOW);
      // delay(lag);
      delayMicroseconds(lag);
      zoomCount += 1;
      zoomPosition -= 1;
      if (zoomPosition <= 0) {
        zoomPosition = 0;
      }
      // Serial.println(zoomCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focus(uint8_t dir, uint8_t lag) {
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t focusCount = 0;
  if (dir == 1) {
    digitalWrite(dirPin_1, HIGH);
    while (analogRead(VAR_X_pin) >= 767) {
      digitalWrite(stepPin_1, HIGH);
      delay(lag);
      digitalWrite(stepPin_1, LOW);
      delay(lag);
      focusCount += 1;
      focusPosition += 1;
      if (focusPosition >= maxFocusSteps) {
        focusPosition = maxFocusSteps;
      }
      // Serial.println(focusPosition);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_1, LOW);
    while (analogRead(VAR_X_pin) <= 256) {
      digitalWrite(stepPin_1, HIGH);
      delay(lag);
      digitalWrite(stepPin_1, LOW);
      delay(lag);
      focusCount += 1;
      focusPosition -= 1;
      if (focusPosition <= 0) {
        focusPosition = 0;
      }
      // Serial.println(focusPosition);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focusNsteps(uint8_t dir, int nSteps, int lag) {
  digitalWrite(nMotorsSleep, HIGH);
  uint32_t focusCount = 0;
  if (dir == 1) {
    digitalWrite(dirPin_1, HIGH);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_1, HIGH);
      // delay(lag);
      delayMicroseconds(lag);
      digitalWrite(stepPin_1, LOW);
      // delay(lag);
      delayMicroseconds(lag);
      focusCount += 1;
      focusPosition += 1;
      if (focusPosition >= maxFocusSteps) {
        focusPosition = maxFocusSteps;
      }
      // Serial.println(focusCount);
    }
  }
  if (dir == 0) {
    digitalWrite(dirPin_1, LOW);
    for (int i = 0; i < nSteps; i++) {
      digitalWrite(stepPin_1, HIGH);
      // delay(lag);
      delayMicroseconds(lag);
      digitalWrite(stepPin_1, LOW);
      // delay(lag);
      delayMicroseconds(lag);
      focusCount += 1;
      focusPosition -= 1;
      if (focusPosition <= 0) {
        focusPosition = 0;
      }
      // Serial.println(focusCount);
    }
  }
  digitalWrite(nMotorsSleep, LOW);
}

void focusCorrection() {
  lox.startRangeContinuous();
  int dir;
  int distance = distanceMeas();
  if (distance > 1000) {
    return;
  }
  // int distanceRange = round(distance / 10);
  // int zoomPositionRange = round(zoomPosition / 10);
  float correctFocus;
  int deltaFocus;
  int steps;

  // correctFocus = -2473.5 + 1.7036 * distance + 1.867 * zoomPosition + 0.02818 * distance * distance - 0.0073557 * zoomPosition * distance + 0.00021246 * zoomPosition * zoomPosition; //function of two variables
  // correctFocus = -0.0012 * distance * distance * distance + 0.4607 * distance * distance - 61.597 * distance + 3126.5;  //function of the one variable
  correctFocus = 0.0000005 * distance * distance * distance * distance - 0.0005 * distance * distance * distance + 0.19 * distance * distance - 32.961 * distance + 3260.6;  //function of the one variable
  if ((correctFocus - focusPosition) >= 0) {
    deltaFocus = correctFocus - focusPosition;
    dir = 1;
  } else {
    deltaFocus = focusPosition - correctFocus;
    dir = 0;
  }
  if (abs(deltaFocus) < deltaFocusThresh) {
    // Serial.println("No need 4 correction");
    return;
  }
  // Serial.println("doing correction");
  // Serial.print("correct focus = ");
  // Serial.println(correctFocus);
  // Serial.print("current focus = ");
  // Serial.println(focusPosition);
  // Serial.print("dir = ");
  // Serial.println(dir);
  // Serial.print("deltaFocus = ");
  // Serial.println(deltaFocus);
  focusNsteps(dir, deltaFocus, fastLag);
  // Serial.println(focusCorrected);
  focusCorrected = 1;
  // Serial.println(" focus was corrected");
  // lox.stopRangeContinuous();
}

void loop() {
  static unsigned long lastTimer1 = 0;
  static bool timer1Stopped = false;


  if (millis() - lastTimer1 > TIMER_INTERVAL_MS) {
    lastTimer1 = millis();
    // if (focusCorrected == 0) {
    if (autofocusState == 1) {
      focusCorrection();
      // distance = distanceMeas();
    }
  }
  //     if (millis() - lastTimer2 > ExposureMS) {

  // analogWrite(UV_LED, 0);
  // analogWrite(RED_LED, 0);
  // analogWrite(GREEN_LED, 0);
  // analogWrite(IR_LED, 0);

  // }
  // if (timer1Stopped) {
  //   Serial.print(F("Start ITimer1, millis() = "));
  //   Serial.println(millis());
  //   // focusCorrected = 0;

  //   ITimer1.restartTimer();
  //   // distanceMeas();



  //   // filterChange(actualFilter);
  // } else {


  //   Serial.print(F("Stop ITimer1, millis() = "));
  //   Serial.println(millis());
  //   ITimer1.stopTimer();
  // }
  // timer1Stopped = !timer1Stopped;
  // Serial.println("timer restarted");
  // }

  // timer1Stopped = timer1Stopped;

  // focusNsteps(0, 500, fastLag);
  // delay(1000);
  // focusNsteps(1, 500, fastLag);
  // delay(1000);
  // zoomNsteps(0, 500, fastLag);
  // delay(1000);
  // zoomNsteps(1, 500, fastLag);
  // delay(1000);


  // Serial.print("Distance = ");
  // Serial.println(distance);
  // delay(1000);
  if(digitalRead(SW_pin) == LOW)
  {
    lox.startRangeContinuous();
    delay(100);
    focusCorrection();
    lox.stopRangeContinuous();
  }
  VAR_X = analogRead(VAR_X_pin);
  // VAR_Y = analogRead(VAR_Y_pin);

  // if ((VAR_Y >= 767)) {
  //   zoom(1, 2);
  // }
  // if (VAR_Y <= 256) {
  //   zoom(0, 2);
  // }

  if ((VAR_X >= 767)) {
    focus(1, 2);
  }
  if (VAR_X <= 256) {
    focus(0, 2);
  }

  //  Serial.print("X = ");
  //  Serial.print(VAR_X);
  //  Serial.print("\t Y = ");
  //  Serial.println(VAR_Y);

  //  delay(20);
  // Serial.println(counter);
  // digitalWrite(3, HIGH);
  //  filterChange(0);
  // delay(500);
  //  digitalWrite(3, LOW);
  //  filterChange(1);
  // delay(500);
  if (Serial.available()) {
    waiting_4_command();
  }
}