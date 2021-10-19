/* *********************************************************************************** */
/*                                                                                     */
/*  Copyright 2021 by Bodo Bauer <bb@bb-zone.com>                                      */
/*                                                                                     */
/*  This program is free software: you can redistribute it and/or modify               */
/*  it under the terms of the GNU General Public License as published by               */
/*  the Free Software Foundation, either version 3 of the License, or                  */
/*  (at your option) any later version.                                                */
/*                                                                                     */
/*  This program is distributed in the hope that it will be useful,                    */
/*  but WITHOUT ANY WARRANTY; without even the implied warranty of                     */
/*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      */
/*  GNU General Public License for more details.                                       */
/*                                                                                     */
/*  You should have received a copy of the GNU General Public License                  */
/*  along with this program.  If not, see <http://www.gnu.org/licenses/>.              */
/* *********************************************************************************** */
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "APDS9930.h" 

/* *********************************************************************************** */
/* Debug Macros                                                                        */
/* *********************************************************************************** */
// #define DEBUG

#ifdef DEBUG
#define PRINT(a)      Serial.print(a)
#define PRINTHEX(a)   Serial.print(a, HEX)
#define PRINTLN(a)    Serial.println(a)
#define PRINTHEXLN(a) Serial.println(a, HEX)
#else
#define PRINT(a)
#define PRINTLN(a)
#define PRINTHEX(a)
#define PRINTHEXLN(a)
#endif

/* *********************************************************************************** */
/* Custom Types                                                                        */
/* *********************************************************************************** */
typedef struct {                 // Tasks to be regulary scheduled
    unsigned long last_time;     // time of last call
    unsigned long interval;      // cycle time to run task
    void (*task)(void*);         // pointer to task handler
    void *argument;              // call argument
} task_t;

/* *********************************************************************************** */
/* GPIOs                                                                               */
/* *********************************************************************************** */
#define PIN_LID_SERVO      D8   // lid opener servo
#define PIN_SWITCH_SERVO   D0   // switch manipulator servo
#define PIN_SWITCH         D5   // switch
#define PIN_STATUS_LED     D4   // on board LED

#define PIN_I2C_SDA        D2   // I2C bus   
#define PIN_I2C_SCL        D1   // I2C bus
#define PIN_SENSOR_INT     D6   // apdp interrupt pin (unused)

/* *********************************************************************************** */
/* Servo positions                                                                     */
/* *********************************************************************************** */
#define SWITCH_PARK_POSITION     0
#define SWITCH_END_POSITION    195
#define SWITCH_HALF_POSITION   130
#define LID_PARK_POSITION       90
#define LID_END_POSITION        00

#define MOVE_FAST             true
#define MOVE_SLOW            false
#define SLOW_STEP_SIZE           2
#define SLOW_DELAY              40

#define POXIMITIY_THESHOLD     500

/* *********************************************************************************** */
/* Globals                                                                             */
/* *********************************************************************************** */
bool  stateOnOff;
bool  stateProximity;

Servo    servoSwitch, servoLid;
APDS9930 apds = APDS9930();

/* *********************************************************************************** */
/* Prototypes                                                                          */
/* *********************************************************************************** */
void readSwitch(void*);
void parkServos(void);
void switchOff(bool fast);
void attachServoSwitch(void);
void attachServoLid(void);
void readApds(void*);
void apdsInit(void);
void lidOpen(bool fast);
void lidClose(bool fast);

/* *********************************************************************************** */
/* @brief What to do when the switch in On - pick one...                               */
/* *********************************************************************************** */
void doNothing(void);
void turnOffFast(void);
void turnOffSlow(void);
void openLidTurnOffFast(void);

void (*reaction[])(void) = {
    turnOffSlow,
    turnOffSlow,

    turnOffFast,
    turnOffFast,
    turnOffFast,

    openLidTurnOffFast,
    openLidTurnOffFast,
};
#define NUM_REACTION 7;

/* *********************************************************************************** */
/* @brief What to do when someone comes close - pick one...                            */
/* *********************************************************************************** */
void openLid(void);
void openLidMoveArm(void);
void clapLid(void);

void (*lidReaction[])(void) = {
    doNothing,

    clapLid,
    clapLid,

    openLidMoveArm,
    openLidMoveArm,

    openLid,
    openLid,
    openLid,
};
#define NUM_LID_REACTION 8;

/* *********************************************************************************** */
/* @brief Schedule Table                                                               */
/* *********************************************************************************** */
task_t taskTable[] = {
  //   last run,  interval, task, argument
  {0L,   250L, readSwitch, NULL},
  {0L,   250L, readApds,  NULL},

  // end marker
  {0, 0, NULL, NULL},
};

/* *********************************************************************************** */
/* @brief Process task list                                                            */
/* *********************************************************************************** */
void runTasks(task_t *tasks) {
    unsigned long  current_time = millis();
    static unsigned long  last_time = 0L;

    // do not run multiple times within a millisecond
    if ( current_time != last_time ) {        
        // process tasks table
        for (int index = 0; tasks[index].task != NULL; index++) {
            if (current_time - tasks[index].last_time >= tasks[index].interval) {
                // run task
                (tasks[index].task)(tasks[index].argument);
                
                // remember last time this task has been run
                tasks[index].last_time = current_time;
            }
        }
        last_time = millis();
    }
}

/* *********************************************************************************** */
/* @brief Move servos to parkposition and deactivate them                              */
/* *********************************************************************************** */
void parkServos(void) {
  attachServoSwitch();
  attachServoLid();

  servoSwitch.write(SWITCH_PARK_POSITION);
  servoLid.write(LID_PARK_POSITION);

  delay(400);

  servoSwitch.detach();
  servoLid.detach();
}

/* *********************************************************************************** */
/* @brief open lid                                                                     */
/* *********************************************************************************** */
void lidOpen(bool fast) {
  attachServoLid();
  if (fast == MOVE_FAST) {
    servoLid.write(LID_END_POSITION);
  }
  else {
    for (int pos = LID_PARK_POSITION; pos <= LID_END_POSITION; pos += SLOW_STEP_SIZE) {
      servoLid.write(pos);
      delay(SLOW_DELAY);
    }
    servoLid.write(LID_END_POSITION);
  }
}

/* *********************************************************************************** */
/* @brief close lid                                                                     */
/* *********************************************************************************** */
void lidClose(bool fast) {
  attachServoLid();
  if (fast == MOVE_FAST) {
    servoLid.write(LID_PARK_POSITION);
  }
  else {
    for (int pos = LID_PARK_POSITION; pos <= LID_PARK_POSITION; pos += SLOW_STEP_SIZE) {
      servoLid.write(pos);
      delay(SLOW_DELAY);
    }
    servoLid.write(LID_PARK_POSITION);
  }
}

/* *********************************************************************************** */
/* @brief Turn On/Off Switch to Off position                                           */
/* *********************************************************************************** */
void switchOff(bool fast) {
  attachServoSwitch();
  if (fast == MOVE_FAST) {
    servoSwitch.write(SWITCH_END_POSITION);
  }
  else {
    for (int pos = SWITCH_PARK_POSITION; pos <= SWITCH_END_POSITION; pos += SLOW_STEP_SIZE) {
      servoSwitch.write(pos);
      delay(SLOW_DELAY);
    }
    servoSwitch.write(SWITCH_END_POSITION);
  }
}

/* *********************************************************************************** */
/* @brief Activate Switch Servo                                                        */
/* *********************************************************************************** */
void attachServoSwitch(void) {
  if (!servoSwitch.attached()) {
    servoSwitch.attach(PIN_SWITCH_SERVO, 1000, 2300);
  }
}

/* *********************************************************************************** */
/* @brief Activate Lid Servo                                                           */
/* *********************************************************************************** */
void attachServoLid(void) {
  if (!servoLid.attached()) {
    servoLid.attach(PIN_LID_SERVO, 1000, 2300);
  }
}

/* *********************************************************************************** */
/* @brief Initialize procximity sensor                                                 */
/* *********************************************************************************** */
void apdsInit(void) {
  // Initialize APDS-9930 (configure I2C and initial values)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (apds.init()) {
    PRINTLN("APDS-9930 initialization complete");
  }
  else {
    PRINTLN("Something went wrong during APDS-9930 init!");
  }

  // Start running the APDS-9930 proximity sensor (no interrupts)
  if (apds.enableProximitySensor(false)) {
    PRINTLN("Proximity sensor is now running");
  }
  else {
    PRINTLN("Something went wrong during sensor init!");
  }
}

/* *********************************************************************************** */
/* @brief Read proximity sensor and react to something getting close                  */
/* *********************************************************************************** */
void readApds(void*) {
  uint16_t proximity_data = 0;
  bool state = stateProximity;
  bool stateSwitch = digitalRead(PIN_SWITCH);

  // do nothing if switch in ON
  if (stateSwitch) {
    // Read the proximity value
    if (!apds.readProximity(proximity_data)) {
      PRINTLN("Error reading proximity value");
    }
    else {
      if (proximity_data > POXIMITIY_THESHOLD) {
        state = true;
        PRINTLN("Someone is getting close...");
      }
      else {
        PRINTLN("Thread went away again");
        state = false;
      }
    }

    if (state != stateProximity) {
      stateProximity = state;
      if (stateProximity) {
        int funcInd = random(100) % NUM_LID_REACTION;
        (*lidReaction[funcInd])();
      }
      else {
        parkServos();
      }
    }
  }
}

/* *********************************************************************************** */
/* @brief Clap with the lid                                                            */
/* *********************************************************************************** */
void clapLid(void) {
  lidOpen(MOVE_FAST);
  delay(70);
  lidClose(MOVE_FAST);
  delay(70);
  lidOpen(MOVE_FAST);
  delay(70);
  lidClose(MOVE_FAST);
}

/* *********************************************************************************** */
/* @brief Don't do anything                                                            */
/* *********************************************************************************** */
void doNothing(void) {
  delay(10);
}

/* *********************************************************************************** */
/* @brief Open the Lid                                                                 */
/* *********************************************************************************** */
void openLid(void) {
  lidOpen(MOVE_FAST);
}

/* *********************************************************************************** */
/* @brief Open the lid and move arm to half position                                   */
/* *********************************************************************************** */
void openLidMoveArm(void) {
  lidOpen(MOVE_FAST);
  attachServoSwitch();
  servoSwitch.write(SWITCH_HALF_POSITION);
}

/* *********************************************************************************** */
/* @brief Initialize System                                                            */
/* *********************************************************************************** */
void setup(void) {
#ifdef DEBUG
  Serial.begin(115200);
  PRINTLN("");
#endif

  parkServos();

  // On/Off Switch
  pinMode(PIN_SWITCH, INPUT);
  stateOnOff = digitalRead(PIN_SWITCH);

  // Status LED
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, stateOnOff);

  // proximity sensor
  apdsInit();

  // seed random number generator
  randomSeed(analogRead(0));
}

/* *********************************************************************************** */
/* @brief Turn On/Off switch off with fast movement                                    */
/* *********************************************************************************** */
void turnOffFast(void) {
  switchOff(MOVE_FAST);
}

/* *********************************************************************************** */
/* @brief Open lid, Turn On/Off switch off with slow movement                          */
/* *********************************************************************************** */
void turnOffSlow(void) {
  lidOpen(MOVE_FAST);
  switchOff(MOVE_SLOW);
}

/* *********************************************************************************** */
/* @brief Open lid, wait, turn On/Off switch off with fast movement                    */
/* *********************************************************************************** */
void openLidTurnOffFast(void) {
  lidOpen(MOVE_FAST);
  delay(400);
  switchOff(MOVE_FAST);
}

/* *********************************************************************************** */
/* @brief Read On/Off switch and set status LED                                         */
/* *********************************************************************************** */
void readSwitch(void*) {
  bool state = digitalRead(PIN_SWITCH);
  static int funcInd = 0;

  if (state == stateOnOff) {
    stateOnOff = !state;
    digitalWrite(PIN_STATUS_LED, !stateOnOff);

    if (stateOnOff) {
      PRINTLN("Switch: On");
      funcInd = random(100) % NUM_REACTION;
      (*reaction[funcInd])();
    }
    else {
      PRINTLN("Switch: Off");
      parkServos();
    }
  }
}

/* *********************************************************************************** */
/* @brief From Her to Eternity                                                         */
/* *********************************************************************************** */
void loop() {
  runTasks(taskTable);
}