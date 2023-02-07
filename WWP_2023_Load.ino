#include <Wire.h>               //Include library for UART communication
#include <Adafruit_MCP4725.h>   //Include DAC library
#include <Adafruit_INA260.h>    //Include power sensor library
#include <PA12.h>
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_MCP4725 dac;           //Initailize DAC

//Linear Actuators
#define ID_NUM 0
#define Linear_Actuator_Enable 16
#define Linear_Actuator_Tx 1
#define Linear_Actuator_Rx 0
#define RPM_Pin 29
#define PCC_Relay_Pin 33
#define EStop_Pin 11
#define PCC_Disconnect_Pin 30

PA12 myServo(&Serial1, Linear_Actuator_Enable, 1);
//          (&Serial, enable_pin,  Tx Level)

//Adjustable Variables
uint16_t regulate_RPM_Setpoint = 4000;
uint16_t regulate_Power_Setpoint = 25000;//(mW)
float optimal_Theta = 20;
float cutin_Theta = 25;
float brake_Theta = 95;
float DAC_Voltage_Cutin = 0;

//Active Pitch Variables
uint16_t theta_Position;
float theta;
//INA260 Reading Variables
uint16_t L_Power = 0;   //Load Power (mW)
uint16_t L_Power_Previous = 0;
uint16_t L_Voltage = 0; //Load Voltage (mV)
uint16_t L_Voltage_Previous = 0;
uint16_t L_Current = 0; //Load Current (mA)
uint16_t L_Current_Previous = 0;

//RPM Reading Variables
volatile uint16_t RPM_Filtered;
volatile uint16_t RPM_Raw;
volatile uint16_t RPM_Raw_Previous;
volatile unsigned long TempStore_RPM = 0;
volatile bool FirstRead_RPM = true;
volatile unsigned long Saved_RPM [10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile unsigned long RPMSum = 0;
volatile unsigned long Htime1, Htime2;
volatile int i = 0;
volatile int F = 0;

//Load Variables
float DAC_Voltage;
float DAC_Voltage_Previous;
float DAC_Perturbation = 10;
float DAC_Voltage_Maximum = 3300;//(mV)

//State Machine Variables
enum States {StartUp, Optimize, Regulate, EStop_Safety, Discontinuity_Safety, Safety_Restart};
States State = StartUp;
enum PCC_Disconnect_States {Wait, RelayOn, Reconnection};
PCC_Disconnect_States PCC_Disconnect_State = Wait;

//Timer Intervals
unsigned Fast_Interval = 10;
unsigned Medium_Interval = 50;
unsigned Slow_Interval = 500;
unsigned Safety_Restart_Interval = 5000;
unsigned Pitch_Transient_Interval = 100;
//Timers
unsigned long Timer_Fast;
unsigned long Timer_Medium;
unsigned long Timer_Slow;
unsigned long Timer_Safety_Restart;
unsigned long Timer_RPM_Transient;

bool load_Optimize_Enable = true;
bool E_Stop;
bool PCC_Disconnected;
bool PCC_Relay;
bool state_Machine_Enable = false;




void setup() {
  //Linear Actuator
  myServo.begin(32);
  myServo.movingSpeed(ID_NUM, 750);
  Serial.begin(9600);           //Begin serial monitor
  dac.begin(0x66);              //Initialize DAC
  ina260.begin();               //Initialize INA 260

  initialize_Pins();

  PCC_Relay = true;
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
  delay (500);
  theta = cutin_Theta;
  set_Theta();
  delay (2000);
  PCC_Relay = false;
  DAC_Voltage = DAC_Voltage_Cutin;

  initialize_Timers();
}

void loop() {
  if (millis() - Timer_Fast >= Fast_Interval)
  {
    Timer_Fast = millis();
    read_EStop();
    read_PCC_Disconnect();
    manage_State();
  }

  if (millis() - Timer_Medium >= Medium_Interval)
  {
    Timer_Medium = millis();
  }

  if (millis() - Timer_Slow >= Slow_Interval)
  {
    Timer_Slow = millis();
    read_INA260();
    optimize_Load();
    set_Load();
    PC_Comms ();
  }
}

void manage_State() {
  if (state_Machine_Enable) {
    switch (State)
    {
      case StartUp:
        if (RPM_Filtered > 500)
        {
          State = Optimize;
          load_Optimize_Enable = true;
          theta = optimal_Theta;
        }
        break;

      case Optimize:
        if (E_Stop)
        {
          State = EStop_Safety;
        }
        if (PCC_Disconnected)
        {
          State = Discontinuity_Safety;
        }
        if (RPM_Filtered > regulate_RPM_Setpoint)
        {
          State = Regulate;
        }
        break;

      case Regulate:
        regulate_RPM();
        if (E_Stop)
        {
          State = EStop_Safety;
        }
        if (PCC_Disconnected)
        {
          State = Discontinuity_Safety;
        }
        if (RPM_Filtered < 0.95 * regulate_RPM_Setpoint)
        {
          State = Optimize;
          theta = optimal_Theta;
        }
        break;

      case EStop_Safety:
        PCC_Relay = true;
        theta = brake_Theta;
        load_Optimize_Enable = false;
        if (!E_Stop)
        {
          State = Safety_Restart;
        }
        break;

      case Discontinuity_Safety:
        PCC_Relay = true;
        theta = brake_Theta;
        load_Optimize_Enable = false;
        if (!PCC_Disconnected)
        {
          State = Safety_Restart;
        }
        break;

      case Safety_Restart:
        theta = cutin_Theta;
        if (millis() - Timer_Safety_Restart >= Safety_Restart_Interval)
        {
          Timer_Safety_Restart = millis();
          PCC_Relay = false;
          State = Optimize;
        }
        break;

      default:
        State = StartUp;
        load_Optimize_Enable = false;
        break;
    }
  }
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
  set_Theta();
}

void initialize_Timers() {
  Timer_Fast = millis();
  Timer_Medium = millis();
  Timer_Slow = millis();
  Timer_Safety_Restart = millis();
  Timer_RPM_Transient = millis();
}
void initialize_Pins() {
  //pinMode(Linear_Actuator_Tx, OUTPUT);
  //pinMode(Linear_Actuator_Rx, INPUT);
  //pinMode(Linear_Actuator_Enable, OUTPUT);
  pinMode(RPM_Pin, INPUT);
  pinMode(PCC_Relay_Pin, OUTPUT);
  pinMode(EStop_Pin, INPUT);
  pinMode(PCC_Disconnect_Pin, INPUT);
}

void read_EStop() {
  E_Stop = digitalRead(EStop_Pin);
}

void read_PCC_Disconnect() {
  switch (PCC_Disconnect_State)
  {
    case Wait:
      if (digitalRead(PCC_Disconnect_Pin) && ina260.readBusVoltage() < 500) {
        PCC_Disconnected = true;
        PCC_Disconnect_State = RelayOn;
      }
      break;
    case RelayOn:
      if (!digitalRead(PCC_Disconnect_Pin) && ina260.readBusVoltage() > 500) {
        PCC_Disconnect_State = Reconnection;
      }
      break;
    case Reconnection:
      if (digitalRead(PCC_Disconnect_Pin) && ina260.readBusVoltage() > 500) {
        PCC_Disconnected = false;
        PCC_Disconnect_State = Wait;
      }
    default:
      PCC_Disconnect_State = Wait;
  }
}

void read_INA260() {
  L_Power_Previous = L_Power;
  L_Voltage_Previous = L_Voltage;
  L_Current_Previous = L_Current;
  L_Power = ina260.readPower();
  L_Voltage = ina260.readBusVoltage();
  L_Current = ina260.readCurrent();

}

void regulate_RPM() {
  if (millis() - Timer_RPM_Transient >= Pitch_Transient_Interval)
  {
    Timer_RPM_Transient = millis();
    if (RPM_Filtered < regulate_RPM_Setpoint || RPM_Filtered > 1.05 * regulate_RPM_Setpoint)
    {
      theta = theta + 0.003 * (RPM_Filtered - regulate_RPM_Setpoint);
    }
  }
}

void set_Load() {
  if (DAC_Voltage > 3300) {
    DAC_Voltage = 3300;
  }
  if (DAC_Voltage < 0) {
    DAC_Voltage = 0;
  }
  dac.setVoltage(((DAC_Voltage + (DAC_Voltage_Maximum * 0.5) / 4096) / DAC_Voltage_Maximum) * (4095), false);   //Set DAC voltage
}

void set_Theta()
{
  if (theta > 95.0) {
    theta = 95.0;
  }
  if (theta < optimal_Theta) {
    theta = optimal_Theta;
  }
  theta_Position = (int)(31.3479 * theta + 208.084);
  myServo.goalPosition(ID_NUM, theta_Position);
}

void read_RPM() {
  attachInterrupt(digitalPinToInterrupt(RPM_Pin), RPM_Interupt, RISING);    //run ISR on rising edge
}

void PC_Comms () {
  //---------------------------------------------------------------------------------------
  if (Serial) // check performance cost on checking if serial is active
  {
    Serial.print("(p) PCC Relay: ");
    Serial.println(PCC_Relay ? "On" : "Off");

    Serial.print("(s) State Machine: ");
    Serial.println(state_Machine_Enable ? "Enabled" : "Disabled");

    Serial.print("(o) Load Optimization: ");
    Serial.println(load_Optimize_Enable ? "Enabled" : "Disabled");

    Serial.print("RPM: ");
    Serial.println(RPM_Filtered);

    Serial.print("Load Voltage: ");
    Serial.print(L_Voltage);
    Serial.println(" mV");

    Serial.print("Load Current: ");
    Serial.print(L_Current);
    Serial.println(" mA");

    Serial.print("Load Power: ");
    Serial.print(L_Power);
    Serial.println(" mW");

    Serial.print("State: ");
    switch (State)
    {
      case StartUp:
        Serial.println("StartUp");
        break;

      case Optimize:
        Serial.println("Optimize");
        break;

      case Regulate:
        Serial.println("Regulate");
        break;

      case EStop_Safety:
        Serial.println("EStop_Safety");
        break;

      case Discontinuity_Safety:
        Serial.println("Discontinuity_Safety");
        break;

      case Safety_Restart:
        Serial.println("Safety_Restart");
        break;

      default:
        Serial.println("Error");
        break;
    }

    Serial.print("Emergency Switch: ");
    Serial.println(E_Stop ? "On" : "Off");

    Serial.print("(t) Theta (0 - 95): ");
    Serial.println(theta);

    Serial.print("(v) Load DAC Voltage ( 0mV - 3300mV ): ");
    Serial.print(DAC_Voltage);
    Serial.println(" mV");
  }
  if (Serial.available() > 0)
  {
    uint8_t cmd = Serial.read();

    switch (cmd)
    {
      case 's':
        state_Machine_Enable = !state_Machine_Enable;
        break;

      case 'o':
        load_Optimize_Enable = !load_Optimize_Enable;
        break;

      case 'p':
        PCC_Relay = !PCC_Relay;
        digitalWrite(24, PCC_Relay);
        break;

      case 't':
        theta = Serial.parseFloat();
        break;

      case 'v':
        DAC_Voltage = Serial.parseFloat();
        break;

      default:
        Serial.println("Command not recognized");
        break;
    }
  }
  Serial.println();
  Serial.println();
  Serial.println();
}

void optimize_Load() {
  if (load_Optimize_Enable) {
    if (L_Power > regulate_Power_Setpoint) {                                           //Check if power regulation is needed
      if ( DAC_Voltage >= DAC_Perturbation) {
        DAC_Voltage_Previous = DAC_Voltage;
        DAC_Voltage = DAC_Voltage - DAC_Perturbation;                         //Regulate power
      }
    }
    else {
      if ( DAC_Voltage <= DAC_Perturbation) {
            DAC_Voltage = DAC_Voltage + DAC_Perturbation;                     //Increase DAC voltage
          }
      else if ( DAC_Voltage >= DAC_Voltage_Maximum - DAC_Perturbation) {
            DAC_Voltage = DAC_Voltage - DAC_Perturbation;                     //Increase DAC voltage
          }
      else if (L_Power - L_Power_Previous > 0) {                                   //Check if power increased
        if (DAC_Voltage - DAC_Voltage_Previous >= 0) {                        //Check positive or negative adjustment
          DAC_Voltage_Previous = DAC_Voltage;                                 //Save last DAC voltage
          //if ( DAC_Voltage <= DAC_Voltage_Maximum - DAC_Perturbation) {
            DAC_Voltage = DAC_Voltage + DAC_Perturbation;                     //Increase DAC voltage
//          }
//         else {
//            DAC_Voltage = DAC_Voltage - DAC_Perturbation;                     //Decrease DAC voltage
//          }
        }
        else if (DAC_Voltage - DAC_Voltage_Previous < 0) {                    //Check positive or negative adjustment
          DAC_Voltage_Previous = DAC_Voltage;                                 //Save last DAC voltage
          //if ( DAC_Voltage >= DAC_Perturbation) {
            DAC_Voltage = DAC_Voltage - DAC_Perturbation;                     //Decrease DAC voltage
//          }
//          else {
//            DAC_Voltage = DAC_Voltage + DAC_Perturbation;                     //Increase DAC voltage
//          }
        }
      }
      else if (L_Power - L_Power_Previous <= 0) {                             //Check if power decreased
        if (DAC_Voltage - DAC_Voltage_Previous > 0) {                         //Check positive or negative adjustment
          DAC_Voltage_Previous = DAC_Voltage;                                 //Save last DAC voltage
          //if ( DAC_Voltage >= DAC_Perturbation) {
            DAC_Voltage = DAC_Voltage - DAC_Perturbation;                     //Decrease DAC voltage
//          }
//          else {
//            DAC_Voltage = DAC_Voltage + DAC_Perturbation;                     //Increase DAC voltage
//          }
        }
        else if (DAC_Voltage - DAC_Voltage_Previous <= 0) {                   //Check positive or negative adjustment
          DAC_Voltage_Previous = DAC_Voltage;                                 //Save last DAC voltage
          //if ( DAC_Voltage <= DAC_Voltage_Maximum - DAC_Perturbation) {
            DAC_Voltage = DAC_Voltage + DAC_Perturbation;                     //Increase DAC voltage
//          }
//          else {
//            DAC_Voltage = DAC_Voltage - DAC_Perturbation;                     //Decrease DAC voltage
//          }
        }
      }
    }
  }
}

void RPM_Interupt() {
  TempStore_RPM = micros();
  if (digitalRead(RPM_Pin) == HIGH) {
    if (FirstRead_RPM) {
      Htime1 = TempStore_RPM;                                               //capture first rising edge time
      FirstRead_RPM = false;
    }
    else {
      Htime2 = TempStore_RPM;                                               //capture second rising edge time
      FirstRead_RPM = true;
      //noInterrupts();
      detachInterrupt(digitalPinToInterrupt(RPM_Pin));
      RPM_Raw = 15000000 / (Htime2 - Htime1);                               //calculate rpm from period
      if (RPM_Raw < 6000) {
        if (abs(RPM_Raw - RPM_Raw_Previous) < 600) {                        //compare new rpm to previous
          if ( i < 9 ) {                                                    //cycle through array
            i++;
          }
          else {
            i = 0;
          }
          RPM_Raw_Previous = RPM_Raw;                                       //set comparative value
          RPMSum = RPMSum - Saved_RPM [i];                                  //subtract oldest value from sum
          Saved_RPM [i] = RPM_Raw;                                          //insert new value into array
          RPMSum = RPMSum + RPM_Raw;                                        //add new value to sum
          RPM_Filtered = (RPMSum / 10);
          F = 0;                                                            //reset number of fails
        }
        else {
          F ++;                                                             //increment failed compared value
          if ( F > 5 ) {
            RPM_Raw_Previous = RPM_Raw;                                     //reset the comparative value
          }
        }
      }
    }
  }
}
