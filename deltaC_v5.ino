#include <SPI.h>
#include <Wire.h>
#include <C:\Users\PC 4\Documents\Arduino\Support\ioFunctions\ioFunctions.ino>
#include <C:\Users\PC 4\Documents\Arduino\Support\ValveControllerComm\ValveControllerComm.ino>
#include <C:\Users\PC 4\Documents\Arduino\Support\OdorSetupFunctions\OdorSetupFunctions.ino>


// command definitions
#define SET_VALVE      1
#define SET_VIAL       2
#define VALVE_STATUS   3
#define VIAL_STATUS    4
#define aSET_MFC       5
#define READ_MFC       6
#define VIAL_ON        7
#define VIAL_OFF       8    
#define SET_ANALOG     9
#define READ_ANALOG   10
#define SET_DIGITAL   11
#define READ_DIGITAL  12
#define MODE_DIGITAL  13
#define sSET_MFC      14

#define WATER1     29
#define SOLENOID2  28
#define SOLENOID3  27
#define SOLENOID4  26
#define SOLENOID5  25
#define SOLENOID6  24
#define SOLENOID7  23
#define SOLENOID8  22

#define LICK1     37
#define LICK2     36

#define ADC_PIN   49
#define DAC1_PIN  53
#define DAC2_PIN  48

#define DIGITAL1  62
#define DIGITAL2  63
#define DIGITAL3  64
#define DIGITAL5  66

#define device 1
#define MFC_Air 1
#define MFC_N2 2

#define OFF 0
#define ON  1
#define FV 1
#define TrialTrig  62  // Dig 1
#define FVTrig     63  // Dig 2
#define OdorTrig   64  // Dig 3
#define LickTrig   65  // Dig 4
#define SniffTrig  66  // Dig 5
#define airVacTrig 67  // Dig 6

#define device 1
#define dummy 4
#define airVacDummy 13
#define sniffIn  1 // sniff input - AIN1 channel
#define sniffOut  2 // sniff output on AOUT2 channel

//parameter declaration

char buffer[128];
uint8_t idx = 0;
char *argv[10];
int arg1, arg2, arg3;
uint8_t txbuffer[64];
int state = 0;
int ain, aout, resp;
uint8_t odorValve, airVacValve, rewarded, res, waterDur, no_sniff;
int lickTime;

uint8_t freq = 2;
uint8_t flow;
float pi = 3.14;
char *stim;
boolean phase = true;

int threshold = 1500;

uint8_t sniffcycle = 0;
uint8_t lickCount  = 0;

unsigned long currentTime, lastSniff, fvOnTime, currLickTime;
unsigned long startTime = 0;
unsigned long nextCrossingTime;
unsigned long prevCrossingTime = 0;
unsigned long waterOnTime, waterOffTime;

uint16_t  ITI, odorFlow;
uint8_t dig_state, lick_state;

void parse(char *line, char **argv, uint8_t maxArgs) {
  uint8_t argCount = 0;
  while (*line != '\0') {       // if not the end of line .......
    while (*line == ',' || *line == ' ' || *line == '\t' || *line == '\n')
      *line++ = '\0';     // replace commas and white spaces with 0
    *argv++ = line;          // save the argument position
    argCount++;
    if (argCount == maxArgs - 1)
      break;
    while (*line != '\0' && *line != ',' && *line != ' ' &&
           *line != '\t' && *line != '\n')
      line++;             // skip the argument until ...
  }
  *argv = '\0';                 // mark the end of argument list
}

int LickCheck(){
    
    resp = digitalRead(LICK1);
    if (resp == LOW){
      digitalWrite(LickTrig, LOW);
      resp = SetDigital(device, LickTrig-61, OFF);
    }
    else {
      digitalWrite(LickTrig, HIGH);
      resp = SetDigital(device, LickTrig-61, ON);
    }  
}

// This function counts the number of sniff cycles
int SniffCheck(boolean inhal) {
  
  int sniffvalue;
//  time = millis();
//  sniffvalue = int(32000*sin(time*freq*4*pi));
//  dac2Write(sniffIn, sniffvalue); // sniff output on AOUT1 channel
  sniffvalue = (16 * adcRead(sniffIn - 1, 0))+threshold; // 12 bit read
  dac2Write(sniffOut-1, sniffvalue); // sniff output on AOUT2 channel
  //Serial.println(sniffvalue);
  
  if ((sniffvalue > 0) && (inhal == true) ) {
    nextCrossingTime = millis();
    if ((nextCrossingTime - prevCrossingTime) > 20) {
      sniffcycle += 1;
      digitalWrite(SniffTrig, LOW);
      resp = SetDigital(device, SniffTrig-61, OFF);
      prevCrossingTime = nextCrossingTime;
      lastSniff = millis();
    }
  }
  else if  ((sniffvalue < 0) && (inhal == false)) {
    nextCrossingTime = millis();
    if ((nextCrossingTime - prevCrossingTime) > 20) {
      digitalWrite(SniffTrig, HIGH);
      resp = SetDigital(device, SniffTrig-61, ON);
      prevCrossingTime = nextCrossingTime;
      lastSniff = millis();
    }
  }

  return digitalRead(DIGITAL5);
}

void help() {
  Serial.println(" BCS verification firmware ");
  Serial.println(" Commands:");
  Serial.println("  valve ADDR N <on,  off>: read/set the valve state for valve N on controller ADDR ");
  Serial.println("  vial ADDR N <on,  off>: read/set the vial state for vial N on controller ADDR");
  Serial.println("  vialOn ADDR N: turn On vial N exclusively, turn Off dummy valves on controller ADDR");
  Serial.println("  vialOff ADDR: turn Off all vials, turn On dummy valves on controller ADDR");
  Serial.println("  aMFC ADDR N <value>: read or set the analog value for MFC N on controller ADDR, value in range 0 - 1");
  Serial.println("  sMFC ADDR N <value>: set the value for MFC N on controller ADDR, value in range 0 - 100, serial comm");
  Serial.println("  sol <N> {on,off}, N = {1..8}");
  Serial.println("  beam <N>, N = {1..8}");
  Serial.println("  cue <N> {on,off}, N = {1..8}");
  Serial.println("  digital <N> {high,low,input,output}, N = {1..8}");
  Serial.println("  aout <N> <VALUE>, N = {1..8}, -32768<VALUE<32767 for N = 1 - 4, 0 <VALUE<65535 for N = 5 - 8");
  Serial.println("  ain <N>, N = {1..8}");
  Serial.println("  params <ITI> <odValve> <airVacValve> <stim> <rewd> <watDur> : params 3000 7 15 LH 1 45");
}

void setup()
{
  pinMode(WATER1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);
  pinMode(SOLENOID3, OUTPUT);
  pinMode(SOLENOID4, OUTPUT);
  pinMode(SOLENOID5, OUTPUT);
  pinMode(SOLENOID6, OUTPUT);
  pinMode(SOLENOID7, OUTPUT);
  pinMode(SOLENOID8, OUTPUT);

  digitalWrite(WATER1, LOW);
  digitalWrite(SOLENOID2, LOW);
  digitalWrite(SOLENOID3, LOW);
  digitalWrite(SOLENOID4, LOW);
  digitalWrite(SOLENOID5, LOW);
  digitalWrite(SOLENOID6, LOW);
  digitalWrite(SOLENOID7, LOW);
  digitalWrite(SOLENOID8, LOW);

  pinMode(ADC_PIN, OUTPUT);
  pinMode(DAC1_PIN, OUTPUT);
  pinMode(DAC2_PIN, OUTPUT);

  pinMode(DIGITAL1, OUTPUT);
  pinMode(DIGITAL2, OUTPUT);
  pinMode(DIGITAL3, OUTPUT);
  pinMode(DIGITAL5, OUTPUT);

  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  digitalWrite(ADC_PIN, LOW);
  Wire.begin();

  // initialize SPI
  startSPI();

  // PC communication
  Serial.begin(115200);
  Serial.println(">System is ready");

  // LCD communication
  Serial1.begin(19200);
  Serial1.write(0x0c); // clear the display
  delay(10);
  Serial1.write(0x11); // Back-light on
  Serial1.println("deltaC_v4");

  // setting 6 DIGITAL channels on the valve controller board to OUTPUT mode.
  for (int DO_num = 1; DO_num < 6; DO_num++) {
    if (resp = ModeDigital(1, DO_num, 1)) {
      Serial.print("Error in digital chanel - ");
      Serial.println(DO_num);
    }
  }
}

void loop()
//================ State machine goes here===========================================================
{
  dig_state = SniffCheck(phase);
  if (((dig_state == LOW) && (phase == true)) || ((dig_state == HIGH) && (phase == false))) {
    phase = !phase;
  }
  
  LickCheck();
    
  switch (state) {

    case 0: {
        uint8_t c;
        if (Serial.available() > 0) { // PC communication
          c = Serial.read();
          if (c == '\r') {
            //Serial.flush();
            buffer[idx] = 0;
            Serial.println();
            parse((char*)buffer, argv, sizeof(argv));
            if (strcmp(argv[0], "params") == 0) {

              ITI         = (uint16_t)atoi(argv[1]);
              odorValve   = (uint8_t)atoi(argv[2]);
              airVacValve = (uint8_t)atoi(argv[3]);
              stim        = argv[4];
              rewarded    = (uint8_t)atoi(argv[5]);
              waterDur    = (uint8_t)atoi(argv[6]);
              flow        = (uint8_t)atoi(argv[7]);
              state       = 1;
            }
            else if (strcmp(argv[0], "valve") == 0) {
              ValveOnOff(argv);
            }
            else if (strcmp(argv[0], "twovalves") == 0) {
              twoValves(argv);
            }
            else if (strcmp(argv[0], "vial") == 0) {
              VialOnOff(argv);
            }
            else if (strcmp(argv[0], "vialOn") == 0) {
              VialDummyOn(argv);
            }
            else if (strcmp(argv[0], "vialOff") == 0) {
              VialDummyOff(argv);
            }
            else if (strcmp(argv[0], "aMFC") == 0) {
              FlowChangeAnalog(c, arg1, arg2, argv, resp);
            }
            else if (strcmp(argv[0], "sMFC") == 0) {
              FlowChangeSerial(c, arg1, arg2, argv, resp);
            }
            else if (strcmp(argv[0], "aout") == 0) {
              AnalogOutput(c, arg1, arg2, argv);
            }
            else if (strcmp(argv[0], "ain") == 0) {
              AnalogInput(c, arg1, arg2, argv, resp, ain);
            }
            else if (strcmp(argv[0], "analogSet") == 0) {
              SetAnalogChannel(c, arg1, arg2, argv, resp);
            }
            else if (strcmp(argv[0], "analogRead") == 0) {
              ReadAnalogChannel(c, arg1, arg2, argv, resp);
            }
            else if (strcmp(argv[0], "digitalSet") == 0) {
              SetDigitalChannel(c, arg1, arg2, arg3, argv, resp);
            }
            else if (strcmp(argv[0], "digitalRead") == 0) {
              ReadDigitalChannel(c, arg1, arg2, argv, resp);
            }
            else if (strcmp(argv[0], "sol") == 0) {
              solOnOff(argv);
            }
            else if ((strcmp(argv[0], "help") == 0) || (strcmp(argv[0], "?") == 0)) {
              help();
            }
            idx = 0;
            Serial.print(">");
          }
          else if (((c == '\b') || (c == 0x7f)) && (idx > 0)) {
            idx--;
            Serial.write(c);
            Serial.print(" ");
            Serial.write(c);
          }
          else if ((c >= ' ') && (idx < sizeof(buffer) - 1)) {
            buffer[idx++] = c;
            Serial.write(c);
          }
        }
        break;
      }

  //waiting for initialization of trial
  case 1: {
    startTime = millis();
    digitalWrite(TrialTrig, HIGH); // trial trig
    resp = SetDigital(device, TrialTrig-61, ON);
    resp = analogSetMFC(device, MFC_Air, (float)(1000-flow)/1000);
    resp = analogSetMFC(device, MFC_N2, (float)flow/100);
    state = 2;
    break;
  }

  //wait 2 second before trail starts, and open odor valve (ITI - 2 seconds)
  case 2: {
    currentTime = millis();
    if ((currentTime - startTime) >= (ITI - 2000)) {
      resp = VialOn(device, odorValve); // odor vial on
      digitalWrite(OdorTrig, HIGH); // odor trig
      resp = SetDigital(device, OdorTrig-61, ON);
      state = 3;
    }
    break;
  }

  //wait for ITI and inhalation 
  case  3: {
    currentTime = millis();
    if  (((currentTime - startTime) >= ITI) && (dig_state == HIGH)){
      if (strcmp(stim, "LH")==0 || strcmp(stim, "LL")==0){
        resp = SetVial(device, airVacValve, ON);
        digitalWrite(airVacTrig, HIGH);
        resp = SetDigital(device, airVacTrig-61, ON);
      }
      state = 4;
    }
    if ((currentTime-lastSniff)>2000){
      no_sniff = 1;
      state = 10;
    }
    break;
  }

  // wait for sniff trigger - exhalation onset 
  // correct next case because fv opens not only at the
  // beginning but also during the exhalation phase
  case 4: {
    currentTime = millis();
    if (dig_state == LOW) { //start at the end of inhalation
      sniffcycle = 0;
      resp = SetValve(device, FV, ON);
      digitalWrite(FVTrig, HIGH); // FV trig
      resp = SetDigital(device, FVTrig-61, ON);
      fvOnTime  = millis();
      state = 5;
    }
    if ((currentTime-lastSniff)>2000){
      no_sniff = 1;
      state = 10;
    }
    break;
  }

  //  wait for the end of second inhalation or 500ms to switch odor concentration
  case 5: {
    currentTime = millis();
    //after x sniffcycles, and at the end of inhalation
    if ((sniffcycle == 1) && (dig_state == LOW)) { 
      if (strcmp(stim, "LH")==0){
        resp = SetVial(device, airVacValve, OFF);
        digitalWrite(airVacTrig, LOW); // air and vacuum solenoids trig
        resp = SetDigital(device, airVacTrig-61, OFF);
      }
      else if (strcmp(stim, "HL")==0){
        resp = SetVial(device, airVacValve, ON);
        digitalWrite(airVacTrig, HIGH); // air and vacuum solenoids trig
        resp = SetDigital(device, airVacTrig-61, ON);
      }
      else { 
        resp = SetVial(device, airVacDummy, ON);
      }
      state = 6;
    }
    if ((currentTime-lastSniff)>2000){
      no_sniff = 1;
      state = 10;
//      Serial.println("5_2");
    }
    break;
  }
  
  //  wait for the end of second inhalation or 1000ms to switch odor off
  case 6: {
    currentTime = millis();
    if (((sniffcycle == 2) && (dig_state == LOW)) || ((currentTime-lastSniff)>2000)){ 
      //after x sniffcycles, and at the end of inhalation
      digitalWrite(FVTrig  , LOW); // final valve trig
      resp = SetDigital(device, FVTrig-61, OFF);
      digitalWrite(OdorTrig, LOW); // final valve trig
      resp = SetDigital(device, OdorTrig-61, OFF);
      
      resp = SetValve(device, FV, OFF);
      delay(10);
      resp = SetVial(device, odorValve, OFF);
      resp = SetVial(device, dummy, OFF);
      delay(50);
      resp = SetVial(device, airVacDummy, OFF);
      resp = SetVial(device, airVacValve, OFF);
      digitalWrite(airVacTrig, LOW); // air and vacuum solenoids trig
      resp = SetDigital(device, airVacTrig-61, OFF);
      
      if ((sniffcycle == 2) && (dig_state == LOW)){
        startTime = millis();
        state = 7;
      }
      if ((currentTime-lastSniff)>2000){
        no_sniff = 1;
        state = 10;
      }
    }
    break;
  }


  // wait for lick
  case 7: { 
    currentTime = millis();
    lick_state = digitalRead(LICK1);
    
    if (((currentTime-startTime)>2000)&&(lick_state==LOW)&&(rewarded==1)){
      //Serial.println("miss ");
      lickTime = -1;
      res = 3;
      state = 10;
      //Serial.println("7_1");
    }
    else if (((currentTime-startTime)>2000)&&(lick_state==LOW)&&(rewarded==0)){
      //Serial.println("correct rejection ");
      lickTime = -1;
      res = 2;
      state = 10;
    }
    else if (((currentTime-startTime)<2000)&&(lick_state==HIGH)&&(rewarded==1)){
      //Serial.println("hit ");
      waterOnTime = millis(); // waterTimer
      digitalWrite(WATER1, HIGH); //Water On
      res = 1;
      lickTime = currentTime-startTime;
      state = 8;
    }
    else if (((currentTime-startTime)<2000)&&(lick_state==HIGH)&&(rewarded==0)){
      //Serial.println("false alarm ");
      res = 4;
      lickTime = currentTime-startTime;
      startTime = millis();
      state = 9;
    }
    break;
  }
    
  // close water valve
  case 8: {
    waterOffTime = millis();
    if ((waterOffTime - waterOnTime) >= waterDur) {
      digitalWrite(WATER1, LOW); //Water Off
      state = 10;
    }
    break;
  }

  // ITI increase 
  case  9: {
    currentTime = millis();
    if  ((currentTime - startTime) >= ITI){
      state = 10;
    }
    break;
  }
    
  case 10: { 
    //Serial.println("10");
    digitalWrite(TrialTrig, LOW); // trial trig
    resp = SetDigital(device, TrialTrig-61, OFF);
    digitalWrite(OdorTrig, LOW);// fv trigger
    resp = SetDigital(device, OdorTrig-61, OFF);
    if (no_sniff == 1){
      Serial.println("No sniff");
      no_sniff = 0;
    } 
    else{
      Serial.print("Output ");
      Serial.print(res);
      Serial.print(" ");
      Serial.println(lickTime);
    }
    res = 0;
    state = 0;
    break;
  } // end of case 10
} // end of switch
} // end of loop


