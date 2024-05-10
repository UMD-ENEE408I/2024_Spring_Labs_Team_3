
#include <WiFi.h>
#include <sys/socket.h>

#include <Arduino.h>

#include <Encoder.h>

#include <Adafruit_MCP3008.h>

#include <Adafruit_MPU6050.h>


/*
ONLY RUN AT 8.27 VOLTS OR HIGHER


*/


/*
CREATE OBJECTS
*/
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

Adafruit_MPU6050 mpu;

/*
ENCODER PINS
*/
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

/*
MOTOR PINS
*/
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

// const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 255; // Max PWM given 8 bit resolution

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

int adc1_buf[8];
int adc2_buf[8];


uint8_t lineArray[13]; 

float mid = 6;


/*
STOP if TRUE
*/
bool running = true;


  // The mode is incremented each time the robot hits a fat block -PK

  // We start in mode 0
  /*
  Mode 0: Line of the republic (Forward movement)

  Mode 1: Maze of mandalore (Searching algorithm w/ GPU)

  Mode 2: Kessel run (PID movement)

  Mode 3: Hoth Asteroid Field (Avoidance of obstacles mode w/ GPU)

  Mode 4: Path of dual fates (Microphone and audio localization w/ GPU)

  Mode 5: Endor Dash (Forward movement)
  
  
  
  
  */

float previousPosition = 6;


float error;
float last_error;
float total_error;

//int base_pid = 90;

/* float Kp = 5;
float Kd = 80;
float Ki = 0; */

void M1_backward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, pwm_value);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M1_IN_2_CHANNEL, PWM_VALUE);
}

void M2_backward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, pwm_value);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M2_IN_2_CHANNEL, PWM_VALUE);
}

void GYRO(float *curr_angle, float *g_prev, unsigned long *t_prev) {
  sensors_event_t a, g, temp; // A struct that holds mpu reading
  float g_0 = 0;              // Current gyroscope reading
  unsigned long t_delta;      // Change in milis() time (ms)
  
  // Latest MPU update
  mpu.getEvent(&a, &g, &temp);
  g_0 = g.gyro.z ;
  
  // Update time elapsed
  t_delta = millis() - *t_prev;
  
  // Run the integral and increment angle
  *curr_angle += 1.2*(180/3.1416)*(t_delta*0.001)*(g_0 + *g_prev)/2;
  
  // Update "previous" values
  *g_prev = g_0;
  *t_prev = t_delta + *t_prev;
}


void readADCandConvert() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);

    
  //Serial.printf("%d %d\n", adc1_buf[i], adc2_buf[i]);
  }

for (int i = 0; i < 7; i++) {
    if (adc1_buf[i]<700) {
      lineArray[2*i] = 1; 
    } else {
      lineArray[2*i] = 0;
    }
    // Serial.print(lineArray[2*i]); Serial.print("\t");
    // Serial.print(adc1_buf[i]); Serial.print("\t");

    if (i<6) {
      if (adc2_buf[i]<700){
        lineArray[2*i+1] = 1;
      } else {
        lineArray[2*i+1] = 0;
      }
      // Serial.print(lineArray[2*i+1]); Serial.print("\t");
      // Serial.print(adc2_buf[i]); Serial.print("\t");
    }
  }

}


/*
digitalConvert() reads the values of the sensors from adc1_buf and adc2_buf
and stores them as a 1 or 0 into lineArray[0 -> 13]

*/

void digitalConvert(){
  
}

void NINETY_DEGREE_TURN(float degrees) {

  M1_stop();
  M2_stop();

  delay(200);

  float curr_angle = 0;            
  float g_prev = 0;                
  unsigned long t_prev = millis();  

  // ccw vs. cw
  if (degrees > 0) {
    M1_backward(90);
    M2_forward(90);
  } else {
    M1_forward(90);
    M2_backward(90);
  }

  // Poll until goal is reached
  while (abs(curr_angle) < abs(degrees)){
    
    GYRO(&curr_angle, &g_prev, &t_prev);

  } 
  
  M1_stop();
  M2_stop();

  delay(200);
}
/*
CHECK_MODE_CHANGE checks all of the sensors are white- which signifies the NEXT mode, creep forward and check
*/
int CHECK_MODE_CHANGE(){


  readADCandConvert();

  if (lineArray[0] && lineArray[1] && lineArray[2] && lineArray[3] && lineArray[4] && lineArray[5] && lineArray[6] && lineArray[7] && lineArray[8]
  && lineArray[9] && lineArray[10] && lineArray[11] && lineArray[12]){

    return 1;

  }
  else{

    return 0;

  }

}


void MOVE_FORWARD(int distance, int base_pid, Encoder & enc1, Encoder & enc2){

  enc1.readAndReset();

  enc2.readAndReset();

  long enc1_value = -1;

  long enc2_value = abs(enc2.read());

  long error = 0;

  long last_error = 0;

  long total_error = 0;

  float Kp_s = 10;

  float Kd_s = 5;

  float Ki_s = 0.005;

  M1_forward(90);
  M2_forward(90);
  // delay(100);

  while((enc1_value < distance)){


    enc1_value = abs(enc1.read());

    enc2_value = abs(enc2.read());

    error = enc1_value - enc2_value;

    //Serial.printf("%d\n", error);

    int pid_value = Kp_s*error + Kd_s*(error-last_error) + Ki_s*total_error;

    last_error = error;

    total_error += error;

    int left_motor = base_pid - pid_value;

    int right_motor = base_pid + pid_value;

    M1_forward(left_motor);

    M2_forward(right_motor);



    if (distance == 0){

      enc1_value = -1;

    }


}


    M1_stop();
    M2_stop();
}


/*
MOVE FORWARD IN EMPTY SPACE UNTIL YOU GET TO A SOLID WHITE LINE
*/
void DEAD_RECKON_MOVE_FORWARD(int base_pid, Encoder & enc1, Encoder & enc2){

  enc1.readAndReset();

  enc2.readAndReset();

  long enc1_value = -1;

  long enc2_value = abs(enc2.read());

  long error = 0;

  long last_error = 0;

  long total_error = 0;

  float Kp_s = 10;

  float Kd_s = 0;

  float Ki_s = 0;

  M1_forward(90);
  M2_forward(90);
  // delay(100);



    enc1_value = abs(enc1.read());

    enc2_value = abs(enc2.read());

    error = enc1_value - enc2_value;

    //Serial.printf("%d\n", error);

    int pid_value = Kp_s*error + Kd_s*(error-last_error) + Ki_s*total_error;

    last_error = error;

    total_error += error;

    int left_motor = base_pid - pid_value;

    int right_motor = base_pid + pid_value;

    M1_forward(left_motor);

    M2_forward(right_motor);

}


void TILL_LINE_MOVE_FORWARD(int base_pid, Encoder & enc1, Encoder & enc2){

  enc1.readAndReset();

  enc2.readAndReset();

  long enc1_value = -1;

  long enc2_value = abs(enc2.read());

  long error = 0;

  long last_error = 0;

  long total_error = 0;

  float Kp_s = 10;

  float Kd_s = 0;

  float Ki_s = 0;

  // delay(100);

  M1_forward(90);
  M2_forward(90);
  delay(100);

  readADCandConvert();

do {

    enc1_value = abs(enc1.read());

    enc2_value = abs(enc2.read());

    error = enc1_value - enc2_value;

    int pid_value = Kp_s*error + Kd_s*(error-last_error) + Ki_s*total_error;

    last_error = error;

    total_error += error;

    int left_motor = base_pid - pid_value;

    int right_motor = base_pid + pid_value;

    M1_forward(left_motor);

    M2_forward(right_motor);

    if(error == 0){


      enc1_value = -1;
    }



}while(!CHECK_MODE_CHANGE());

M1_forward(base_pid);
M2_forward(base_pid);
delay(100);

M1_stop();
M2_stop();
delay(100);

}


// Command to turn the robot left upon detecting a mode change block
// Stops the left motor and guns the right motor to turn left
void MODE_CHANGE_BLOCK_LEFT(Encoder & enc1, Encoder & enc2){


    /*
    TUNED TO SALEH'S ROBOT
    */
  M1_stop();
   M2_stop();

   delay(100);

   MOVE_FORWARD(600,90,enc1, enc2);

    M1_stop();
    M2_stop();

    delay(100);
    


    NINETY_DEGREE_TURN(90);

    M1_stop();
    M2_stop();

    delay(100);


   MOVE_FORWARD(500,80,enc1, enc2);

}


// Command to turn the robot right upon detecting a mode change block
// Stops the right motor and guns the left motor to turn right
void MODE_CHANGE_BLOCK_RIGHT(Encoder & enc1, Encoder & enc2){

    /*
    TUNED TO SALEH'S ROBOT
    */
  M1_stop();
   M2_stop();

   delay(100);

   MOVE_FORWARD(500,80,enc1, enc2);

    M1_stop();
    M2_stop();

    delay(100);
    


    NINETY_DEGREE_TURN(-90);

    M1_stop();
    M2_stop();

    delay(100);


   MOVE_FORWARD(500,80,enc1, enc2);

}



float getPosition(float previousPosition) {
  
  float pos = 0;
  uint8_t white_count = 0;
  for (int i = 0; i < 13; i++) {
    if (lineArray[i] == 1) {
      pos += i;
      white_count+=1;
    } 
  }


  // Serial.print("white: "); Serial.print(white_count); Serial.print("\t");
  // Serial.print("pos: "); Serial.print(pos); Serial.println("\t");
  if (white_count == 0) {
    return previousPosition;
  }
  return pos/white_count;
}



void MODE_1(int base_pid, int Kp, int Kd, int Ki){


  /*
  RUN PID CONTROLLER UNTIL MODE CHANGE BLOCK IS HIT
  */

  while(!CHECK_MODE_CHANGE()){

    /*
    RUN PID CONTROLLER
    */
   
 // Serial.print("MODE_1\n");

    int t_start = micros();
    int t_end = micros();

/*     long enc1_value = enc1.read();
    long enc2_value = -1*enc2.read();

    enc1.write(0);
    enc2.write(0);
 */
    //enc1.readAndReset();

    //enc2.readAndReset();
    float pos = getPosition(previousPosition);
    previousPosition = pos;

    error = pos - mid;
    total_error += error;

    int pid_value = Kp*error + Kd*(error-last_error) + Ki*total_error;
    int right_motor = base_pid + pid_value;
    int left_motor = base_pid - pid_value;

    M1_forward(left_motor);
    M2_forward(right_motor);

    /* Serial.print("time: \t"); Serial.print(t_end - t_start); Serial.print("\t");
    Serial.print("pos: \t"); Serial.print(pos);Serial.print("right: \t"); Serial.print(right_motor); 
    Serial.println(); */

    last_error = error;


  }
}

void START_BLOCK(){


/*
START IN CENTER OF WHITE BLOCK
ACTIVATE PID MOVEMENT WHEN NOT ALL SENSORS READ WHITE
*/

M2_forward(0.4*PWM_VALUE);
M1_forward(0.4*PWM_VALUE);



while(CHECK_MODE_CHANGE()){

}

  /*
  
  MODE 1: FORWARD LINE
  
  */

  MODE_1(90, 5,250,0);


}



/*

PID MOVEMENT FOR A CERTAIN DISTANCE IN INCHES
*/
void MODE_2(int dist, int base_pid, int Kp, int Kd, int Ki){


  /*DIST IS IN INCHES
  -> CONVERTED TO TICKS
  
  12 INCHES -> 1000 TICKS
  */



 int tick_goal = dist*1000/12;
  /*
  RUN PID CONTROLLER UNTIL MODE CHANGE BLOCK IS HIT
  */

 int tick = 0;

for(tick; tick < tick_goal; tick++){

  readADCandConvert();

    /*
    RUN PID CONTROLLER
    */
   
 // Serial.print("MODE_1\n");

    int t_start = micros();
    int t_end = micros();

/*     long enc1_value = enc1.read();
    long enc2_value = -1*enc2.read();

    enc1.write(0);
    enc2.write(0);
 */
    //enc1.readAndReset();

    //enc2.readAndReset();
    float pos = getPosition(previousPosition);
    previousPosition = pos;

    error = pos - mid;
    total_error += error;

    int pid_value = Kp*error + Kd*(error-last_error) + Ki*total_error;
    int right_motor = base_pid + pid_value;
    int left_motor = base_pid - pid_value;

    M1_forward(left_motor);
    M2_forward(right_motor);

    /* Serial.print("time: \t"); Serial.print(t_end - t_start); Serial.print("\t");
    Serial.print("pos: \t"); Serial.print(pos);Serial.print("right: \t"); Serial.print(right_motor); 
    Serial.println(); */

    last_error = error;


  }
}

  void SKIP_MAZE(Encoder & enc1, Encoder & enc2){



  //MODE_2(5,80,6,100,0);


  delay(300);
  NINETY_DEGREE_TURN(-90);

  delay(300);
  
  //MODE_CHANGE_BLOCK_RIGHT(enc1, enc2);


  MOVE_FORWARD(4800,255,enc1,enc2);

  delay(300);

  // M1_stop();
  // M2_stop();

  NINETY_DEGREE_TURN(90);

  TILL_LINE_MOVE_FORWARD(90,enc1,enc2);

  delay(200);

  NINETY_DEGREE_TURN(-90);



  MODE_1(90,5,130,0);


    /*
    SKIP MAZE
    */
    //base_pid, Kp, Kd, Ki
    //MODE_1(80, 5,250,0);

  
  }

  void KESSEL(){



    M1_stop();
    M2_stop();
    delay(100);



    /*
    KESSEL RUN 
    */

   
    readADCandConvert();

    //base_pid, Kp, Kd, Ki
    MODE_1(70, 8,120,0);

  }


  void ASTEROIDS( Encoder & enc1, Encoder & enc2){


    M1_stop();
    M2_stop();
    delay(500);
  MOVE_FORWARD(2400,90,enc1,enc2);


  //MODE_2(10,75,6,60,0);

  M1_stop();
  M2_stop();
  
  delay(500);

  NINETY_DEGREE_TURN(85);

  delay(200);



  //MOVE_FORWARD(1200,100,enc1,enc2);


 // NINETY_DEGREE_TURN(90);



/*
GUN FORWARD
*/
 MOVE_FORWARD(5200,150,enc1,enc2);

  
  delay(200);

  NINETY_DEGREE_TURN(90);

  
  TILL_LINE_MOVE_FORWARD(90,enc1, enc2);

  
  NINETY_DEGREE_TURN(-90);


  MODE_1(85,6,60,0);


  }



void DUAL_FATES(const char* ssid, 
const char* password, WiFiServer server, Encoder & enc1, Encoder & enc2){

  
  MODE_1(70,8,130,0);


  int Amp_left, Amp_right;


  /*
  GET AMPLITUDE LEFT AND RIGHT
  */

 NINETY_DEGREE_TURN(90);
 
delay(100);



/*
GET AMPLITUDE FROM LEFT SIDE
*/


WiFiClient client = server.available(); // Check for a client connection

    if (client) {
        Serial.println("Connected to client");
        while (client.connected()) { // Loop while the client is connected
            if (client.available()) { // If data is available from the client



                /*
                GET AMPLITUDE LEFT
                */

                int Amp_left = client.parseInt(); // Read the incoming integer
                Serial.println("Received data: " + String(Amp_left));

              
            }
        } 
        client.stop(); // Close the connection
        Serial.println("Client Disconnected.");
    }

 delay(10000);
 
 //float Amp_left = 0;

 
 NINETY_DEGREE_TURN(-180);

 
 delay(100);

 /*
GET AMPLITUDE FROM LEFT SIDE
*/

/* 
WiFiClient client = server.available(); // Check for a client connection
 */
    if (client) {
        Serial.println("Connected to client");
        while (client.connected()) { // Loop while the client is connected
            if (client.available()) { // If data is available from the client



                /*
                GET AMPLITUDE RIGHT
                */

                int Amp_right = client.parseInt(); // Read the incoming integer
                Serial.println("Received data: " + String(Amp_right));

              
            }
        } 
        client.stop(); // Close the connection
        Serial.println("Client Disconnected.");
    }

 delay(10000);

 
 /*
 GET AMPLITUDE RIGHT
 */
 //float Amp_right = 0;

bool path = (Amp_right - Amp_left) > 0;


//bool path = 1;

 /*
 PATH > 0: LEFT
 PATH < 0: RIGHT
 DEFAULT: SKIP (GO STRAIGHT)
 */

switch (path){



  /*
  CASE 0: TURN LEFT
  */
  case(0):

  NINETY_DEGREE_TURN(180);

  delay(100);


  do{

    /*
    RUN PID CONTROLLER
    */
   
 // Serial.print("MODE_1\n");

    int t_start = micros();
    int t_end = micros();

/*     long enc1_value = enc1.read();
    long enc2_value = -1*enc2.read();

    enc1.write(0);
    enc2.write(0);
 */
    //enc1.readAndReset();

    //enc2.readAndReset();
    float pos = getPosition(previousPosition);
    previousPosition = pos;

    error = pos - mid;
    total_error += error;

    int pid_value = 8*error + 120*(error-last_error) + 0*total_error;
    int right_motor = 70 + pid_value;
    int left_motor = 70 - pid_value;

    M1_forward(left_motor);
    M2_forward(right_motor);

    /* Serial.print("time: \t"); Serial.print(t_end - t_start); Serial.print("\t");
    Serial.print("pos: \t"); Serial.print(pos);Serial.print("right: \t"); Serial.print(right_motor); 
    Serial.println(); */

    last_error = error;


  readADCandConvert();
  }while(!(lineArray[12] && lineArray[11] && lineArray[10] && lineArray[9]));

  M1_forward(90);
  M2_forward(90);
  delay(200);

  NINETY_DEGREE_TURN(90);

  MODE_1(90,6,90,0);



  break;

  

  /*
  CASE 1: TURN RIGHT
  */
  case(1):

  MOVE_FORWARD(4,90,enc1,enc2);

  delay(100);


  do{

    /*
    RUN PID CONTROLLER
    */
   
 // Serial.print("MODE_1\n");

    int t_start = micros();
    int t_end = micros();

/*     long enc1_value = enc1.read();
    long enc2_value = -1*enc2.read();

    enc1.write(0);
    enc2.write(0);
 */
    //enc1.readAndReset();

    //enc2.readAndReset();
    float pos = getPosition(previousPosition);
    previousPosition = pos;

    error = pos - mid;
    total_error += error;

    int pid_value = 8*error + 120*(error-last_error) + 0*total_error;
    int right_motor = 70 + pid_value;
    int left_motor = 70 - pid_value;

    M1_forward(left_motor);
    M2_forward(right_motor);

    /* Serial.print("time: \t"); Serial.print(t_end - t_start); Serial.print("\t");
    Serial.print("pos: \t"); Serial.print(pos);Serial.print("right: \t"); Serial.print(right_motor); 
    Serial.println(); */

    last_error = error;


  readADCandConvert();
  }while(!(lineArray[0] && lineArray[1] && lineArray[2] && lineArray[3]));

  M1_forward(90);
  M2_forward(90);
  delay(200);

  NINETY_DEGREE_TURN(-90);

  MODE_1(90,6,90,0);



  break;

  

  /*
  SKIP IT
  */
  default:

  
  readADCandConvert();

  while(!CHECK_MODE_CHANGE()){

    DEAD_RECKON_MOVE_FORWARD(120,enc1,enc2);

  }


  break;

 }


}


void ENDOR_DASH(Encoder & enc1, Encoder & enc2){

  /*
  GUN FORWARD
  */
  //TILL_LINE_MOVE_FORWARD(255,enc1,enc2);

  MOVE_FORWARD(5600,150,enc1,enc2);


    M1_forward(90);
    M2_forward(90);
    delay(200);
    M1_stop();
    M2_stop();
    delay(200);

  M1_backward(100);
  M2_forward(100);

    delay(100000);






}



// void MAZE_UTURN_LEFT(Encoder & enc1, Encoder & enc2){

//   NINETY_DEGREE_TURN(90);

//   delay(100);
//   MOVE_FORWARD(5,85,enc1,enc2);


//   delay(100);
//   NINETY_DEGREE_TURN(-90);

  
//   delay(100);

//   MOVE_FORWARD(12,85,enc1,enc2);

//   delay(100);


//   NINETY_DEGREE_TURN(-90);

//   delay(100);

//   TILL_LINE_MOVE_FORWARD(85,enc1,enc2);

  
//   MOVE_FORWARD(100,85,enc1,enc2);

//   delay(100);

//   NINETY_DEGREE_TURN(90);

//   MODE_2(4,85,4,80,0);

// }

// void MAZE_UTURN_RIGHT(Encoder & enc1, Encoder & enc2){


//   NINETY_DEGREE_TURN(-90);

//   delay(100);
//   MOVE_FORWARD(5,85,enc1,enc2);


//   delay(100);
//   NINETY_DEGREE_TURN(90);

  
//   delay(100);

//   MOVE_FORWARD(12,85,enc1,enc2);

//   delay(100);


//   NINETY_DEGREE_TURN(90);

//   delay(100);

//   TILL_LINE_MOVE_FORWARD(85,enc1,enc2);

//   MOVE_FORWARD(100,85,enc1,enc2);

//   delay(100);

//   NINETY_DEGREE_TURN(-90);

//   MODE_2(4,85,4,80,0);


// }

void MAZE_AVOID_LEFT(Encoder & enc1, Encoder & enc2){

  M1_stop();
  M2_stop();

  delay(200);

NINETY_DEGREE_TURN(90);

delay(100);

MOVE_FORWARD(350,88,enc1,enc2);


delay(500);

NINETY_DEGREE_TURN(-90);

delay(500);

MOVE_FORWARD(650,88,enc1,enc2);

delay(500);

NINETY_DEGREE_TURN(-90);

delay(500);

TILL_LINE_MOVE_FORWARD(88,enc1,enc2);

MOVE_FORWARD(100,88,enc1,enc2);

delay(500);

NINETY_DEGREE_TURN(90);

//MODE_2(4,85,4,80,0);



}

void MAZE_AVOID_RIGHT(Encoder & enc1, Encoder & enc2){


NINETY_DEGREE_TURN(-90);

delay(100);

MOVE_FORWARD(500,85,enc1,enc2);


delay(200);

NINETY_DEGREE_TURN(90);

delay(200);

MOVE_FORWARD(650,85,enc1,enc2);

delay(200);

NINETY_DEGREE_TURN(90);

delay(200);


TILL_LINE_MOVE_FORWARD(85,enc1,enc2);

MOVE_FORWARD(100,85,enc1,enc2);

delay(200);

NINETY_DEGREE_TURN(-90);

//MODE_2(4,85,4,80,0);

  
}


void MAZE_FORWARD(Encoder & enc1, Encoder & enc2){

  MODE_2(13,85,2,400,0);

  M1_stop();
  M2_stop();

  delay(100);

}

void PRIMARY(const char* ssid, const char* password, WiFiServer server, Encoder & enc1, Encoder & enc2){

  START_BLOCK();

  MODE_CHANGE_BLOCK_RIGHT(enc1, enc2);

  MOVE_FORWARD(2,90,enc1,enc2);

  SKIP_MAZE(enc1, enc2);

  MODE_CHANGE_BLOCK_LEFT(enc1, enc2);

  KESSEL();

  ASTEROIDS( enc1 , enc2);

  MODE_CHANGE_BLOCK_RIGHT(enc1,enc2);


  DUAL_FATES(ssid, password, server,enc1,enc2);

  MODE_CHANGE_BLOCK_LEFT(enc1, enc2);

  MODE_1(90,6,120,0);

  
  MOVE_FORWARD(600,90,enc1, enc2);


  MODE_CHANGE_BLOCK_LEFT(enc1, enc2);

  ENDOR_DASH(enc1, enc2);

}



void DO_MAZE(Encoder & enc1, Encoder & enc2){



/*
FROM PYTHON:




*/

  START_BLOCK();

  
  MODE_CHANGE_BLOCK_RIGHT(enc1, enc2);


  MODE_2(14,90,4,300,0);


    M1_stop();
  M2_stop();
  delay(1000);


  MAZE_FORWARD(enc1,enc2);

  delay(1000);

  MAZE_FORWARD(enc1,enc2);
  

  delay(1000);

  MAZE_FORWARD(enc1,enc2);

  delay(1000);

  MAZE_FORWARD(enc1,enc2);

  delay(100000);


  MAZE_AVOID_LEFT(enc1, enc2);

  MAZE_AVOID_RIGHT(enc1,enc2);


  delay(1000);
  MAZE_AVOID_RIGHT(enc1,enc2);

  delay(1000);
  MAZE_FORWARD(enc1, enc2);



}