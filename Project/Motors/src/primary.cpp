
#include <WiFi.h>
#include <sys/socket.h>

#include <Arduino.h>

#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

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

const unsigned int PWM_VALUE = 1023; // Max PWM given 8 bit resolution

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10;

int adc1_buf[8];
int adc2_buf[8];

float mid = 6;
int base_pid = 450;


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
  int mode = 0;


uint8_t lineArray[13]; 
float previousPosition = 6;


float error;
float last_error;
float total_error;

float Kp = 25;
float Kd = 500;
float Ki = 0.01;

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


void readADC() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
}


/*
digitalConvert() reads the values of the sensors from adc1_buf and adc2_buf
and stores them as a 1 or 0 into lineArray[0 -> 13]

*/

void digitalConvert(){
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



// Command to turn the robot left upon detecting a mode change block
// Stops the left motor and guns the right motor to turn left
void MODE_CHANGE_BLOCK_LEFT(){

  M1_stop();
  M2_forward(PWM_VALUE);

}


// Command to turn the robot right upon detecting a mode change block
// Stops the right motor and guns the left motor to turn right
void MODE_CHANGE_BLOCK_RIGHT(){


  M2_stop();
  M1_forward(PWM_VALUE);

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


void RUN_PID(float previousPosition, float mid){

  digitalConvert();

  float pos = getPosition(previousPosition);
  previousPosition = pos;

   error = pos - mid;
   total_error += error;

   int pid_value = Kp*error + Kd*(error-last_error) + Ki*total_error;
   int right_motor = base_pid + pid_value;
   int left_motor = base_pid - pid_value;

   M1_forward(left_motor);
   M2_forward(right_motor);

}


// WIFI STUFF

// Change this stuff later

const char* ssid = "Elink"; // Name of Network
const char* password = "David!Kristina"; // Network Password

/*
CHECK_MODE_CHANGE checks all of the sensors are white- which signifies the NEXT mode, creep forward and check
*/
int CHECK_MODE_CHANGE(){

  if (lineArray[0] & lineArray[1] & lineArray[2] & lineArray[3] & lineArray[4] & lineArray[5] & lineArray[6] & lineArray[7] & lineArray[8]
   & lineArray[9] & lineArray[10] & lineArray[11] & lineArray[12]){


    /*
    
    wait()
    Creep forward 

    M1_stop();
    M2_stop();
    */

    return 1;

   }
   else{

    return 0;

   }

}


/*
THIS MODE JUST FOLLOWS A STRAIGHT LINE WITH 90 degree turns

*/
void MOVE_LINEAR(){

  digitalConvert();

  /*
  MOVE AT A SLOWER PACE TO DEAL WITH 90 DEGREE TURNS
  */
  M1_forward(PWM_VALUE/2);
  
  M2_forward(PWM_VALUE/2);

  /*
  MOVE FORWARD FOR 1 SECOND
  */
  sleep(1000);



  /*
  GO STRAIGHT
  */
 if (!lineArray[0] & !lineArray[1] & !lineArray[2] & !lineArray[3] & !lineArray[4] & lineArray[5] & lineArray[6] & !lineArray[7] & !lineArray[8]
   & !lineArray[9] & !lineArray[10] & !lineArray[11] & !lineArray[12]){
  M1_forward(PWM_VALUE/2);
  
  M2_forward(PWM_VALUE/2);

  sleep(1000);

}
/*
TURN LEFT (WHITE ON LEFT SIDE, BLACK ON RIGHT SIDE)
*/
else if (lineArray[0] & lineArray[1] & lineArray[2] & lineArray[3] & lineArray[4] & lineArray[5] & lineArray[6] & !lineArray[7] & !lineArray[8]
   & !lineArray[9] & !lineArray[10] & !lineArray[11] & !lineArray[12]){

    //
    M1_backward(PWM_VALUE/2);
    M2_forward(PWM_VALUE/2);

    sleep(1000);


   }


/*
TURN RIGHT 90 DEGREES (BLACK ON LEFT SIDE, WHITE ON RIGHT SIDE)
*/
else if (!lineArray[0] & !lineArray[1] & !lineArray[2] & !lineArray[3] & !lineArray[4] & !lineArray[5] & !lineArray[6] & !lineArray[7] & lineArray[8]
   & lineArray[9] & lineArray[10] & lineArray[11] & lineArray[12]){

  sleep(1000);

   }


}


void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason



  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200);

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);



    // WIFI STUFF

    delay(1000);
    // Connecting to Mutual WiFi Network
    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
    // SERVER SOCKET SECTION *************************
    // Used when Nvidia Sends Messages -> Heltec
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0); 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(8000); 
    String ipString = WiFi.localIP().toString();
    serverAddress.sin_addr.s_addr = inet_addr(ipString.c_str()); 
    // binding socket. 
    bind(serverSocket, (struct sockaddr*)&serverAddress, 
         sizeof(serverAddress)); 
    listen(serverSocket, 5); 
    // accepting connection request 
    int clientSocket = accept(serverSocket, nullptr, nullptr); 
  
    // recieving data 
    char buffer[1024] = { 0 }; 
    recv(clientSocket, buffer, sizeof(buffer), 0); 
    Serial.println(buffer);

    // closing the socket. 
    close(serverSocket);

}

/*
  Mode 0: Line of the republic (Forward movement)

  Mode 1: Maze of mandalore (Searching algorithm w/ GPU)

  Mode 2: Kessel run (PID movement)

  Mode 3: Hoth Asteroid Field (Avoidance of obstacles mode w/ GPU)

  Mode 4: Path of dual fates (Microphone and audio localization w/ GPU)

  Mode 5: Endor Dash (Forward movement)
  */

void loop() {

  int t_start = micros();
  readADC();
  int t_end = micros();


  //Put switch statements here for modes
switch(mode) {

  // START - GUN IT FORWARD
  case 0:
  

    /*
    RUN THE PID CONTROLLER UNTIL ALL SENSORS TURN WHITE
    */
    while (~CHECK_MODE_CHANGE()){

      RUN_PID(previousPosition, mid);

    }

    /*
    TURN RIGHT AND MOVE
    */
    MODE_CHANGE_BLOCK_RIGHT;

    // ALL MOTORS FORWARD
    // PID CONTROLLER

    break;

  // MAZE OF MANDALORE - MAZE MODE (PRIMARY GUNS FORWARD)
  case 1:



    // THE PRIMARY ROBOT SKIPS ALL OF THIS 


    break;


  // KESSEL RUN - PID CONTROLLER BROKEN PATH
  case 2:

    // TUNE PID CONTROLLER HERE

    break;

  //HOTH ASTEROID FIELD - AVOIDANCE MODE
  case 3:

    break;


  // DUAL FATES - SOUND LOCALIZATION (PYTHON HERE)
  case 4:

    //call python

    break;
  
  // GUN IT
  case 5:

    // ALL MOTORS FORWARD
    // PID CONTROLLER


    break;
      

  
  // ENDOR DASH - GUN IT FORWARD UNTIL HIT MODE CHANGE
  case 6:

    // ALL MOTORS FORWARD
    // MAKE SURE TO LINE UP ROBOT ON THE LINE


    break;

  // END ARENA- STOP
  case 7:

    // STOP ALL MOTORS

    break;
}

// Serial.print("left: \t"); Serial.print(left_motor); Serial.print("\t");

// Serial.print("right: \t"); Serial.print(right_motor); Serial.print("\t");

  // Serial.print("time: \t"); Serial.print(t_end - t_start); Serial.print("\t");
  // Serial.print("pos: \t"); Serial.print(pos);
  // Serial.println();

  // M1_forward(512);
  // M2_forward(512);
  last_error = error;

  // delay(100);

}