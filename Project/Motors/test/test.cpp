
#include <WiFi.h>
#include <sys/socket.h>

#include <Arduino.h>

#include <Encoder.h>

#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

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

const unsigned int PWM_VALUE = 1023; // Max PWM given 8 bit resolution

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10;

int adc1_buf[8];
int adc2_buf[8];

float mid = 6;


/*
STOP if TRUE
*/
bool status = false;


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

uint8_t lineArray[13]; 
float previousPosition = 6;


float error;
float last_error;
float total_error;

int base_pid = 400;

float Kp = 10;
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
  M2_forward(PWM_VALUE/4);

  delay(500);



}


// Command to turn the robot right upon detecting a mode change block
// Stops the right motor and guns the left motor to turn right
void MODE_CHANGE_BLOCK_RIGHT(){


  /*
  ARC RIGHT (NEEDS TO BE FINE TUNED)
  */
  M2_stop();
  M1_forward(PWM_VALUE/4);

  delay(100);



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



// WIFI STUFF

// Change this stuff later

// const char* ssid = "Elink"; // Name of Network
// const char* password = "David!Kristina"; // Network Password

/*
CHECK_MODE_CHANGE checks all of the sensors are white- which signifies the NEXT mode, creep forward and check
*/
int CHECK_MODE_CHANGE(){

  if (lineArray[0] & lineArray[1] & lineArray[2] & lineArray[3] & lineArray[4] & lineArray[5] & lineArray[6] & lineArray[7] & lineArray[8]
   & lineArray[9] & lineArray[10] & lineArray[11] & lineArray[12]){


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
  M1_forward(PWM_VALUE/4);
  
  M2_forward(PWM_VALUE/4);

  sleep(1000);

}
/*
TURN LEFT (WHITE ON LEFT SIDE, BLACK ON RIGHT SIDE)
*/
else if (lineArray[0] & lineArray[1] & lineArray[2] & lineArray[3] & lineArray[4] & lineArray[5] & lineArray[6] & !lineArray[7] & !lineArray[8]
   & !lineArray[9] & !lineArray[10] & !lineArray[11] & !lineArray[12]){


    /*
    ZERO POINT TURN
    */
    M1_backward(PWM_VALUE/2);
    M2_forward(PWM_VALUE/2);

    sleep(1000);

   }


/*
TURN RIGHT 90 DEGREES (BLACK ON LEFT SIDE, WHITE ON RIGHT SIDE)
*/
else if (!lineArray[0] & !lineArray[1] & !lineArray[2] & !lineArray[3] & !lineArray[4] & !lineArray[5] & !lineArray[6] & !lineArray[7] & lineArray[8]
   & lineArray[9] & lineArray[10] & lineArray[11] & lineArray[12]){


    /*
    ZERO POINT TURN
    */
    M1_backward(PWM_VALUE/2);
    M2_forward(PWM_VALUE/2);

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

  delay(2000);



    // WIFI STUFF

    // delay(1000);
    // // Connecting to Mutual WiFi Network
    // WiFi.mode(WIFI_STA); //Optional
    // WiFi.begin(ssid, password);
    // Serial.println("\nConnecting");

    // while(WiFi.status() != WL_CONNECTED){
    //     Serial.print(".");
    //     delay(100);
    // }
    // Serial.println("\nConnected to the WiFi network");
    // Serial.print("Local ESP32 IP: ");
    // Serial.println(WiFi.localIP());
    // // SERVER SOCKET SECTION *************************
    // // Used when Nvidia Sends Messages -> Heltec
    // int serverSocket = socket(AF_INET, SOCK_STREAM, 0); 
    // sockaddr_in serverAddress; 
    // serverAddress.sin_family = AF_INET; 
    // serverAddress.sin_port = htons(8000); 
    // String ipString = WiFi.localIP().toString();
    // serverAddress.sin_addr.s_addr = inet_addr(ipString.c_str()); 
    // // binding socket. 
    // bind(serverSocket, (struct sockaddr*)&serverAddress, 
    //      sizeof(serverAddress)); 
    // listen(serverSocket, 5); 
    // // accepting connection request 
    // int clientSocket = accept(serverSocket, nullptr, nullptr); 
  
    // // recieving data 
    // char buffer[1024] = { 0 }; 
    // recv(clientSocket, buffer, sizeof(buffer), 0); 
    // Serial.println(buffer);

    // // closing the socket. 
    // close(serverSocket);

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

  // Create the encoder objects after the motor has
  // stopped, else some sort exception is triggered
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);


while(!status){


  /*
  
  MODE 1: FORWARD LINE
  
  */



/*
START IN CENTER OF WHITE BLOCK
ACTIVATE PID MOVEMENT WHEN NOT ALL SENSORS READ WHITE
*/

M1_forward(0.33*PWM_VALUE);
M2_forward(0.33*PWM_VALUE);

readADC();
digitalConvert();

while(CHECK_MODE_CHANGE()){

readADC();
digitalConvert();


Serial.print("here 21");

}

M1_stop();
M2_stop();



}

status = true;

}