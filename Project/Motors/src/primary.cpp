
#include <WiFi.h>
#include <sys/socket.h>

#include <Arduino.h>

#include <Encoder.h>

#include <Adafruit_MCP3008.h>

#include "primary.hpp"






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

  /*
  MPU setup stuff
  */
  while (!mpu.begin()) delay(10);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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


  START_BLOCK();

  MODE_CHANGE_BLOCK_RIGHT(enc1, enc2);

  MOVE_FORWARD(2,90,enc1,enc2);

  SKIP_MAZE(enc1, enc2);

  MODE_CHANGE_BLOCK_LEFT(enc1, enc2);

  KESSEL();

  ASTEROIDS( enc1 , enc2);

  MODE_CHANGE_BLOCK_RIGHT(enc1,enc2);


  DUAL_FATES(enc1,enc2);

  MODE_CHANGE_BLOCK_LEFT(enc1, enc2);

  MODE_1(90,6,120,0);

  
  MOVE_FORWARD(600,90,enc1, enc2);


  MODE_CHANGE_BLOCK_LEFT(enc1, enc2);

  ENDOR_DASH(enc1, enc2);
 

}
