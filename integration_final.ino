#include <Servo.h>
#include <EEPROM.h>            

#include "FS.h" 
#include "SD_MMC.h"
#include "Arduino.h"
#include "soc/soc.h"
#include "esp_camera.h"         
#include "driver/rtc_io.h"    
#include "soc/rtc_cntl_reg.h"

#define SensorLeft    9   
#define SensorMiddle  10   
#define SensorRight   11   
unsigned char SL;        
unsigned char SM;       
unsigned char SR;        
unsigned char Lpwm_val =120;
unsigned char Rpwm_val = 120;
int Car_state=0;             
unsigned char old_SL,old_SM,old_SR;

// Define camera configurations
#define EEPROM_SIZE 1
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Initialize pins and rotate angle for robot arm
Servo myservo;
#define servoPin 0
int angle = 90;

// Initialize pins for wheel servo motors
int IN1=7;
int IN2=5;
int IN3=4;
int IN4=2;
int ENA=6;
int ENB=3;

// Initialize pic numbers
int pictureNumber = 0;

void Sensor_IO_Config() {
  pinMode(SensorLeft,INPUT);
  pinMode(SensorMiddle,INPUT);
  pinMode(SensorRight,INPUT);
}

void Sensor_Scan(void) {
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

void M_Control_IO_config() {
  pinMode(IN4,OUTPUT); 
  pinMode(IN3,OUTPUT); 
  pinMode(IN1,OUTPUT); 
  pinMode(IN2,OUTPUT); 
  pinMode(ENA,OUTPUT); 
  pinMode(ENB,OUTPUT); 
}

void Set_Speed(unsigned char Left,unsigned char Right) {
  analogWrite(ENA,Left);   
  analogWrite(ENV,Right);
}

void advance() {
     digitalWrite(IN3,LOW);  
     digitalWrite(IN4,HIGH);
     digitalWrite(IN1,LOW);  
     digitalWrite(IN2,HIGH); 
     Car_state = 1;  
}

void turnR() {
     digitalWrite(IN3,LOW);  
     digitalWrite(IN4,HIGH);
     digitalWrite(IN1,HIGH);
     digitalWrite(IN2,LOW); 
     Car_state = 4;
}

void turnL() {
     digitalWrite(IN3,HIGH);
     digitalWrite(IN4,LOW);   
     digitalWrite(IN1,LOW);   
     digitalWrite(IN2,HIGH);
     Car_state = 3;
}  
  
void stopp() {
     digitalWrite(IN3,HIGH);
     digitalWrite(IN4,HIGH);   
     digitalWrite(IN1,HIGH);   
     digitalWrite(IN2,HIGH);
     Car_state = 5;
}

void back() {
     digitalWrite(IN3,HIGH);  
     digitalWrite(IN4,LOW);
     digitalWrite(IN1,HIGH);  
     digitalWrite(IN2,LOW);
     Car_state = 2;   
}


void setup() {
  Sensor_IO_Config();
  M_Control_IO_config();        
  Set_Speed(Lpwm_val,Rpwm_val); 
  stopp();
   
  // Initialize robot arm
  myservo.attach(servoPin);

  //Initialize wheel servo motors
  for (int i = 5; i <11; i ++) {
    pinMode(i, OUTPUT);  
  }

  //Initailize cameras
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Initialize Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    return;
  }
  
  if(!SD_MMC.begin()){
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    return;
  }
    
  camera_fb_t* fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    return;
  }
  
  // Initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(file){
    file.write(fb->buf, fb->len);
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb); 
  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);

  // Pause for 2 seconds
  delay(2000);
  esp_deep_sleep_start();
}

void run_pause_arm() {
  // Run the wheel servo motors
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,200);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,HIGH);
  analogWrite(ENB,200);
  delay(2000);
  
  // Pause for 2 seconds
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  delay(2000);

  // Back to 0 degree
  myservo.write(0);
  delay(1000);
  // Sweep from 0 to 90 degrees:
  for (angle = 0; angle <= 90; angle += 1) {
    myservo.write(angle);
    delay(15);
  }

  // Back from 90 to 0 degrees:
  for (angle = 90; angle >= 0; angle -= 1) {
    myservo.write(angle);
    delay(30);
  }

  // Run the wheel servo motors
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,200);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENB,200);
  delay(2000);

  // Pause for 2 seconds
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  delay(2000);
}

void line_tracking(){
  Sensor_Scan();
  if (SM == HIGH){
    if (SL == LOW & SR == HIGH) {
      turnR();
    } else if (SR == LOW & SL == HIGH) {
      turnL();
    } else 
      {
        advance();
      }
    }
  else {
    if (SL== LOW & SR == HIGH) {
      turnR();
    }
    else if (SR == LOW & SL == HIGH) {
      turnL();
    }
    else {
      back();
      delay(100);
      stopp() ; 
    }
  }  
}

void loop() { 
  line_tracking();
  run_pause_arm();
}
