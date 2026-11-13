
#include <ESP8266WiFi.h>
#include <espnow.h>

#define FORWARD_FACING



#ifdef FORWARD_FACING
#define mot_right_p  15  // D8  props face rearward
#define mot_right_n  13  // D7
#define mot_left_p   12  // D6
#define mot_left_n   14  // D5
#else
#define mot_right_n  15  // D8  props face rearward
#define mot_right_p  13  // D7
#define mot_left_n   12  // D6
#define mot_left_p   14  // D5
#endif

typedef struct struct_message {  // Must match the sender structure
  int left;
  int right;
} struct_message;


// Create a struct_message called myData
struct_message myData;

bool got_something;

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
  {
    memcpy(&myData, incomingData, sizeof(myData));

    got_something = true;
  }

void setup()
{
  Serial.begin(115200);
  Serial.print("\r\n\r\nEsp Now Recieving with Device MAC: ");
  Serial.println(WiFi.macAddress());
  
  pinMode( LED_BUILTIN, OUTPUT);
  pinMode( mot_left_p, OUTPUT); 
  pinMode( mot_left_n, OUTPUT);
  pinMode( mot_right_p, OUTPUT); 
  pinMode( mot_right_n, OUTPUT); 

  analogWriteRange(255);  // this will set the sensitivity of the motors to control
     
  digitalWrite( LED_BUILTIN, LOW);
  delay(500);
  digitalWrite( LED_BUILTIN, HIGH);  

  WiFi.mode(WIFI_STA); 
  
  if (esp_now_init() != 0)  
  {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  if (esp_now_set_self_role(ESP_NOW_ROLE_SLAVE) != 0) {
    Serial.println("Error setting role");
  }

  esp_now_register_recv_cb(OnDataRecv);

  //WiFi.setOutputPower(20.5);    // Sets WiFi RF power output to highest level, highest RF power usage
}

void kill_motors(){
  analogWrite(mot_right_n, 0);  // shut down all motors if signal timeout
  analogWrite(mot_right_p, 0);       
  analogWrite(mot_left_n, 0); 
  analogWrite(mot_left_p, 0); 
  digitalWrite(mot_right_n, 0);  
  digitalWrite(mot_right_p, 0); 
  digitalWrite(mot_left_n, 0);  
  digitalWrite(mot_left_p, 0);  
}   

unsigned long lastTime = 0;  // create timeout to shut down motors if communication is lost
unsigned long timerDelay = 500; 

void loop()
{
  if(got_something == true ) {
    got_something = false;
    lastTime = millis();
        
    digitalWrite(LED_BUILTIN, LOW);  // indicate reception of data
    delay(10);
    digitalWrite(LED_BUILTIN, HIGH);


      if(myData.left < 0) {
        analogWrite(mot_left_p, -myData.left);
        digitalWrite(mot_left_n, 0);  // ground other terminal of motor
      } else {
        analogWrite(mot_left_n, myData.left);
        digitalWrite(mot_left_p, 0); 
      }
      
      
      if(myData.right < 0) {
        analogWrite(mot_right_p, -myData.right);
        digitalWrite(mot_right_n, 0); 
      } else {
        analogWrite(mot_right_n, myData.right);
        digitalWrite(mot_right_p, 0);  
      }
      
      Serial.printf("right = %d left = %d\n", myData.right, myData.left); 
  }  
      
    if ((millis() - lastTime) > timerDelay){  // dead man timeout
       kill_motors();
    }
}
