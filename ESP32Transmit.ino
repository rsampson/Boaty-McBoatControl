#include <WiFi.h>
#include <esp_now.h>


//MAC ADDRESS OF THE DEVICE YOU ARE SENDING TO
uint8_t remoteMac[] =  {0x3C, 0x71, 0xBF, 0x32, 0x56, 0xA4}; // wemos mini (good)
// uint8_t  remoteMac[] =  {0x5C, 0xCF, 0x7F, 0x06, 0x72, 0x6A}; // large Node MCU
//uint8_t remoteMac[] =  {0x3C, 0x71, 0xBF, 0x32, 0xC2, 0x1D}; // mini (bad)
// uint8_t remoteMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int left;
  int right;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// gpio mapping
const int buttonPin = 25;      
const int ledPin =  4;         
const int beeperPin = 27;

// joystick calibration
const int xMax = 3900;
const int xMin = 5;
const int xMid = 1835;
const int yMax = 2900;
const int yMin = 1100;
const int yMid = 1915;

int buttonState = 0;         // variable for reading the pushbutton status

void onSend(const uint8_t* mac, esp_now_send_status_t stat) {
  // Serial.println(stat == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (stat != ESP_NOW_SEND_SUCCESS) {
    digitalWrite(ledPin, LOW);
    delay(10);
    digitalWrite(ledPin, HIGH);  
    tone(beeperPin, 500, 15);  
  }
}

void setup()
{ // enabling the serial port does not cause extra current drain
  Serial.begin(115200);
  Serial.print("\r\n\r\nEsp Now Transmitter with Device MAC: ");
  Serial.println(WiFi.macAddress());

  pinMode( LED_BUILTIN, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  
  pinMode(beeperPin, OUTPUT);
  
  digitalWrite(ledPin, HIGH); 
  
  WiFi.mode(WIFI_STA); 

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onSend);

    // Register peer
  memcpy(peerInfo.peer_addr, remoteMac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  //WiFi.setTxPower(WIFI_POWER_19_5dBm);    // Set WiFi RF power output to highest level
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error adding peer");
  }

 }

int x;  // raw joystick input
int y;


void loop()
{
   x  = analogRead (34);  //gpio34 x
   y  = analogRead (35); // y

   x = constrain(x, xMin, xMax); // limit joystick output to linear region
   y = constrain(y, yMin, yMax);  
    
  // Serial.printf("x = %d y = %d\n", x, y); // usefull for joystick calibration

// motion control
// handle the special (rotate) cases first
    int drive;
    int balance;
    
   if (abs(x - xMid) < (xMid / 20) && y > yMid) { // spin right if x is small
      drive = map(y, yMid, yMax, 0, 200); 
      myData.right = -drive;
      myData.left  =  drive;   
   } 
   else if (abs(x - xMid) < (xMid / 20) && y <= yMid) { // spin left "
      drive = map(y, yMin, yMid, 200, 0); 
      myData.right =  drive;
      myData.left  = -drive;     
   }
   else {  // normal driving (no spin)

       if (x > xMid && y > yMid) { // upper right quadrent
           drive =    map(x, xMid, xMax, 0, 255);        
           balance =  map(y, yMid, yMax, 0, 255); 
           myData.right = drive - balance / 2;
           myData.left  = drive + balance / 2;                
       } 
      else if (x > xMid && y <= yMid) { // upper left quadrent
           drive =    map(x, xMid, xMax, 0, 255);        
           balance =  map(y, yMin, yMid,  255, 0); 
           myData.right = drive + balance / 2;
           myData.left  = drive - balance / 2;                
      }
      else if (x <= xMid && y > yMid) { // lower right quadrent
           drive =    map(x, xMin, xMid, 255, 0);        
           balance =  map(y, yMid, yMax, 0, 255); 
           myData.right = -drive + balance / 2;
           myData.left  = -drive - balance / 2;                
      }
      else if (x <= xMid && y <= yMid) {  // lower left quadrent
           drive =    map(x, xMin, xMid, 255, 0);        
           balance =  map(y, yMin, yMid, 255, 0); 
           myData.right = -drive - balance / 2;
           myData.left  = -drive + balance / 2;                
      }
   }

   if (abs(myData.left)  < 3) myData.left  = 0; // cut off motors if insignificant drive
   if (abs(myData.right) < 3) myData.right = 0;

   myData.left  = constrain(myData.left,  -255, 255); // limit motor drive to pwm limit
   myData.right = constrain(myData.right, -255, 255);  
      
   Serial.printf("right = %d  left  = %d\n", myData.right, myData.left);
 
 
    // Send message via ESP-NOW
  esp_err_t result = esp_now_send(remoteMac, (uint8_t *) &myData, sizeof(myData));
   
//  if (result == ESP_OK) {
//    Serial.println("Sent with success");
//  }
//  else {
//    Serial.println("Error sending the data");
//  }
 // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    // turn LED off:
    digitalWrite(ledPin, HIGH);
    noTone(beeperPin);
  } else {
    // turn LED on:
    digitalWrite(ledPin, LOW);
    tone(beeperPin, 500, 15);
  }
      
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  
  delay(100); 
}
