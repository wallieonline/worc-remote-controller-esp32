#define CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED 0
#define CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED 0

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
//#include <Ps3Controller.h>

//Assign pins
#define FLED_OUT_PIN 2
#define CPPM_IN_PIN 12
#define VRX_IN_PIN 36
#define VRY_IN_PIN 39

//Assign CPPM and STICK channel order
#define AIL 0
#define ELE 1
#define THR 2
#define RUD 3
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7
volatile static int gCppm[8]; //Used in ISR
volatile boolean gGotCppm; //Used in ISR
int gStick[8];
boolean gGotStick;

//Assign ESPNOW channel struct
typedef struct rc_data_t {
  uint16_t gAIL;
  uint16_t gELE;
  uint16_t gTHR;
  uint16_t gRUD;
  uint16_t gAUX1;
  uint16_t gAUX2;
  uint16_t gAUX3;
  uint16_t gAUX4;
} rc_data_t;
rc_data_t gEspn;

//Assign failsafe values
#define RC_NEUTRAL 1500//Used for failsafe
#define RC_MIN 1000 //Used for failsafe
#define RC_MAX 2000 //Used for failsafe

//Assign stick or cppm mode
#define MODE_FAILS 0
#define MODE_STICK 1 //Mode for two-axis analog stick on ESP32 ADC inputs
#define MODE_CPPM 2 //Mode for CPPM on CPPN_IN_PIN input
#define MODE_SIXAXIS 3 //Mode for PS3 six-axis controller on bluetooth
uint8_t gMode = MODE_FAILS;

esp_now_peer_info_t peerInfo;

//Assign PS3-MAC address
uint8_t gClonedMac[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

//Assign RX-MAC address
uint8_t broadcastAddress1[] = {0x64, 0xE8, 0x33, 0x88, 0x20, 0x90}; //64:E8:33:88:20:90 - ESP32-C3 Robot Controller
uint8_t broadcastAddress2[] = {0x3C, 0x71, 0xBF, 0x26, 0x5B, 0xFD}; //3C:71:BF:26:5B:FD - ESP8266 Monitoring Module

float myProcessInput(int input, int inputmin, int inputmax, int expo)
{
  float inputexpo = map(input, inputmin, inputmax, -1000, 1000);
  inputexpo = inputexpo / 1000;
  //https://www.rcgroups.com/forums/showthread.php?375044-what-is-the-formula-for-the-expo-function/page2
  float outputexpo = -(((1.0 - ((100.0 - expo) / 100.0)) * pow(inputexpo, 3.0)) + (inputexpo * ((100.0 - expo) / 100.0)));
  outputexpo = outputexpo * 1000;
  int output = map(outputexpo, -1000, 1000, 1000, 2000);
  return output;
}

ICACHE_RAM_ATTR void myReadCPPM2() //With ICACHE_RAM_ATTR you put the function on the RAM
{
  static uint32_t current_micros;
  static uint32_t last_micros = 0; //needs to be static
  static uint32_t counter;
  static uint32_t pulse;
  static uint8_t channels = 0; //needs to be static
  current_micros = micros(); //You can call on micros() to find out the current time within your ISR
  counter = (current_micros - last_micros);
  last_micros = current_micros;
  if (counter < 510) { //must be a pulse if less than 510us normal pulse is 300us
      pulse = counter;
  }
  else if (counter > 1910) { //sync pulses over 1910us
      channels = 0;
  }
  else if (channels < sizeof(gCppm)) { //servo values between 510us and (510us+1910us)=2420us will end up here
      gCppm[channels] = (counter + pulse - 14); //Interupt latency of 14us
      channels++;
  }
  if (channels == 3) gGotCppm = true; //Change number of channels to determine if you GotCppm data normally 8
}

void myInitCPPM2()
{
  pinMode(CPPM_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CPPM_IN_PIN), myReadCPPM2, CHANGE);
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  Serial.begin(115200);
  pinMode(FLED_OUT_PIN, OUTPUT);
  //pinMode(VRX2_IN_PIN, INPUT);
  //pinMode(VRY2_IN_PIN, INPUT);
  myInitCPPM2();
  //ps3SetBluetoothMacAddress(gClonedMac);
  //Ps3.begin(gClonedMac);
  //Check for two-axis analog stick on ESP32 ADC inputs
  if (analogRead(VRX_IN_PIN) == 0 && analogRead(VRY_IN_PIN) == 0) {
    gGotStick = false;
  } else {
    gGotStick = true;
  }

  //Set device as a Wi-Fi Station
  //old //WiFi.enableLongRange(true);
  //old //WiFi.setSleep(false);
  //old //WiFi.mode(WIFI_STA);
  //old //WiFi.disconnect();
  esp_netif_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();
  
  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  //Register for Send CB
  esp_now_register_send_cb(OnDataSent);
  //Register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  Serial.println("Add peer...");
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  //register second peer
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  Serial.println("Add peer...");
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  delay(2000);
  digitalWrite(FLED_OUT_PIN, HIGH);
}

void loop()
{
  static uint32_t current_millis;
  static uint32_t last_cppm_millis = 0;
  static uint32_t last_stick_millis = 0;
  //delay(20); //Less load on MCU better for ISR?
  current_millis = millis();
  if (gGotCppm) {
    gMode = MODE_CPPM;
    gGotCppm = false;
    last_cppm_millis = current_millis;
    if (gCppm[AIL]) gEspn.gAIL = gCppm[AIL];
    if (gCppm[ELE]) gEspn.gELE = gCppm[ELE];
    if (gCppm[THR]) gEspn.gTHR = gCppm[THR];
    if (gCppm[RUD]) gEspn.gRUD = gCppm[RUD];
    if (gCppm[AUX1]) gEspn.gAUX1 = gCppm[AUX1];
    if (gCppm[AUX2]) gEspn.gAUX2 = gCppm[AUX2];
    if (gCppm[AUX3]) gEspn.gAUX3 = gCppm[AUX3];
    if (gCppm[AUX4]) gEspn.gAUX4 = gCppm[AUX4];
    esp_now_send(0, (uint8_t *) &gEspn, sizeof(gEspn)); //Send to all peers
    //esp_now_send(broadcastAddress1, (uint8_t *) &gEspn, sizeof(gEspn));
    //Serial.print("MODE_CPPM"); Serial.print("  ");
    //Serial.print(gCppm[0]); Serial.print("  ");
    //Serial.print(gCppm[1]); Serial.print("  ");
    //Serial.print(gCppm[2]); Serial.print("  ");
    //Serial.print(gCppm[3]); Serial.println();
  } else if (current_millis >= last_cppm_millis + 200 && gGotStick == true) { //Switch back to STICK_MODE if GotCppm is false and GotStick is true
    gMode = MODE_STICK;
  } else if (current_millis >= last_cppm_millis + 200 && gGotStick == false) { //Switch back to FAILS_MODE if GotCppm is false and GotStick is false
    gMode = MODE_FAILS;
  }
  
  if (gMode == MODE_STICK && current_millis >= last_stick_millis + 20) {
    last_stick_millis = current_millis;
    gStick[AIL] = myProcessInput(analogRead(VRX_IN_PIN), 0, 4095, 50);
    gStick[ELE] = myProcessInput(analogRead(VRY_IN_PIN), 0, 4095, 1);
    //gStick[AIL] = map(analogRead(VRX_IN_PIN), 0, 4095, 2000, 1000);
    //gStick[ELE] = map(analogRead(VRY_IN_PIN), 0, 4095, 2000, 1000);
    if (gStick[AIL]) gEspn.gAIL = gStick[AIL];
    if (gStick[ELE]) gEspn.gELE = gStick[ELE];
    if (gStick[THR]) gEspn.gTHR = gStick[THR];
    if (gStick[RUD]) gEspn.gRUD = gStick[RUD];
    if (gStick[AUX1]) gEspn.gAUX1 = gStick[AUX1];
    if (gStick[AUX2]) gEspn.gAUX2 = gStick[AUX2];
    if (gStick[AUX3]) gEspn.gAUX3 = gStick[AUX3];
    if (gStick[AUX4]) gEspn.gAUX4 = gStick[AUX4];
    esp_now_send(0, (uint8_t *) &gEspn, sizeof(gEspn)); //Send to all peers
    //esp_now_send(broadcastAddress1, (uint8_t *) &gEspn, sizeof(gEspn));
    //Serial.print("MODE_STICK"); Serial.print("  ");
    //Serial.print(analogRead(VRX_IN_PIN)); Serial.print("  ");
    //Serial.print(analogRead(VRY_IN_PIN)); Serial.print("  ");
    //Serial.print(gStick[0]); Serial.print("  ");
    //Serial.print(gStick[1]); Serial.print("  ");
    //Serial.print(gStick[2]); Serial.print("  ");
    //Serial.print(gStick[3]); Serial.println();
  }

  if (gMode == MODE_FAILS) {
    gEspn.gAIL = RC_NEUTRAL;
    gEspn.gELE = RC_NEUTRAL;
    gEspn.gTHR = RC_MIN;
    gEspn.gRUD = RC_NEUTRAL;
    gEspn.gAUX1 = RC_MIN;
    gEspn.gAUX2 = RC_MIN;
    gEspn.gAUX3 = RC_MIN;
    gEspn.gAUX4 = RC_MIN;
    Serial.print("MODE_FAILS"); Serial.print("  ");
    Serial.print(RC_NEUTRAL); Serial.print("  ");
    Serial.print(RC_NEUTRAL); Serial.print("  ");
    Serial.print(RC_MIN); Serial.print("  ");
    Serial.print(RC_NEUTRAL); Serial.println();
  }
}