/*
 * WORC Remote Controller - ESP32 ESP-NOW Remote Controller
 * Copyright (C) 2026 wallieonline (https://github.com/wallieonline)
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * Additional terms under section 7 of the GPL apply (attribution and
 * naming, see the NOTICE file). This copyright notice and the
 * attribution may not be removed.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

//Broadcast MAC address - no pairing needed on the transmitter side
uint8_t broadcastAddress1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Bind mode settings
#define PAIRING_TIMEOUT_MS 5000 //Send BIND signal for 5 seconds after boot
const uint8_t BIND_MSG[] = "WORC"; //Small bind payload, no MAC in payload (receiver reads sender MAC from callback)

//Assign pins
#define FLED_OUT_PIN 2
#define CPPM_IN_PIN 12
#define VRX_IN_PIN 36
#define VRY_IN_PIN 39

//Assign failsafe values
#define RC_NEUTRAL 1500 //Used for failsafe
#define RC_MIN 1000 //Used for failsafe
#define RC_MAX 2000 //Used for failsafe

//Assign CPPM and STICK channel order
#define AIL 0
#define ELE 1
#define THR 2
#define RUD 3
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7
static volatile int gCppm[8] = {RC_NEUTRAL, RC_NEUTRAL, RC_MIN, RC_NEUTRAL, RC_MIN, RC_MIN, RC_MIN, RC_MIN}; //Used in ISR, failsafe defaults
volatile bool gGotCppm; //Used in ISR
int gStick[8] = {RC_NEUTRAL, RC_NEUTRAL, RC_MIN, RC_NEUTRAL, RC_MIN, RC_MIN, RC_MIN, RC_MIN}; //Used in loop, failsafe defaults
bool gGotStick; //Used in loop

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

//Assign stick or cppm mode
#define MODE_FAILS 0
#define MODE_STICK 1 //Mode for two-axis analog stick on ESP32 ADC inputs
#define MODE_CPPM 2 //Mode for CPPM on CPPM_IN_PIN input
uint8_t gMode = MODE_FAILS;

esp_now_peer_info_t peerInfo;

int myReadAnalogAvg(int pin, uint8_t samples = 5)
{
  long total = 0;
  for (uint8_t i = 0; i < samples; i++) {
    total += analogRead(pin);
  }
  int average = total / samples;
  return average;
}

int myProcessInput(int input, int inputmin, int inputmax, int expo)
{
  float inputexpo = map(input, inputmin, inputmax, -1000, 1000);
  inputexpo = inputexpo / 1000;
  //https://www.rcgroups.com/forums/showthread.php?375044-what-is-the-formula-for-the-expo-function/page2
  float outputexpo = -(((1.0 - ((100.0 - expo) / 100.0)) * pow(inputexpo, 3.0)) + (inputexpo * ((100.0 - expo) / 100.0)));
  outputexpo = outputexpo * 1000;
  return map((long)round(outputexpo), -1000, 1000, 1000, 2000);
}

IRAM_ATTR void myReadCPPM2() //With IRAM_ATTR you put the function on the RAM
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
      pulse = 0; //Start each frame with a clean pulse measurement
  }
  else if (channels < sizeof(gCppm) / sizeof(gCppm[0])) { //servo values between 510us and (510us+1910us)=2420us will end up here
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
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  Serial.begin(115200);
  pinMode(FLED_OUT_PIN, OUTPUT);
  myInitCPPM2();
  //Check for two-axis analog stick on ESP32 ADC inputs
  if (analogRead(VRX_IN_PIN) == 0 && analogRead(VRY_IN_PIN) == 0) {
    gGotStick = false;
  } else {
    gGotStick = true;
  }
  //Set device as a Wi-Fi Station
  esp_netif_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.ampdu_tx_enable = 0;
  cfg.ampdu_rx_enable = 0;
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();
  esp_wifi_set_max_tx_power(68); //17 dBm (68 x 0.25 dBm) Medium power (~50mW)
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);   // lock the channel (1, 6, or 11 are the non-overlapping ones)
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW, restarting...");
    delay(1000);
    ESP.restart();
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
    Serial.println("Failed to add peer, restarting...");
    delay(1000);
    ESP.restart();
  }
}

void loop()
{
  static uint32_t current_millis;
  static uint32_t last_cppm_millis = 0;
  static uint32_t last_stick_millis = 0;
  static uint32_t last_bind_millis = 0;
  static uint32_t last_fails_millis = 0;
  current_millis = millis();
  //Bind mode: broadcast BIND signal for PAIRING_TIMEOUT_MS after boot
  if (current_millis < PAIRING_TIMEOUT_MS) {
    digitalWrite(FLED_OUT_PIN, (current_millis / 50) % 2); //Blink fast while pairing
    if (current_millis >= last_bind_millis + 50) {
      last_bind_millis = current_millis;
      esp_now_send(broadcastAddress1, BIND_MSG, sizeof(BIND_MSG) - 1); //Broadcast BIND signal
    }
    return;
  }
  digitalWrite(FLED_OUT_PIN, HIGH); //LED solid on

  if (gGotCppm) {
    gMode = MODE_CPPM;
    gGotCppm = false;
    last_cppm_millis = current_millis;
    noInterrupts(); //Protect gCppm copy against myReadCPPM2 ISR
    if (gCppm[AIL]) gEspn.gAIL = gCppm[AIL];
    if (gCppm[ELE]) gEspn.gELE = gCppm[ELE];
    if (gCppm[THR]) gEspn.gTHR = gCppm[THR];
    if (gCppm[RUD]) gEspn.gRUD = gCppm[RUD];
    if (gCppm[AUX1]) gEspn.gAUX1 = gCppm[AUX1];
    if (gCppm[AUX2]) gEspn.gAUX2 = gCppm[AUX2];
    if (gCppm[AUX3]) gEspn.gAUX3 = gCppm[AUX3];
    if (gCppm[AUX4]) gEspn.gAUX4 = gCppm[AUX4];
    interrupts();
    esp_now_send(broadcastAddress1, (uint8_t *) &gEspn, sizeof(gEspn)); //Broadcast to all receivers
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
    gStick[AIL] = myProcessInput(myReadAnalogAvg(VRX_IN_PIN), 0, 4095, 50);
    gStick[ELE] = myProcessInput(myReadAnalogAvg(VRY_IN_PIN), 0, 4095, 1);
    gStick[THR] = RC_MIN; //Failsafe values for channels not driven by the stick
    gStick[RUD] = RC_NEUTRAL;
    gStick[AUX1] = RC_MIN;
    gStick[AUX2] = RC_MIN;
    gStick[AUX3] = RC_MIN;
    gStick[AUX4] = RC_MIN;
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
    esp_now_send(broadcastAddress1, (uint8_t *) &gEspn, sizeof(gEspn)); //Broadcast to all receivers
    //Serial.print("MODE_STICK"); Serial.print("  ");
    //Serial.print(analogRead(VRX_IN_PIN)); Serial.print("  ");
    //Serial.print(analogRead(VRY_IN_PIN)); Serial.print("  ");
    //Serial.print(gStick[0]); Serial.print("  ");
    //Serial.print(gStick[1]); Serial.print("  ");
    //Serial.print(gStick[2]); Serial.print("  ");
    //Serial.print(gStick[3]); Serial.println();
  }

  if (gMode == MODE_FAILS && current_millis >= last_fails_millis + 20) {
    last_fails_millis = current_millis;
    gEspn.gAIL = RC_NEUTRAL;
    gEspn.gELE = RC_NEUTRAL;
    gEspn.gTHR = RC_MIN;
    gEspn.gRUD = RC_NEUTRAL;
    gEspn.gAUX1 = RC_MIN;
    gEspn.gAUX2 = RC_MIN;
    gEspn.gAUX3 = RC_MIN;
    gEspn.gAUX4 = RC_MIN;
    esp_now_send(broadcastAddress1, (uint8_t *) &gEspn, sizeof(gEspn)); //Keep the link alive with failsafe values
    //Serial.print("MODE_FAILS"); Serial.print("  ");
    //Serial.print(RC_NEUTRAL); Serial.print("  ");
    //Serial.print(RC_NEUTRAL); Serial.print("  ");
    //Serial.print(RC_MIN); Serial.print("  ");
    //Serial.print(RC_NEUTRAL); Serial.println();
  }
}