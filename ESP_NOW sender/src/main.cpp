/*#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>

// Broadcast address for pairing
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t myMACAddress[6];

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// Structure to hold sensor data
typedef struct struct_message {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} struct_message;

// Create a struct_message called myData
struct_message myData;

void printMAC(const uint8_t *mac, const char *label) {
  Serial.print(label);
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

void sendMACAddress() {
  esp_wifi_get_mac(WIFI_IF_STA, myMACAddress);
  printMAC(myMACAddress, "Sending our MAC address: ");
  esp_err_t result = esp_now_send(broadcastAddress, myMACAddress, sizeof(myMACAddress));
  if (result == ESP_OK) {
    Serial.println("Sent MAC address successfully");
  } else {
    Serial.printf("Error sending MAC address: %d\n", result);
  }
}

void sendSensorData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Set values to send
  myData.ax = a.acceleration.x;
  myData.ay = a.acceleration.y;
  myData.az = a.acceleration.z;
  myData.gx = g.gyro.x;
  myData.gy = g.gyro.y;
  myData.gz = g.gyro.z;

  // Send sensor data via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent sensor data successfully");
  } else {
    Serial.printf("Error sending sensor data: %d\n", result);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; // Default channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }
  Serial.println("Sender is ready to send MAC address");

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  sendMACAddress();
  sendSensorData();
  delay(3000); // Send data every 3 seconds
}*/
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#define TAM 8//limite de dados a serem enviados no maximo 8 um array de 8

// Endereço do solicitante
uint8_t requesterMAC[6];

// MPU6050 sensor
Adafruit_MPU6050 mpu;
unsigned long timestamp=0;
// Estrutura para armazenar dados do sensor
typedef struct struct_message {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  int tempo;
} struct_message;

// Array de estruturas struct_message para armazenar os dados
struct_message myData[TAM];

// Flag para indicar quando enviar dados
bool sendDataFlag = false;

void printMAC(const uint8_t *mac, const char *label) {
  Serial.print(label);
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

// Callback para receber dados e ativar o envio
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.println("Received signal to send data");
  memcpy(requesterMAC, mac_addr, 6); // Armazena o MAC do solicitante
  printMAC(requesterMAC, "Responder para: ");
  sendDataFlag = true;
  timestamp=millis();
}

void sendSensorData() {
  // Configura o peer (solicitante) dinamicamente
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, requesterMAC, 6);
  peerInfo.channel = 0; // Canal padrão (ajuste se necessário)
  peerInfo.encrypt = false;

  // Adiciona o peer se ainda não foi adicionado
  if (!esp_now_is_peer_exist(requesterMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  // Coleta e armazena os dados do sensor
  for (int count = 0; count < TAM; count++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
  
    myData[count].ax = a.acceleration.x;
    myData[count].ay = a.acceleration.y;
    myData[count].az = a.acceleration.z;
    myData[count].gx = g.gyro.x;
    myData[count].gy = g.gyro.y;
    myData[count].gz = g.gyro.z;
    myData[count].tempo = millis()-timestamp;
    Serial.print("Data ");
    Serial.print(count);
    Serial.print(": ax=");
    Serial.print(myData[count].ax);
    Serial.print(", ay=");
    Serial.print(myData[count].ay);
    Serial.print(", az=");
    Serial.print(myData[count].az);
    Serial.print(", gx=");
    Serial.print(myData[count].gx);
    Serial.print(", gy=");
    Serial.print(myData[count].gy);
    Serial.print(", gz=");
    Serial.println(myData[count].gz);
    Serial.print(", tempo=");
    Serial.println(myData[count].tempo);

    delay(100); // Ajuste o delay conforme necessário
    
  }

  // Envia os dados via ESP-NOW
  esp_err_t result = esp_now_send(requesterMAC, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent sensor data successfully");
  } else {
    Serial.printf("Error sending sensor data: %d\n", result);
  }

  // Remove o peer após o envio para liberar memória
  esp_now_del_peer(requesterMAC);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Registra o callback para receber dados
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW iniciado e callback registrado");

  // Inicializa o sensor MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  // Envia os dados se o flag estiver ativado
  if (sendDataFlag) {
    sendSensorData();
    sendDataFlag = false;
  }
}