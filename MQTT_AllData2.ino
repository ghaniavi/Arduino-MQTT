#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <DHT.h>

const char* ssid = "Cisco02640";
const char* password = "Bysonics123";
const char* mqtt_server = "192.168.1.119";

WiFiClient espClient;
PubSubClient client(espClient);

int PulseSensorPurplePin = A0;
//int led = 22;
//int DayaHR = 5;
//#define ONE_WIRE_BUS 26
#define DHTPIN 33
#define EMG_PIN A0
const uint8_t scl = 14;
const uint8_t sda = 12;
// For Heart Rate
//int number = 0;
//int Signal;
//int HasilHR;

//Accelerometer
int X, Y, Z;

//DHT11
float h;
float t;
float f;
float hif;
float hic;

char EMG[10];
char AC_X2[10];
char AC_Y2[10];
char AC_Z2[10];
char DHTTC[10];
char DHTTF[10];
char DHTH[10];

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const char* Topic3 = "Suhu";
const char* Topic4 = "AC_X2";
const char* Topic5 = "AC_Y2";
const char* Topic6 = "AC_Z2";
const char* Topic7 = "DHT";
const char* Topic8 = "EMG";

// For Accelerometer
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

// For DS18B20
// Setup a oneWire instance to communicate with a OneWire device
//OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
//DallasTemperature sensors(&oneWire);
//DeviceAddress sensor1 = { 0x28, 0x95, 0x21, 0xE1, 0x2F, 0x14, 0x1, 0xF5 };

double Ax, Ay, Az, T, Gx, Gy, Gz;

void setup() {
  Serial.begin(115200);
  setup_wifi();
  Wire.begin(sda, scl);
  MPU6050_Init();
  //sensors.begin();
  client.setServer(mqtt_server, 1883);
  dht.begin();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  accelerometer();
  dht11();
  
  //Signal = analogRead(PulseSensorPurplePin);
  //HasilHR = Signal * 200 / 4096;

  //sensors.requestTemperatures();
  int value = analogRead(EMG_PIN);
  int data1 = value;
  
  int datarand = rand() % 1000;              //Random
  //int data2 = HasilHR;                      //Heart Rate
  //int data3 = sensors.getTempCByIndex(0);   //DS18B20
  int data4 = X;                            //AC_X
  int data5 = Y;                            //AC_Y
  int data6 = Z;                            //AC_Z
  int data7 = h;                            //Humidity
  int data8 = t;                            //Temperature °C

  sprintf(EMG, "%i", datarand);
  sprintf(AC_X2, "%i", data4);
  sprintf(AC_Y2, "%i", data5);
  sprintf(AC_Z2, "%i", data6);
  sprintf(DHTH, "%i", data7);
  sprintf(DHTTC, "%i", data8);

  client.publish("EMG", EMG);
  client.publish("AC_X2", AC_X2);
  client.publish("AC_Y2", AC_Y2);
  client.publish("AC_Z2", AC_Z2);
  client.publish("DHTTC", DHTTC);
  client.publish("DHTH", DHTH);

  client.loop();
  delay(100);
}

void dht11() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(t, h, false);

}

void accelerometer() {
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  Ax = (double)AccelX / AccelScaleFactor;
  Ay = (double)AccelY / AccelScaleFactor;
  Az = (double)AccelZ / AccelScaleFactor;
  T = (double)Temperature / 340 + 36.53;                              //temperature formula
  Gx = (double)GyroX / GyroScaleFactor;
  Gy = (double)GyroY / GyroScaleFactor;
  Gz = (double)GyroZ / GyroScaleFactor;
  X = Ax * 180;
  Y = Ay * 180;
  Z = Az * 180;
}

void setup_wifi() {
  // Connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  Serial.println("In reconnect...");
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Arduino_Moisture")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
