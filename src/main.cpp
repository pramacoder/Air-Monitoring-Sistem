/*
 * ========================================
 * Sistem Pemantauan Kualitas Udara IoT
 * ========================================
 *
 * Proyek: Indoor Air Quality Monitoring
 * Platform: ESP32 + ThingSpeak
 * IDE: PlatformIO (Cursor/VS Code)
 *
 * Author: IoT Learning Project
 * Date: 2025
 * ========================================
 */

// ============ INCLUDE LIBRARIES ============
// PENTING: Arduino.h HARUS PERTAMA untuk PlatformIO!
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============ KONFIGURASI WIFI ============
const char *ssid = "Wokwi-GUEST"; // Untuk Wokwi simulator
const char *password = "";        // Kosong untuk Wokwi

// ============ KONFIGURASI THINGSPEAK ============
const char *server = "http://api.thingspeak.com/update";
String apiKey = "UO73X0TLW6WXW6L5"; // Ganti dengan API Key ThingSpeak Anda

// ============ PIN DEFINITION ============
#define DHTPIN 15     // DHT22 data pin
#define DHTTYPE DHT22 // DHT sensor type
#define MQ2_PIN 34    // MQ2 analog pin (ADC1_CH6)
#define PM25_PIN 35   // Potensiometer untuk simulasi PM2.5 (ADC1_CH7)

// LED Pins
#define LED_GREEN 25  // LED Hijau - Kualitas BAIK
#define LED_YELLOW 26 // LED Kuning - Kualitas SEDANG
#define LED_RED 27    // LED Merah - Kualitas BURUK

// I2C LCD
#define LCD_SDA 21
#define LCD_SCL 22

// ============ OBJECT INITIALIZATION ============
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); // Alamat I2C 0x27, 16 kolom, 2 baris

// ============ THRESHOLD KUALITAS UDARA ============
struct AirQualityThresholds
{
  float temp_min = 18.0;       // °C
  float temp_max = 27.0;       // °C
  float humidity_min = 30.0;   // %
  float humidity_max = 60.0;   // %
  int gas_moderate = 1000;     // PPM (MQ2)
  int gas_unhealthy = 2000;    // PPM
  float pm25_good = 12.0;      // µg/m³
  float pm25_moderate = 35.5;  // µg/m³
  float pm25_unhealthy = 55.5; // µg/m³
} threshold;

// ============ VARIABEL GLOBAL ============
float temperature = 0.0;
float humidity = 0.0;
int gasValue = 0;
float pm25Value = 0.0;
String airQualityStatus = "INITIALIZING";
unsigned long lastUploadTime = 0;
const unsigned long uploadInterval = 20000; // Upload setiap 20 detik

// ============ FUNCTION DECLARATIONS ============
void connectWiFi();
void readSensors();
void evaluateAirQuality();
void updateLEDs();
void displayOnLCD();
void displayOnSerial();
void uploadToThingSpeak();
void testLEDs();

// ============ SETUP ============
void setup()
{
  // Inisialisasi Serial
  Serial.begin(115200);
  delay(3000); // PENTING: Tunggu Serial Monitor ready!

  // Flush serial buffer
  while (Serial.available())
    Serial.read();

  Serial.println("\n\n========================================");
  Serial.println("   SISTEM PEMANTAUAN KUALITAS UDARA");
  Serial.println("========================================\n");

  // Inisialisasi sensor DHT22
  Serial.println("[*] Initializing DHT22 sensor...");
  dht.begin();
  delay(2000); // DHT22 needs time to stabilize
  Serial.println("[✓] DHT22 sensor initialized");

  // Inisialisasi LCD
  Serial.println("[*] Initializing LCD I2C...");
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Air Quality");
  lcd.setCursor(0, 1);
  lcd.print("System Starting");
  Serial.println("[✓] LCD I2C initialized");

  // Setup LED pins
  Serial.println("[*] Initializing LED indicators...");
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // Test LED sequence
  testLEDs();
  Serial.println("[✓] LED indicators initialized");

  // Setup ADC pins
  Serial.println("[*] Initializing analog sensors...");
  pinMode(MQ2_PIN, INPUT);
  pinMode(PM25_PIN, INPUT);
  Serial.println("[✓] Analog sensors initialized");

  // Connect to WiFi
  connectWiFi();

  delay(2000);
  lcd.clear();
  Serial.println("\n[✓] System ready!");
  Serial.println("========================================\n");
}

// ============ MAIN LOOP ============
void loop()
{
  // Feed watchdog
  yield();

  // Baca semua sensor
  readSensors();

  // Evaluasi kualitas udara
  evaluateAirQuality();

  // Update LED indikator
  updateLEDs();

  // Tampilkan di LCD
  displayOnLCD();

  // Tampilkan di Serial Monitor
  displayOnSerial();

  // Upload ke ThingSpeak
  if (millis() - lastUploadTime >= uploadInterval)
  {
    uploadToThingSpeak();
    lastUploadTime = millis();
  }

  delay(2000); // Update setiap 2 detik
}

// ============ FUNGSI KONEKSI WIFI ============
void connectWiFi()
{
  Serial.print("[*] Connecting to WiFi: ");
  Serial.println(ssid);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    lcd.setCursor(attempts % 16, 1);
    lcd.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("[✓] WiFi Connected!");
    Serial.print("[*] IP Address: ");
    Serial.println(WiFi.localIP());

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
  }
  else
  {
    Serial.println("[✗] WiFi Connection Failed!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed");
    lcd.setCursor(0, 1);
    lcd.print("Check Settings");
  }
}

// ============ FUNGSI BACA SENSOR ============
void readSensors()
{
  // Baca DHT22 (Suhu & Kelembapan)
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Validasi pembacaan DHT22
  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("[!] Warning: Failed to read DHT22 sensor!");
    temperature = 25.0; // Default value
    humidity = 50.0;    // Default value
  }

  // Baca MQ2 (Gas Sensor)
  // Konversi ADC (0-4095) ke PPM simulasi (0-3000)
  int mq2Raw = analogRead(MQ2_PIN);
  gasValue = map(mq2Raw, 0, 4095, 0, 3000);

  // Baca Potensiometer (PM2.5 Simulasi)
  // Konversi ADC (0-4095) ke µg/m³ (0-500)
  int pm25Raw = analogRead(PM25_PIN);
  pm25Value = map(pm25Raw, 0, 4095, 0, 500);
}

// ============ FUNGSI EVALUASI KUALITAS UDARA ============
void evaluateAirQuality()
{
  int badScore = 0;
  int moderateScore = 0;

  // Evaluasi Suhu
  if (temperature < threshold.temp_min || temperature > threshold.temp_max)
  {
    badScore++;
  }

  // Evaluasi Kelembapan
  if (humidity < threshold.humidity_min || humidity > threshold.humidity_max)
  {
    moderateScore++;
  }

  // Evaluasi Gas
  if (gasValue >= threshold.gas_unhealthy)
  {
    badScore += 2;
  }
  else if (gasValue >= threshold.gas_moderate)
  {
    moderateScore++;
  }

  // Evaluasi PM2.5
  if (pm25Value >= threshold.pm25_unhealthy)
  {
    badScore += 2;
  }
  else if (pm25Value >= threshold.pm25_moderate)
  {
    moderateScore++;
  }

  // Tentukan status kualitas udara
  if (badScore >= 2)
  {
    airQualityStatus = "BURUK";
  }
  else if (moderateScore >= 2 || badScore >= 1)
  {
    airQualityStatus = "SEDANG";
  }
  else
  {
    airQualityStatus = "BAIK";
  }
}

// ============ FUNGSI UPDATE LED ============
void updateLEDs()
{
  // Matikan semua LED terlebih dahulu
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  // Nyalakan LED sesuai status
  if (airQualityStatus == "BAIK")
  {
    digitalWrite(LED_GREEN, HIGH);
  }
  else if (airQualityStatus == "SEDANG")
  {
    digitalWrite(LED_YELLOW, HIGH);
  }
  else if (airQualityStatus == "BURUK")
  {
    digitalWrite(LED_RED, HIGH);
  }
}

// ============ FUNGSI TAMPIL LCD ============
void displayOnLCD()
{
  lcd.clear();

  // Baris 1: Status dan Suhu
  lcd.setCursor(0, 0);
  if (airQualityStatus == "BAIK")
  {
    lcd.print("OK");
  }
  else if (airQualityStatus == "SEDANG")
  {
    lcd.print("MDRT");
  }
  else
  {
    lcd.print("BAD");
  }

  lcd.print(" T:");
  lcd.print(temperature, 1);
  lcd.print("C");

  // Baris 2: Kelembapan dan PM2.5
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(humidity, 0);
  lcd.print("% PM:");
  lcd.print(pm25Value, 0);

  // Alternatif tampilan - rotasi setiap 4 detik
  static unsigned long lastRotate = 0;
  static int displayMode = 0;

  if (millis() - lastRotate > 4000)
  {
    displayMode = !displayMode;
    lastRotate = millis();
  }

  if (displayMode == 1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gas: ");
    lcd.print(gasValue);
    lcd.print(" PPM");
    lcd.setCursor(0, 1);
    lcd.print("PM2.5: ");
    lcd.print(pm25Value, 1);
  }
}

// ============ FUNGSI TAMPIL SERIAL MONITOR ============
void displayOnSerial()
{
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│ Status: ");
  Serial.print(airQualityStatus);

  // Padding untuk alignment
  int padding = 27 - airQualityStatus.length();
  for (int i = 0; i < padding; i++)
    Serial.print(" ");
  Serial.println("│");

  Serial.println("├─────────────────────────────────────┤");

  Serial.print("│ Suhu        : ");
  Serial.print(temperature, 1);
  Serial.print(" °C");
  int tempPad = 18 - String(temperature, 1).length();
  for (int i = 0; i < tempPad; i++)
    Serial.print(" ");
  Serial.println("│");

  Serial.print("│ Kelembapan  : ");
  Serial.print(humidity, 1);
  Serial.print(" %");
  int humPad = 19 - String(humidity, 1).length();
  for (int i = 0; i < humPad; i++)
    Serial.print(" ");
  Serial.println("│");

  Serial.print("│ Gas (CO2)   : ");
  Serial.print(gasValue);
  Serial.print(" PPM");
  int gasPad = 16 - String(gasValue).length();
  for (int i = 0; i < gasPad; i++)
    Serial.print(" ");
  Serial.println("│");

  Serial.print("│ PM2.5       : ");
  Serial.print(pm25Value, 1);
  Serial.print(" µg/m³");
  int pmPad = 12 - String(pm25Value, 1).length();
  for (int i = 0; i < pmPad; i++)
    Serial.print(" ");
  Serial.println("│");

  Serial.println("└─────────────────────────────────────┘\n");
}

// ============ FUNGSI UPLOAD THINGSPEAK ============
void uploadToThingSpeak()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;

    // Bangun URL dengan parameter
    String url = String(server) + "?api_key=" + apiKey;
    url += "&field1=" + String(temperature);
    url += "&field2=" + String(humidity);
    url += "&field3=" + String(gasValue);
    url += "&field4=" + String(pm25Value);

    // Kirim status sebagai angka: 1=BAIK, 2=SEDANG, 3=BURUK
    int statusCode = 1;
    if (airQualityStatus == "SEDANG")
      statusCode = 2;
    else if (airQualityStatus == "BURUK")
      statusCode = 3;
    url += "&field5=" + String(statusCode);

    Serial.println("[↑] Uploading to ThingSpeak...");

    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0)
    {
      Serial.print("[✓] Upload successful! Response: ");
      Serial.println(http.getString());
    }
    else
    {
      Serial.print("[✗] Upload failed! Error: ");
      Serial.println(http.errorToString(httpCode));
    }

    http.end();
  }
  else
  {
    Serial.println("[!] WiFi not connected. Skipping upload.");
  }
}

// ============ FUNGSI TEST LED ============
void testLEDs()
{
  Serial.println("[*] Testing LED sequence...");

  digitalWrite(LED_GREEN, HIGH);
  delay(300);
  digitalWrite(LED_GREEN, LOW);

  digitalWrite(LED_YELLOW, HIGH);
  delay(300);
  digitalWrite(LED_YELLOW, LOW);

  digitalWrite(LED_RED, HIGH);
  delay(300);
  digitalWrite(LED_RED, LOW);
}