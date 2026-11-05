#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// MPU6050
// =====================================================

#define MPU6050_ADDR       0x68
#define MPU_PWR_MGMT_1     0x6B
#define MPU_ACCEL_CONFIG   0x1C
#define MPU_ACCEL_XOUT_H   0x3B
#define MPU_WHO_AM_I       0x75

// =====================================================
// GPS
// =====================================================
 
#define GPS_RX 26   // ESP32 RX <- GPS TX
#define GPS_TX 27   // ESP32 TX -> GPS RX

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

// =====================================================
// A7670C GSM / SMS
// =====================================================
// Example pins for a classic ESP32 only. Change if unavailable.
// GSM TXD -> ESP32 GSM_RX; GSM RXD <- ESP32 GSM_TX.
// Confirm physical UART pins and logic voltage before wiring.
#define GSM_RX 32
#define GSM_TX 33
#define GSM_BAUD 115200
HardwareSerial GSM_Serial(2);

// Enter the DESTINATION number that receives alerts, not the SIM's own number.
const char ALERT_PHONE_NUMBER[] = "+918939433561";
bool fallSmsSentForCurrentFall = false;
bool sosSmsSent = false;


// =====================================================
// OLED
// =====================================================

#define OLED_SDA 21
#define OLED_SCL 22

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// =====================================================
// WiFi
// =====================================================

const char* ssid = "Dharun";
const char* password = "12345678";

WebServer server(80);

// =====================================================
// FALL DETECTION
// =====================================================

#define FALL_THRESHOLD 2.5

bool fallDetected = false;
unsigned long fallTime = 0;

// Alert duration = 15 seconds
#define ALERT_DURATION 15000

// =====================================================
// BUZZER + BUTTON
// =====================================================

// Buzzer connected to D19
#define BUZZER_PIN 19

// Push button connected to D18
#define BUTTON_PIN 18

// SOS push button connected between D5 (GPIO5) and GND
#define SOS_BUTTON_PIN 5

// =====================================================
// GPS DATA
// =====================================================

double latitude = 0.0;
double longitude = 0.0;

bool gpsValid = false;

// SOS alert remains active until the ESP32 is restarted.
bool sosActive = false;
bool sosGpsValid = false;
double sosLatitude = 0.0;
double sosLongitude = 0.0;
bool sosButtonPreviouslyPressed = false;
unsigned long sosLastDebounceTime = 0;
#define SOS_DEBOUNCE_MS 40

// =====================================================
// MPU6050 DATA
// =====================================================

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float totalAcceleration = 0.0;

bool mpuConnected = false;

// =====================================================
// I2C WRITE
// =====================================================

void writeMPURegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);

  uint8_t error = Wire.endTransmission();

  if (error != 0)
  {
    Serial.print("MPU register write error: ");
    Serial.println(error);
  }
}

// =====================================================
// I2C READ
// =====================================================

uint8_t readMPURegister(uint8_t reg)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);

  uint8_t error = Wire.endTransmission(false);

  if (error != 0)
  {
    Serial.print("MPU register read error: ");
    Serial.println(error);
    return 0xFF;
  }

  Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);

  if (Wire.available())
  {
    return Wire.read();
  }

  return 0xFF;
}

// =====================================================
// INITIALIZE MPU6050
// =====================================================

bool initializeMPU6050()
{
  Serial.println("Initializing MPU6050...");

  delay(100);

  uint8_t whoAmI = readMPURegister(MPU_WHO_AM_I);

  Serial.print("MPU6050 WHO_AM_I = 0x");

  if (whoAmI < 0x10)
    Serial.print("0");

  Serial.println(whoAmI, HEX);

  // ---------------------------------------------------
  // Your module returns 0x70.
  // Accept 0x68, 0x69 and 0x70.
  // ---------------------------------------------------

  if (whoAmI != 0x68 &&
      whoAmI != 0x69 &&
      whoAmI != 0x70)
  {
    Serial.println("Unknown MPU device!");
    return false;
  }

  Serial.println("MPU6050-compatible device detected!");

  // Wake up MPU
  writeMPURegister(MPU_PWR_MGMT_1, 0x00);

  delay(100);

  // ---------------------------------------------------
  // Set accelerometer range to ±8G
  //
  // 0x10 = ±8G
  // Sensitivity = 4096 LSB/G
  // ---------------------------------------------------

  writeMPURegister(MPU_ACCEL_CONFIG, 0x10);

  delay(100);

  // Test reading acceleration
  int16_t testX, testY, testZ;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0)
  {
    Serial.println("Failed to access accelerometer!");
    return false;
  }

  uint8_t received = Wire.requestFrom(
    MPU6050_ADDR,
    (uint8_t)6
  );

  if (received != 6)
  {
    Serial.println("Failed to read accelerometer!");
    return false;
  }

  testX = (Wire.read() << 8) | Wire.read();
  testY = (Wire.read() << 8) | Wire.read();
  testZ = (Wire.read() << 8) | Wire.read();

  Serial.println("Initial accelerometer read successful!");

  Serial.print("Raw X: ");
  Serial.println(testX);

  Serial.print("Raw Y: ");
  Serial.println(testY);

  Serial.print("Raw Z: ");
  Serial.println(testZ);

  Serial.println("MPU6050 initialized successfully!");

  return true;
}

// =====================================================
// READ MPU6050 ACCELEROMETER
// =====================================================

bool readMPU6050()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  uint8_t received = Wire.requestFrom(
    MPU6050_ADDR,
    (uint8_t)6
  );

  if (received != 6)
  {
    return false;
  }

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // ±8G = 4096 LSB/G
  const float sensitivity = 4096.0;

  accelX = ((float)rawX / sensitivity) * 9.80665;
  accelY = ((float)rawY / sensitivity) * 9.80665;
  accelZ = ((float)rawZ / sensitivity) * 9.80665;

  totalAcceleration = sqrt(
    accelX * accelX +
    accelY * accelY +
    accelZ * accelZ
  );

  return true;
}

// =====================================================
// A7670C SMS HELPERS
// =====================================================

bool waitForGSMResponse(const char *expected, unsigned long timeoutMs)
{
  String response;
  unsigned long start = millis();
  while (millis() - start < timeoutMs)
  {
    while (GSM_Serial.available())
    {
      char c = (char)GSM_Serial.read();
      response += c;
      Serial.write(c);
      if (response.indexOf(expected) >= 0) return true;
      if (response.length() > 500) response.remove(0, 250);
    }
    delay(5);
  }
  Serial.print("\nTimed out waiting for: ");
  Serial.println(expected);
  return false;
}

bool sendSMS(const String &message)
{
  if (String(ALERT_PHONE_NUMBER).length() < 8 ||
      String(ALERT_PHONE_NUMBER).indexOf('X') >= 0)
  {
    Serial.println("SMS not sent: set ALERT_PHONE_NUMBER first.");
    return false;
  }

  while (GSM_Serial.available()) GSM_Serial.read();
  GSM_Serial.println("AT");
  if (!waitForGSMResponse("OK", 3000)) return false;

  GSM_Serial.println("AT+CMGF=1");
  if (!waitForGSMResponse("OK", 3000)) return false;

  GSM_Serial.print("AT+CMGS=\"");
  GSM_Serial.print(ALERT_PHONE_NUMBER);
  GSM_Serial.println("\"");
  if (!waitForGSMResponse(">", 10000)) return false;

  GSM_Serial.print(message);
  GSM_Serial.write(26); // Ctrl+Z sends the SMS.
  if (waitForGSMResponse("+CMGS:", 30000))
  {
    Serial.println("\nSMS submitted to modem.");
    return true;
  }
  Serial.println("\nSMS failed; check SIM, signal, SMS service and number.");
  return false;
}

String makeAlertMessage(const String &alertType)
{
  String message = "SMART WEARABLE EMERGENCY: " + alertType + ".";
  if (gpsValid)
  {
    message += "\nLatitude: " + String(latitude, 6);
    message += "\nLongitude: " + String(longitude, 6);
    message += "\nMap: https://maps.google.com/?q=";
    message += String(latitude, 6) + "," + String(longitude, 6);
  }
  else message += "\nGPS location is not available yet.";
  return message;
}

// =====================================================
// FALL DETECTION
// =====================================================

void checkFall()
{
  if (!mpuConnected)
    return;

  if (!readMPU6050())
  {
    Serial.println("Failed to read MPU6050!");
    return;
  }

  Serial.print("Acceleration: ");
  Serial.print(totalAcceleration);
  Serial.println(" m/s²");

  // ---------------------------------------------------
  // Fall threshold
  // 2.5G approximately
  // ---------------------------------------------------

  if (totalAcceleration > FALL_THRESHOLD * 9.80665)
  {
    if (!fallDetected)
    {
      fallDetected = true;
      fallTime = millis();

      // Turn ON buzzer
      digitalWrite(BUZZER_PIN, HIGH);

      Serial.println("================================");
      Serial.println("!!! FALL DETECTED !!!");
      Serial.println("BUZZER ON");
      Serial.println("================================");

      if (!fallSmsSentForCurrentFall)
      {
        fallSmsSentForCurrentFall = sendSMS(makeAlertMessage("FALL DETECTED"));
      }

      Serial.print("Acceleration = ");
      Serial.print(totalAcceleration);
      Serial.println(" m/s²");
    }
  }

  // ---------------------------------------------------
  // Turn off alert after 15 seconds
  // ---------------------------------------------------

  if (fallDetected &&
      millis() - fallTime >= ALERT_DURATION)
  {
    fallDetected = false;
    fallSmsSentForCurrentFall = false;

    // Turn OFF buzzer
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Fall alert cleared.");
    Serial.println("BUZZER OFF - 15 seconds completed.");
  }
}

// =====================================================
// BUTTON CHECK
// =====================================================

void checkButton()
{
  // Button uses INPUT_PULLUP:
  // HIGH = not pressed
  // LOW  = pressed

  if (digitalRead(BUTTON_PIN) == LOW)
  {
    if (fallDetected)
    {
      fallDetected = false;
      fallSmsSentForCurrentFall = false;

      // Immediately turn OFF buzzer
      digitalWrite(BUZZER_PIN, LOW);

      Serial.println("================================");
      Serial.println("BUTTON PRESSED");
      Serial.println("FALL ALERT CANCELLED");
      Serial.println("BUZZER OFF");
      Serial.println("================================");

      // Small debounce delay
      delay(50);
    }
  }
}

// =====================================================
// SOS BUTTON (D5 / GPIO5)
// =====================================================

void checkSOSButton()
{
  bool buttonPressed = (digitalRead(SOS_BUTTON_PIN) == LOW);

  // Trigger only once when the button changes from released to pressed.
  if (buttonPressed && !sosButtonPreviouslyPressed &&
      millis() - sosLastDebounceTime >= SOS_DEBOUNCE_MS)
  {
    delay(20); // small debounce check

    if (digitalRead(SOS_BUTTON_PIN) == LOW && !sosActive)
    {
      sosActive = true;

       if (!sosSmsSent)
       {
         sosSmsSent = sendSMS(makeAlertMessage("SOS BUTTON PRESSED"));
       }
// Save current coordinates if GPS already has a valid fix.
      if (gpsValid)
      {
        sosLatitude = latitude;
        sosLongitude = longitude;
        sosGpsValid = true;
      }

      Serial.println("================================");
      Serial.println("!!! SOS BUTTON PRESSED !!!");
      Serial.println("SOS ALERT ACTIVE ON DASHBOARD");

      if (sosGpsValid)
      {
        Serial.print("SOS Latitude: ");
        Serial.println(sosLatitude, 6);
        Serial.print("SOS Longitude: ");
        Serial.println(sosLongitude, 6);
      }
      else
      {
        Serial.println("GPS fix not available yet; waiting for valid coordinates.");
      }

      Serial.println("================================");
    }

    sosLastDebounceTime = millis();
  }

  sosButtonPreviouslyPressed = buttonPressed;
}

// =====================================================
// UPDATE GPS
// =====================================================

void updateGPS()
{
  while (GPS_Serial.available())
  {
    gps.encode(GPS_Serial.read());
  }

  if (gps.location.isValid())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    gpsValid = true;

    // If SOS was pressed before GPS acquired a fix, attach the first valid fix.
    if (sosActive && !sosGpsValid)
    {
      sosLatitude = latitude;
      sosLongitude = longitude;
      sosGpsValid = true;

      Serial.println("GPS coordinates added to active SOS alert.");
    }
  }
}

// =====================================================
// OLED DISPLAY
// =====================================================

void updateOLED()
{
  oled.clearBuffer();

  oled.setFont(u8g2_font_6x10_tf);

  if (sosActive)
  {
    oled.drawStr(20, 15, "!!! SOS ALERT !!!");
    oled.drawStr(0, 32, "HELP REQUESTED");

    if (sosGpsValid)
    {
      oled.setCursor(0, 48);
      oled.print("LAT:");
      oled.print(sosLatitude, 4);
      oled.setCursor(0, 62);
      oled.print("LON:");
      oled.print(sosLongitude, 4);
    }
    else
    {
      oled.drawStr(0, 52, "Waiting for GPS...");
    }
  }
  else if (fallDetected)
  {
    oled.drawStr(20, 15, "!!! ALERT !!!");
    oled.drawStr(40, 32, "FALL");
    oled.drawStr(35, 48, "DETECTED");

    if (gpsValid)
    {
      oled.setCursor(0, 63);
      oled.print("GPS OK");
    }
    else
    {
      oled.setCursor(0, 63);
      oled.print("GPS SEARCH");
    }
  }
  else
  {
    oled.drawStr(25, 12, "SMART WEARABLE");

    oled.drawStr(45, 28, "NORMAL");

    if (gpsValid)
    {
      oled.drawStr(0, 43, "GPS: OK");

      oled.setCursor(0, 58);
      oled.print("LAT:");
      oled.print(latitude, 4);
    }
    else
    {
      oled.drawStr(0, 43, "GPS: SEARCHING");

      oled.drawStr(0, 58, "Waiting...");
    }
  }

  oled.sendBuffer();
}

// =====================================================
// WEB DASHBOARD
// =====================================================

void handleRoot()
{
  String html = "";

  html += "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";

  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";

  html += "<meta http-equiv='refresh' content='2'>";

  html += "<title>Smart Wearable</title>";

  html += "<style>";

  html += "body{";
  html += "font-family:Arial;";
  html += "text-align:center;";
  html += "background:#f2f2f2;";
  html += "padding:20px;";
  html += "}";

  html += ".box{";
  html += "background:white;";
  html += "padding:20px;";
  html += "margin:auto;";
  html += "max-width:500px;";
  html += "border-radius:15px;";
  html += "box-shadow:0 0 10px #aaa;";
  html += "}";

  html += ".normal{";
  html += "color:green;";
  html += "font-size:28px;";
  html += "}";

  html += ".alert{";
  html += "color:red;";
  html += "font-size:30px;";
  html += "font-weight:bold;";
  html += "}";

  html += "a{";
  html += "display:inline-block;";
  html += "margin-top:15px;";
  html += "padding:12px 20px;";
  html += "background:#2196F3;";
  html += "color:white;";
  html += "text-decoration:none;";
  html += "border-radius:8px;";
  html += "}";

  html += "</style>";

  html += "</head>";

  html += "<body>";

  html += "<div class='box'>";

  html += "<h1>SMART WEARABLE</h1>";

  // ---------------------------------------------------
  // Status
  // ---------------------------------------------------

  if (sosActive)
  {
    html += "<div class='alert'>!!! SOS EMERGENCY !!!</div>";
    html += "<p><b>Emergency message:</b> HELP NEEDED! Please assist immediately.</p>";

    if (sosGpsValid)
    {
      html += "<p><b>SOS Latitude:</b><br>";
      html += String(sosLatitude, 6);
      html += "</p>";
      html += "<p><b>SOS Longitude:</b><br>";
      html += String(sosLongitude, 6);
      html += "</p>";
      html += "<a target='_blank' href='https://www.google.com/maps?q=";
      html += String(sosLatitude, 6);
      html += ",";
      html += String(sosLongitude, 6);
      html += "'>OPEN SOS LOCATION IN GOOGLE MAPS</a>";
    }
    else
    {
      html += "<p>Waiting for a valid GPS fix to attach coordinates...</p>";
    }
  }
  else if (fallDetected)
  {
    html += "<div class='alert'>!!! FALL DETECTED !!!</div>";
  }
  else
  {
    html += "<div class='normal'>NORMAL</div>";
  }

  // ---------------------------------------------------
  // GPS
  // ---------------------------------------------------

  html += "<h2>GPS LOCATION</h2>";

  if (gpsValid)
  {
    html += "<p><b>Latitude:</b><br>";
    html += String(latitude, 6);
    html += "</p>";

    html += "<p><b>Longitude:</b><br>";
    html += String(longitude, 6);
    html += "</p>";

    html += "<a target='_blank' href='https://www.google.com/maps?q=";
    html += String(latitude, 6);
    html += ",";
    html += String(longitude, 6);
    html += "'>";

    html += "OPEN LOCATION IN GOOGLE MAPS";

    html += "</a>";
  }
  else
  {
    html += "<p>GPS searching for satellites...</p>";
  }

  // ---------------------------------------------------
  // MPU DATA
  // ---------------------------------------------------

  html += "<h2>ACCELERATION</h2>";

  html += "<p>X: ";
  html += String(accelX, 2);
  html += " m/s²</p>";

  html += "<p>Y: ";
  html += String(accelY, 2);
  html += " m/s²</p>";

  html += "<p>Z: ";
  html += String(accelZ, 2);
  html += " m/s²</p>";

  html += "<p><b>Total: ";
  html += String(totalAcceleration, 2);
  html += " m/s²</b></p>";

  html += "</div>";

  html += "</body>";
  html += "</html>";

  server.send(200, "text/html", html);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  // ===================================================
  // BUZZER + BUTTON
  // ===================================================

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);

  // ===================================================
  // I2C
  // ===================================================

  Wire.begin(OLED_SDA, OLED_SCL);

  Wire.setClock(400000);

  delay(100);

  // ===================================================
  // OLED
  // ===================================================

  oled.begin();

  oled.clearBuffer();

  oled.setFont(u8g2_font_6x10_tf);

  oled.drawStr(15, 30, "SMART WEARABLE");

  oled.drawStr(25, 45, "Starting...");

  oled.sendBuffer();

  // ===================================================
  // MPU6050
  // ===================================================

  mpuConnected = initializeMPU6050();

  if (!mpuConnected)
  {
    Serial.println();
    Serial.println("MPU6050 INITIALIZATION FAILED");
    Serial.println();
  }

  // ===================================================
  // GPS
  // ===================================================

  GPS_Serial.begin(
    9600,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  Serial.println("GPS initialized.");

  // A7670C UART (pins above are examples; verify your board first)
  GSM_Serial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(300);
  Serial.println("A7670C UART initialized.");

  // ===================================================
  // WiFi
  // ===================================================

  Serial.println("Connecting to WiFi...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);

    Serial.print(".");
  }

  Serial.println();

  Serial.println("WiFi connected!");

  Serial.print("Dashboard IP: ");

  Serial.println(WiFi.localIP());

  // ===================================================
  // WEB SERVER
  // ===================================================

  server.on("/", handleRoot);

  server.begin();

  Serial.println("Web server started.");

  Serial.println();
  Serial.println("================================");
  Serial.println("SMART WEARABLE READY");
  Serial.println("================================");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // GPS
  // ---------------------------------------------------

  updateGPS();

  // ---------------------------------------------------
  // Fall detection
  // ---------------------------------------------------

  checkFall();

  // ---------------------------------------------------
  // Check button
  // ---------------------------------------------------

  checkButton();
  checkSOSButton();

  // ---------------------------------------------------
  // OLED
  // ---------------------------------------------------

  updateOLED();

  // ---------------------------------------------------
  // Web server
  // ---------------------------------------------------

  server.handleClient();

  delay(100);
}
