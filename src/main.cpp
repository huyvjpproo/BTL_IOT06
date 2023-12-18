#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

const char *ssid = "huy";
const char *password = "12345678";

WiFiServer server(80);

String header;

// Đèn sân
String outputState_led_denSan = "off";
int mode_densan = 0; // 0: Manual, 1: Auto
const int led_anhSang = 2;
const int lightSensorPin_denSan = 34;

// Đèn phòng tắm
String outputState_led_phongTam = "off";
int mode_denPhongTam = 0; // 0: Manual, 1: Auto
const int pirPin = 18;
const int led_phongTam = 19;

// Bật tắt quạt
#define DHTPIN 22     // Chân kết nối cảm biến DHT11
#define DHTTYPE DHT11 // Loại cảm biến DHT11
#define LEDPIN_2 23   // Chân kết nối đèn LED
DHT dht(DHTPIN, DHTTYPE);

int mode_nhetDoDoAm = 0; // 0: Manual, 1: Auto
String outputState_led_nhietDoDoAm = "off";

// Dàn phơi
const int sensor_mua = 35; // Chân kết nối cảm biến mưa
const int led_mua = 25;    // Chân kết nối đèn
int mode_mua = 0;          // 0: Manual, 1: Auto
String outputState_led_mua = "off";
int check_mua = 0;

// Đèn ngủ
#define sensor_amThanh 5
#define led_amThanh 33
int mode_amThanh = 0; // 0: Manual, 1: Auto
String outputState_led_amThanh = "off";
boolean val = 1;       // Mặc định không phát hiện âm thanh
boolean ledStatus = 0; // Mặc định không bật đèn
int check = -1;

// Báo cháy
int LED_Gas = 26;
int sensor_Gas = 21;
int mode_gas = 0; // 0: Manual, 1: Auto
String outputState_led_gas = "off";
boolean alertGasDetected = false;
boolean userAcknowledged = false;

unsigned long currentTime = millis();S
unsigned long previousTime = 0;
const long timeoutTime = 2000;

void setup()
{
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {

    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  pinMode(led_anhSang, OUTPUT);
  pinMode(led_phongTam, OUTPUT);
  pinMode(LEDPIN_2, OUTPUT);
  pinMode(led_mua, OUTPUT);
  pinMode(led_amThanh, OUTPUT);
  pinMode(LED_Gas, OUTPUT);
  analogReadResolution(12);
  digitalWrite(led_anhSang, LOW);
  digitalWrite(led_phongTam, LOW);
  digitalWrite(LEDPIN_2, LOW);
  digitalWrite(led_mua, LOW);
  digitalWrite(led_amThanh, LOW);
  digitalWrite(LED_Gas, LOW);
  dht.begin();
}

// Hàm điều khiển đèn sân
void control_led_denSan()
{
  if (mode_densan == 1)
  {
    int lightValue = analogRead(lightSensorPin_denSan);
    Serial.println(lightValue);
    if (lightValue > 2048)
    {
      Serial.println("Low light, turning GPIO 2 on");
      digitalWrite(led_anhSang, HIGH);
    }
    else
    {
      Serial.println("Sufficient light, turning GPIO 2 off");
      digitalWrite(led_anhSang, LOW);
    }
  }
}

// Hàm điều khiển đèn phòng tắm
void control_led_phongTam()
{
  if (mode_denPhongTam == 1)
  {
    int motionDetected = digitalRead(pirPin);
    if (motionDetected == HIGH)
    {
      Serial.println("Motion detected, turning LED in bathroom on");
      digitalWrite(led_phongTam, HIGH);
    }
    else
    {
      Serial.println("No motion, turning LED in bathroom off");
      digitalWrite(led_phongTam, LOW);
    }
  }
}
// hàm điểu khiểu DHT11
void control_led_nhietDoDoAM()
{
  if (mode_nhetDoDoAm == 1)
  {
    float temperature = dht.readTemperature(); // Đọc nhiệt độ

    // Kiểm tra nếu đọc dữ liệu thành công
    if (!isnan(temperature))
    {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");

      if (temperature > 30)
      {
        digitalWrite(LEDPIN_2, HIGH); // Bật đèn
        Serial.println("High temperature detected. Light on.");
      }
      else
      {
        digitalWrite(LEDPIN_2, LOW); // Tắt đèn
        Serial.println("Normal temperature. Light off.");
      }
    }
    else
    {
      Serial.println("Failed to read from DHT sensor!");
    }
  }
}
// hàm điểu khiển mưa
void control_led_mua()
{
  if (mode_mua == 1)
  {
    int sensorValue = analogRead(sensor_mua);
    Serial.println(sensorValue);
    if (sensorValue < 3000 && check_mua == 0)
    {
      check_mua = 1;
      digitalWrite(led_mua, HIGH); // Bật đèn khi có mưa
      Serial.println("Trời đang mưa!");
      delay(3000);
      digitalWrite(led_mua, LOW);
    }
    else if (sensorValue > 4000 && check_mua == 1)
    {

      check_mua = 0;
      digitalWrite(led_mua, HIGH); // Bật đèn khi có mưa
      delay(3000);
      digitalWrite(led_mua, LOW); // Tắt đèn khi không có mưa
      Serial.println("Trời không mưa!");
    }
  }
}
// hàm điều khiển âm thanh

void control_led_amThanh()
{
  if (mode_amThanh == 1)
  {
    val = digitalRead(sensor_amThanh);
    Serial.println(val);
    check++;
    if (val == 0 && check % 2 == 0)
      ledStatus = 1;
    if (val == 0 && check % 2 == 1)
      ledStatus = 0;
    digitalWrite(led_amThanh, ledStatus);
  }
}
// hàm điểu khiển gas
void control_led_gas()
{

  int LPG_detected = digitalRead(sensor_Gas);
  Serial.println(LPG_detected);

  if (LPG_detected == 1)
  {

    Serial.println("LPG detected...! take action immediately.");
    digitalWrite(LED_Gas, HIGH);
    delay(500);
    digitalWrite(LED_Gas, LOW);
    delay(500);
  }
  else
  {

    Serial.println("No detected, stay cool");
    digitalWrite(LED_Gas, LOW);
  }
}
void loop()
{
  control_led_denSan();
  control_led_phongTam();
  control_led_nhietDoDoAM();
  control_led_mua();
  control_led_amThanh();
  control_led_gas();
  WiFiClient client = server.available();

  if (client)
  {
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected() && currentTime - previousTime <= timeoutTime)
    {
      currentTime = millis();
      if (client.available())
      {
        char c = client.read();
        Serial.write(c);
        header += c;
        if (c == '\n')
        {
          if (currentLine.length() == 0)
          {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if (mode_densan == 0)
            {
              if (header.indexOf("GET /2/on") >= 0)
              {
                Serial.println("GPIO 2 on");
                outputState_led_denSan = "on";
                digitalWrite(led_anhSang, HIGH);
              }
              else if (header.indexOf("GET /2/off") >= 0)
              {
                Serial.println("GPIO 2 off");
                outputState_led_denSan = "off";
                digitalWrite(led_anhSang, LOW);
              }
            }

            if (mode_denPhongTam == 0)
            {
              if (header.indexOf("GET /19/on") >= 0)
              {
                Serial.println("LED in bathroom on");
                outputState_led_phongTam = "on";
                digitalWrite(led_phongTam, HIGH);
              }
              else if (header.indexOf("GET /19/off") >= 0)
              {
                Serial.println("LED in bathroom off");
                outputState_led_phongTam = "off";
                digitalWrite(led_phongTam, LOW);
              }
            }
            if (mode_nhetDoDoAm == 0)
            {
              if (header.indexOf("GET /23/on") >= 0)
              {
                Serial.println("GPIO 23 on");
                outputState_led_nhietDoDoAm = "on";
                digitalWrite(LEDPIN_2, HIGH);
              }
              else if (header.indexOf("GET /23/off") >= 0)
              {
                Serial.println("GPIO 23 off");
                outputState_led_nhietDoDoAm = "off";
                digitalWrite(LEDPIN_2, LOW);
              }
            }
            if (mode_mua == 0)
            {
              if (header.indexOf("GET /25/on") >= 0)
              {
                Serial.println("GPIO 25 on");
                outputState_led_mua = "on";
                digitalWrite(led_mua, HIGH);
                delay(3000);
                digitalWrite(led_mua, LOW);
              }
              else if (header.indexOf("GET /25/off") >= 0)
              {
                Serial.println("GPIO 25 off");
                outputState_led_mua = "off";
                digitalWrite(led_mua, HIGH);
                delay(3000);
                digitalWrite(led_mua, LOW);
              }
            }

            if (mode_amThanh == 0)
            {
              if (header.indexOf("GET /33/on") >= 0)
              {
                Serial.println("GPIO 33 on");
                outputState_led_amThanh = "on";
                digitalWrite(led_amThanh, HIGH);
              }
              else if (header.indexOf("GET /33/off") >= 0)
              {
                Serial.println("GPIO 33 off");
                outputState_led_amThanh = "off";
                digitalWrite(led_amThanh, LOW);
              }
            }

            if (header.indexOf("GET /auto/denSan/on") >= 0)
            {
              Serial.println("Auto mode for Den San on");
              mode_densan = 1;
            }
            else if (header.indexOf("GET /auto/denSan/off") >= 0)
            {
              Serial.println("Auto mode for Den San off");
              mode_densan = 0;
            }

            if (header.indexOf("GET /auto/denPhongTam/on") >= 0)
            {
              Serial.println("Auto mode for Den Phong Tam on");
              mode_denPhongTam = 1;
            }
            else if (header.indexOf("GET /auto/denPhongTam/off") >= 0)
            {
              Serial.println("Auto mode for Den Phong Tam off");
              mode_denPhongTam = 0;
            }
            if (header.indexOf("GET /auto/DHT11/on") >= 0)
            {
              Serial.println("Auto mode for DHT11 on");
              mode_nhetDoDoAm = 1;
            }
            else if (header.indexOf("GET /auto/DHT11/off") >= 0)
            {
              Serial.println("Auto mode for DHT11 off");
              mode_nhetDoDoAm = 0;
            }
            if (header.indexOf("GET /auto/danPhoi/on") >= 0)
            {
              Serial.println("Auto mode for danPhoi on");
              mode_mua = 1;
            }
            else if (header.indexOf("GET /auto/danPhoi/off") >= 0)
            {
              Serial.println("Auto mode for danPhoi off");
              mode_mua = 0;
            }
            if (header.indexOf("GET /auto/amThanh/on") >= 0)
            {
              Serial.println("Auto mode for Âm thanh on");
              mode_amThanh = 1;
            }
            else if (header.indexOf("GET /auto/amThanh/off") >= 0)
            {
              Serial.println("Auto mode for Âm thanh off");
              mode_amThanh = 0;
            }

            if (header.indexOf("GET /auto/gas/on") >= 0)
            {
              Serial.println("Auto mode for gas on");
              mode_gas = 1;
            }
            else if (header.indexOf("GET /auto/gas/off") >= 0)
            {
              Serial.println("Auto mode for gas off");
              mode_gas = 0;
            }

            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta charset=\"UTF-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}");
            client.println(".container { display: flex; justify-content: space-around; }</style></head>");

            client.println("<body><h1>MY HOME</h1>");

            client.println("<div class=\"container\">");
            // Khung cho Đèn sân
            client.println("<div>");
            client.println("<p>Đèn sân</p>");
            client.println("<p>Chế độ: " + String(mode_densan == 1 ? "Auto" : "Manual") + "</p>");

            // Nút điều khiển Đèn sân
            if (mode_densan == 0)
            {
              if (outputState_led_denSan == "off")
              {
                client.println("<p><a href=\"/2/on\"><button class=\"button\">ON</button></a></p>");
              }
              else
              {
                client.println("<p><a href=\"/2/off\"><button class=\"button button2\">OFF</button></a></p>");
              }
            }

            // Nút chuyển đổi chế độ Đèn sân
            client.println("<p><a href=\"/auto/denSan/on\"><button class=\"button\">Auto</button></a></p>");
            client.println("<p><a href=\"/auto/denSan/off\"><button class=\"button button2\">Manual</button></a></p>");
            client.println("</div>");

            // Khung cho Đèn phòng tắm
            client.println("<div>");
            client.println("<p>Đèn phòng tắm</p>");
            client.println("<p>Chế độ: " + String(mode_denPhongTam == 1 ? "Auto" : "Manual") + "</p>");

            // Nút điều khiển Đèn phòng tắm
            if (mode_denPhongTam == 0)
            {
              if (outputState_led_phongTam == "off")
              {
                client.println("<p><a href=\"/19/on\"><button class=\"button\">On</button></a></p>");
              }
              else
              {
                client.println("<p><a href=\"/19/off\"><button class=\"button button2\">OFF</button></a></p>");
              }
            }

            // Nút chuyển đổi chế độ Đèn phòng tắm
            client.println("<p><a href=\"/auto/denPhongTam/on\"><button class=\"button\">Auto</button></a></p>");
            client.println("<p><a href=\"/auto/denPhongTam/off\"><button class=\"button button2\">Manual</button></a></p>");
            client.println("</div>");

            // Khung cho DHT11
            client.println("<div>");
            client.println("<p>Quạt</p>");
            client.println("<p>Chế độ: " + String(mode_nhetDoDoAm == 1 ? "Auto" : "Manual") + "</p>");

            // Nút điều khiển DT11
            if (mode_nhetDoDoAm == 0)
            {
              if (outputState_led_nhietDoDoAm == "off")
              {
                client.println("<p><a href=\"/23/on\"><button class=\"button\">ON</button></a></p>");
              }
              else
              {
                client.println("<p><a href=\"/23/off\"><button class=\"button button2\">OFF</button></a></p>");
              }
            }

            // Nút chuyển đổi chế độ DT11
            client.println("<p><a href=\"/auto/DHT11/on\"><button class=\"button\">Auto</button></a></p>");
            client.println("<p><a href=\"/auto/DHT11/off\"><button class=\"button button2\">Manual</button></a></p>");
            client.println("</div>");

            // Khung cho giàn phơi
            client.println("<div>");
            client.println("<p>Giàn phơi</p>");
            client.println("<p>Chế độ: " + String(mode_mua == 1 ? "Auto" : "Manual") + "</p>");

            // Nút điều khiển giàn phơi
            if (mode_mua == 0)
            {
              if (outputState_led_mua == "off")
              {
                client.println("<p><a href=\"/25/on\"><button class=\"button\">ON</button></a></p>");
              }
              else
              {
                client.println("<p><a href=\"/25/off\"><button class=\"button button2\">OFF</button></a></p>");
              }
            }

            // Nút chuyển đổi chế độ Giàn phơi
            client.println("<p><a href=\"/auto/danPhoi/on\"><button class=\"button\">Auto</button></a></p>");
            client.println("<p><a href=\"/auto/danPhoi/off\"><button class=\"button button2\">Manual</button></a></p>");
            client.println("</div>");

            // Khung cho âm thanh
            client.println("<div>");
            client.println("<p>Đèn phòng ngủ</p>");
            client.println("<p>Chế độ: " + String(mode_amThanh == 1 ? "Auto" : "Manual") + "</p>");

            // Nút điều khiển âm thanh
            if (mode_amThanh == 0)
            {
              if (outputState_led_amThanh == "off")
              {
                client.println("<p><a href=\"/33/on\"><button class=\"button\">ON</button></a></p>");
              }
              else
              {
                client.println("<p><a href=\"/33/off\"><button class=\"button button2\">OFF</button></a></p>");
              }
            }

            // Nút chuyển đổi chế độ âm thanh
            client.println("<p><a href=\"/auto/amThanh/on\"><button class=\"button\">Auto</button></a></p>");
            client.println("<p><a href=\"/auto/amThanh/off\"><button class=\"button button2\">Manual</button></a></p>");
            client.println("</div>");

            client.println("</div>"); // Kết thúc container
            client.println("</body></html>");

            client.println();
            break;
          }
          else
          {
            currentLine = "";
          }
        }
        else if (c != '\r')
        {
          currentLine += c;
        }
      }
    }
    header = "";
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}