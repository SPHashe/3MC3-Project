#include <WiFi.h>
#include <WebServer.h>

// Access Point credentials
const char* ssid = "ESP32_Arm";
const char* password = "robotarm";

// Create web server on port 80
WebServer server(80);

// Define LED pin (GPIO 2 for onboard LED)
const int XPositive = 4;
const int XNegative = 18;
const int YPositive = 19;
const int YNegative = 21;
const int ZPositive = 22;
const int ZNegative = 23;
bool ledState = LOW;

// HTML content to be served
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 LED Control</title>
  <style>
    body { font-family: Arial; text-align: center; margin-top: 50px; }
    .button {
      padding: 16px 36px; margin: 20px; font-size: 20px;
      border: none; border-radius: 10px; cursor: pointer;
    }
    .on { background-color: #4CAF50; color: white; }
    .on:hover { background-color: #45a049; }
    .off { background-color: #f44336; color: white; }
    .off:hover { background-color: #da190b; }
  </style>
</head>
<body>
  <h1>ESP32 LED Control</h1>
  <p><strong>Status:</strong> <span id="ledStatus">%STATE%</span></p>
  <!-- Row 1 -->
  <div class="button-row">
    <button class="button on" onclick="location.href='/X/Positive'">XPositive</button>
    <button class="button off" onclick="location.href='/X/Negative'">XNegative</button>
  </div>

  <!-- Row 2 -->
  <div class="button-row">
    <button class="button on" onclick="location.href='/Y/Positive'">YPositive</button>
    <button class="button off" onclick="location.href='/Y/Negative'">YNegative</button>
  </div>

  <!-- Row 3 -->
  <div class="button-row">
    <button class="button on" onclick="location.href='/Z/Positive'">ZPositive</button>
    <button class="button off" onclick="location.href='/Z/Negative'">ZNegative</button>
  </div>
</body>
</html>
)rawliteral";

void handleRoot() {
  String html = htmlPage;
  html.replace("%STATE%", ledState ? "ON" : "OFF");
  server.send(200, "text/html", html);
}

void XPositiveON() {
  digitalWrite(XPositive, HIGH);
  digitalWrite(XNegative, LOW);
  digitalWrite(YPositive, LOW);
  digitalWrite(YNegative, LOW);
  digitalWrite(ZPositive, LOW);
  digitalWrite(ZNegative, LOW);
  ledState = true;
  handleRoot();
}

void XNegativeON() {
  digitalWrite(XPositive, LOW);
  digitalWrite(XNegative, HIGH);
  digitalWrite(YPositive, LOW);
  digitalWrite(YNegative, LOW);
  digitalWrite(ZPositive, LOW);
  digitalWrite(ZNegative, LOW);
  ledState = false;
  handleRoot();
}

void YPositiveON(){
  digitalWrite(XPositive, LOW);
  digitalWrite(XNegative, LOW);
  digitalWrite(YPositive, HIGH);
  digitalWrite(YNegative, LOW);
  digitalWrite(ZPositive, LOW);
  digitalWrite(ZNegative, LOW);
  handleRoot();
}

void YNegativeON(){
  digitalWrite(XPositive, LOW);
  digitalWrite(XNegative, LOW);
  digitalWrite(YPositive, LOW);
  digitalWrite(YNegative, HIGH);
  digitalWrite(ZPositive, LOW);
  digitalWrite(ZNegative, LOW);
  handleRoot();
}

void ZPositiveON(){
  digitalWrite(XPositive, LOW);
  digitalWrite(XNegative, LOW);
  digitalWrite(YPositive, LOW);
  digitalWrite(YNegative, LOW);
  digitalWrite(ZPositive, HIGH);
  digitalWrite(ZNegative, LOW);
  handleRoot();
}

void ZNegativeON(){
  digitalWrite(XPositive, LOW);
  digitalWrite(XNegative, LOW);
  digitalWrite(YPositive, LOW);
  digitalWrite(YNegative, LOW);
  digitalWrite(ZPositive, LOW);
  digitalWrite(ZNegative, HIGH);
  handleRoot();
}

void setup() {
  Serial.begin(115200);
  pinMode(XPositive, OUTPUT);
  pinMode(XNegative, OUTPUT);
  pinMode(YPositive, OUTPUT);
  pinMode(YNegative, OUTPUT);
  pinMode(ZPositive, OUTPUT);
  pinMode(ZNegative, OUTPUT);
  digitalWrite(XPositive, ledState);

  // Start Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Define routes
  server.on("/", handleRoot);
  server.on("/X/Positive", XPositiveON);
  server.on("/X/Negative", XNegativeON);
  server.on("/Y/Positive", YPositiveON);
  server.on("/Y/Negative", YNegativeON);
  server.on("/Z/Positive", ZPositiveON);
  server.on("/Z/Negative", ZNegativeON);
  server.begin();
}

void loop() {
  server.handleClient();
}
