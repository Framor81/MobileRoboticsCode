#include <WebSocketsServer.h>
#include <WiFi.h>

typedef enum { WS_ENABLED, WS_DISABLED } WsState;

typedef enum { LED_ON = LOW, LED_OFF = HIGH } LedState;

//    ▄▄▄  ▀▀█           █             ▀▀█            ▄▄▄▄    ▄             ▄
//  ▄▀   ▀   █     ▄▄▄   █▄▄▄    ▄▄▄     █           █▀   ▀ ▄▄█▄▄   ▄▄▄   ▄▄█▄▄   ▄▄▄
//  █   ▄▄   █    █▀ ▀█  █▀ ▀█  ▀   █    █           ▀█▄▄▄    █    ▀   █    █    █▀  █
//  █    █   █    █   █  █   █  ▄▀▀▀█    █               ▀█   █    ▄▀▀▀█    █    █▀▀▀▀
//   ▀▄▄▄▀   ▀▄▄  ▀█▄█▀  ██▄█▀  ▀▄▄▀█    ▀▄▄         ▀▄▄▄█▀   ▀▄▄  ▀▄▄▀█    ▀▄▄  ▀█▄▄▀

// WebSocket server configuration and state
const int PORT = 8181;
const char* SSID = "Pomona";
WebSocketsServer webSocket = WebSocketsServer(PORT);

// Heartbeat state
WsState wsState = WS_DISABLED;

// LED state
LedState ledState = LED_OFF;

//  ▄    ▄            █           ▄                  ▄▄▄▄▄                  ▀               █
//  █    █ ▄▄▄▄    ▄▄▄█   ▄▄▄   ▄▄█▄▄   ▄▄▄          █   ▀█  ▄▄▄    ▄ ▄▄  ▄▄▄     ▄▄▄    ▄▄▄█
//  █    █ █▀ ▀█  █▀ ▀█  ▀   █    █    █▀  █         █▄▄▄█▀ █▀  █   █▀  ▀   █    █▀ ▀█  █▀ ▀█
//  █    █ █   █  █   █  ▄▀▀▀█    █    █▀▀▀▀         █      █▀▀▀▀   █       █    █   █  █   █
//  ▀▄▄▄▄▀ ██▄█▀  ▀█▄██  ▀▄▄▀█    ▀▄▄  ▀█▄▄▀         █      ▀█▄▄▀   █     ▄▄█▄▄  ▀█▄█▀  ▀█▄██
//         █
//         ▀

unsigned long heartbeatInterval = 1000;
unsigned long heartbeatLastTime = 0;

unsigned long messagingInterval = 600;
unsigned long messagingLastTime = 0;

unsigned long blinkInterval = 250;
unsigned long blinkLastTime = 0;

// ▄     ▄  ▄▄▄▄         ▄    ▄                   █  ▀▀█
// █  █  █ █▀   ▀        █    █  ▄▄▄   ▄ ▄▄    ▄▄▄█    █     ▄▄▄    ▄ ▄▄
// ▀ █▀█ █ ▀█▄▄▄         █▄▄▄▄█ ▀   █  █▀  █  █▀ ▀█    █    █▀  █   █▀  ▀
//  ██ ██▀     ▀█        █    █ ▄▀▀▀█  █   █  █   █    █    █▀▀▀▀   █
//  █   █  ▀▄▄▄█▀        █    █ ▀▄▄▀█  █   █  ▀█▄██    ▀▄▄  ▀█▄▄▀   █

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  IPAddress ip;

  switch (type) {
    case WStype_DISCONNECTED:
      wsState = WS_DISABLED;
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    case WStype_CONNECTED:
      ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      webSocket.sendTXT(num, "Connected");
      break;

    case WStype_TEXT:
      if (strcmp((char*)payload, "heartbeat") == 0) {
        wsState = WS_ENABLED;
        heartbeatLastTime = millis();
      } else {
        Serial.printf("[%u] get Text: %s\n", num, payload);
      }
      break;

    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      //   hexdump(payload, length);
      // webSocket.sendBIN(num, payload, length);
      break;

    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      Serial.printf("[%u] received unhandled WS event type: %d\n", num, type);
      break;
  }
}

//   ▄▄▄▄           ▄
//  █▀   ▀  ▄▄▄   ▄▄█▄▄  ▄   ▄  ▄▄▄▄
//  ▀█▄▄▄  █▀  █    █    █   █  █▀ ▀█
//      ▀█ █▀▀▀▀    █    █   █  █   █
//  ▀▄▄▄█▀ ▀█▄▄▀    ▀▄▄  ▀▄▄▀█  ██▄█▀
//                              █
//                              ▀

void setup() {
  // Start serial connection for debugging
  Serial.begin(115200);

  // Connect to WiFi network
  WiFi.begin(SSID);
  Serial.printf("\n[SETUP] Connecting to '%s'... ", SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("done\n");

  IPAddress ip = WiFi.localIP();
  Serial.printf("[SETUP] IP Address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);

  // Start WebSockets server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.printf("[SETUP] WebSocketsServer started at ws://%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], PORT);

  // Set the builtin LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
}

//  ▄
//  █       ▄▄▄    ▄▄▄   ▄▄▄▄
//  █      █▀ ▀█  █▀ ▀█  █▀ ▀█
//  █      █   █  █   █  █   █
//  █▄▄▄▄▄ ▀█▄█▀  ▀█▄█▀  ██▄█▀
//                       █
//                       ▀

void loop() {
  // Update for processing WebSockets
  webSocket.loop();

  // Update for handling heartbeat
  unsigned long now = millis();
  if (now - heartbeatLastTime > heartbeatInterval) {
    if (wsState == WS_ENABLED) {
      wsState = WS_DISABLED;
      Serial.println("[HEARTBEAT] Heartbeat timeout");
    }
  }

  // Update for sending a message
  now = millis();
  if (now - messagingLastTime > messagingInterval) {
    messagingLastTime = now;

    float number = random(0, 1000) / 1000.0;
    char buffer[32];
    sprintf(buffer, "%f", number);
    webSocket.sendTXT(0, buffer);
  }

  // Update for blinking the builtin LED
  if (wsState == WS_ENABLED) {
    now = millis();
    if (now - blinkLastTime > blinkInterval) {
      blinkLastTime = now;

      ledState = (ledState == LED_ON) ? LED_OFF : LED_ON;
      digitalWrite(LED_BUILTIN, ledState);
    }
  } else {
    digitalWrite(LED_BUILTIN, LED_OFF);
  }
}
