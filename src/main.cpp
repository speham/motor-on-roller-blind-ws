#include "FS.h"
#include "NidayandHelper.h"
#include "index_html.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Stepper_28BYJ_48.h>

//#include <CheapStepper.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <WebSocketsServer.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <string>

//--------------- CHANGE PARAMETERS ------------------
// Configure Default Settings for Access Point logon
// Name of access point
String APid = "Rollerblind";
// Password for access point
String APpw = "123456789";

//----------------------------------------------------

// Version number for checking if there are new code releases and notifying the
// user
String version = "1.4.2";

NidayandHelper helper = NidayandHelper();

// Fixed settings for WIFI
WiFiClient espClient;
// Setup WIFI Manager
WiFiManager wifiManager;
// MQTT client
PubSubClient psclient(espClient);
// WIFI config: MQTT server config (optional)
char mqtt_server[40];
// WIFI config: MQTT port config (optional)
char mqtt_port[6] = "1883";
// WIFI config: MQTT server username (optional)
char mqtt_uid[40];
// WIFI config: MQTT server password (optional)
char mqtt_pwd[40];

// MQTT topic for sending messages
String outputTopic;
// MQTT topic for listening
String inputTopic1;
boolean mqttActive = true;
// WIFI config: Bonjour name of device
char config_name[40];
// WIFI config: Detault rotation is CCW
char config_rotation[40] = "false";
unsigned long lastBlink = 0;
int state = 0;
long lastPublish = 0;

// Action manual/auto
String action1;
String msg;

int set1;
int pos1;

bool isMoving;  
long lastMsgPosSend = 0;

// Direction of blind (1 = down, 0 = stop, -1 = up)
int path1 = 0;
// The set position 0-100% by the client
int setPos1 = 0;

long currentPosition1 = 0;
long maxPosition1 = 100000;

boolean loadDataSuccess = false;
// If true will store positions to SPIFFS
boolean saveItNow = false;
// Used for WIFI Manager callback to save parameters
bool shouldSaveConfig = false;
// To enable actions first time the loop is run
boolean initLoop = true;
// Turns counter clockwise to lower the curtain
boolean ccw = true;

#define MotorStepPin    D2 
#define MotorDirPin     D1 
#define MotorEnablePin  D5 
#include <AccelStepper.h>
AccelStepper Stepper1(AccelStepper::DRIVER, MotorStepPin, MotorDirPin);

// TCP server at port 80 will respond to HTTP requests
ESP8266WebServer server(80);
// WebSockets will respond on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

bool loadConfig() {
  if (!helper.loadconfig()) {
    return false;
  }
  JsonObject &root = helper.getconfig();

  root.printTo(Serial);
  Serial.println();

  // Store variables locally
  currentPosition1 = root["currentPosition1"]; // 400
  Stepper1.setCurrentPosition(currentPosition1);
  maxPosition1 = root["maxPosition1"];         // 20000
  strcpy(config_name, root["config_name"]);
  strcpy(mqtt_server, root["mqtt_server"]);
  strcpy(mqtt_port, root["mqtt_port"]);
  strcpy(mqtt_uid, root["mqtt_uid"]);
  strcpy(mqtt_pwd, root["mqtt_pwd"]);
  strcpy(config_rotation, root["config_rotation"]);
  return true;
}

/**
   Save configuration data to a JSON file
   on SPIFFS
*/
bool saveConfig() {
  const size_t capacity = JSON_OBJECT_SIZE(12);
  DynamicJsonBuffer jsonBuffer(capacity);
  JsonObject &json = jsonBuffer.createObject();
  json["currentPosition1"] = currentPosition1;
  json["maxPosition1"] = maxPosition1;
  json["config_name"] = config_name;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_uid"] = mqtt_uid;
  json["mqtt_pwd"] = mqtt_pwd;
  json["config_rotation"] = config_rotation;

  return helper.saveconfig(json);
}

/*
   Connect to MQTT server and publish a message on the bus.
   Finally, close down the connection and radio
*/
void sendmsg(String topic) {
  set1 = (setPos1 * 100) / maxPosition1;
  pos1 = (currentPosition1 * 100) / maxPosition1;

  msg = "{ \"set1\":" + String(set1) + ", \"position1\":" + String(pos1) + " }";
  // Serial.println(msg);
  if (!mqttActive)
    return;

  helper.mqtt_publish(psclient, topic, msg);
  webSocket.broadcastTXT(msg);
}
/****************************************************************************************
 */
void processMsg(String command, String value, int motor_num,
                uint8_t clientnum) {
  /*
     Below are actions based on inbound MQTT payload
  */
  if (command == "start") {
    /*
       Store the current position as the start position
    */
    if (motor_num == 1) {
      currentPosition1 = 0;
      path1 = 0;
      saveItNow = true;
      action1 = "manual";
    }

  } else if (command == "max") {
    /*
       Store the max position of a closed blind
    */
    if (motor_num == 1) {
      maxPosition1 = currentPosition1;
      path1 = 0;
      saveItNow = true;
      action1 = "manual";
    }

  } else if (command == "manual" && value == "0") {
    /*
       Stop
    */

    if (motor_num == 1) {
      path1 = 0;
      saveItNow = true;
      action1 = "manual";
    }

  } else if (command == "manual" && value == "1") {
    /*
       Move down without limit to max position
    */
    if (motor_num == 1) {
      path1 = 1;
      action1 = "manual";
    }

  } else if (command == "manual" && value == "-1") {
    /*
       Move up without limit to top position
    */
    if (motor_num == 1) {
      path1 = -1;
      action1 = "manual";
    }

  } else if (command == "update") {
    // Send position details to client
    sendmsg(outputTopic);
    webSocket.sendTXT(clientnum, msg);

  } else if (command == "ping") {
    // Do nothing
  } else {
    /*
       Any other message will take the blind to a position
       Incoming value = 0-100
       path is now the position
    */

    Serial.println("Received position " + value);
    if (motor_num == 1) {
      path1 = maxPosition1 * value.toInt() / 100;
      setPos1 = path1; // Copy path for responding to updates
      action1 = "auto";

      set1 = (setPos1 * 100) / maxPosition1;
      pos1 = (currentPosition1 * 100) / maxPosition1;

      // Send the instruction to all connected devices
      sendmsg(outputTopic);
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
  case WStype_TEXT:
    Serial.printf("[%u] get Text: %s\n", num, payload);

    String res = (char *)payload;

    StaticJsonBuffer<100> jsonBuffer1;
    JsonObject &root = jsonBuffer1.parseObject(payload);
    if (root.success()) {
      const int motor_id = root["id"];
      String command = root["action"];
      String value = root["value"];
      // Send to common MQTT and websocket function
      processMsg(command, value, motor_id, num);
      break;
    } else {
      Serial.println("parseObject() failed");
    }
  }
}
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String res = "";
  for (int i = 0; i < length; i++) {
    res += String((char)payload[i]);
  }
  String motor_str = topic;
  int motor_id = motor_str.charAt(motor_str.length() - 1) - 0x30;

  if (res == "update" || res == "ping") {
    processMsg(res, "", NULL, NULL);
  } else
    processMsg("auto", res, motor_id, NULL); 
}

/*
   Callback from WIFI Manager for saving configuration
*/
void saveConfigCallback() { shouldSaveConfig = true; }

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleResetSettings() { helper.resetsettings(wifiManager); }

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void motorDisable()
{
  digitalWrite(MotorEnablePin, HIGH);
}

void motorEnable()
{
  digitalWrite(MotorEnablePin, LOW);
}

void motorSetup()
{
  pinMode(MotorEnablePin, OUTPUT);
  motorDisable();
  
  Stepper1.setMaxSpeed(100000);
  Stepper1.setAcceleration(100000);
  Stepper1.setSpeed(100000);
}

void DisableLeds()
{
  pinMode(2, OUTPUT); 
  digitalWrite(2, HIGH);
  pinMode(16, OUTPUT); 
  digitalWrite(16, HIGH);
}

void setup(void) {
  Serial.begin(115200);
  delay(100);
  Serial.print("Starting now\n");

  // Reset the action
  action1 = "";

  // Set MQTT properties
  outputTopic = helper.mqtt_gettopic("out");
  inputTopic1 = helper.mqtt_gettopic("in1");

  // Set the WIFI hostname
  WiFi.hostname(config_name);

  // Define customer parameters for WIFI Manager
  WiFiManagerParameter custom_config_name("Name", "Bonjour name", config_name,
                                          40);
  WiFiManagerParameter custom_rotation("Rotation", "Clockwise rotation",
                                       config_rotation, 40);
  WiFiManagerParameter custom_text(
      "<p><b>Optional MQTT server parameters:</b></p>");
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server", mqtt_server,
                                          40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_uid("uid", "MQTT username", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_pwd("pwd", "MQTT password", mqtt_server, 40);
  WiFiManagerParameter custom_text2(
      "<script>t = document.createElement('div');t2 = "
      "document.createElement('input');t2.setAttribute('type', "
      "'checkbox');t2.setAttribute('id', 'tmpcheck');t2.setAttribute('style', "
      "'width:10%');t2.setAttribute('onclick', "
      "\"if(document.getElementById('Rotation').value == "
      "'false'){document.getElementById('Rotation').value = 'true'} else "
      "{document.getElementById('Rotation').value = 'false'}\");t3 = "
      "document.createElement('label');tn = document.createTextNode('Clockwise "
      "rotation');t3.appendChild(t2);t3.appendChild(tn);t.appendChild(t3);"
      "document.getElementById('Rotation').style.display='none';document."
      "getElementById(\"Rotation\").parentNode.insertBefore(t, "
      "document.getElementById(\"Rotation\"));</script>");

  // reset settings - for testing
  // clean FS, for testing
  // helper.resetsettings(wifiManager);

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  // add all your parameters here
  wifiManager.addParameter(&custom_config_name);
  wifiManager.addParameter(&custom_rotation);
  wifiManager.addParameter(&custom_text);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_uid);
  wifiManager.addParameter(&custom_mqtt_pwd);
  wifiManager.addParameter(&custom_text2);

  wifiManager.autoConnect(APid.c_str(), APpw.c_str());

  // Load config upon start
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  /* Save the config back from WIFI Manager.
      This is only called after configuration
      when in AP mode
  */
  if (shouldSaveConfig) {
    // read updated parameters
    strcpy(config_name, custom_config_name.getValue());
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_uid, custom_mqtt_uid.getValue());
    strcpy(mqtt_pwd, custom_mqtt_pwd.getValue());
    strcpy(config_rotation, custom_rotation.getValue());

    // Save the data
    saveConfig();
  }
  motorSetup();
  /*
     Try to load FS data configuration every time when
     booting up. If loading does not work, set the default
     positions
  */
  Serial.println("Trying to load config");
  loadDataSuccess = loadConfig();
  Serial.println("Config loaded");
  if (!loadDataSuccess) {
    currentPosition1 = 0;
    maxPosition1 = 100000;
    Stepper1.setCurrentPosition(currentPosition1);
  }

  /*
    Setup multi DNS (Bonjour)
    */
 // if (MDNS.begin(config_name)) {
 //   Serial.println("MDNS responder started");
 //   MDNS.addService("http", "tcp", 80);
 //   MDNS.addService("ws", "tcp", 81);
//
 // } else {
 //   Serial.println("Error setting up MDNS responder!");
 //   while (1) {
 //     delay(1000);
 //   }
 // }
  Serial.print("Connect to http://" + String(config_name) +
               ".local or http://");
  Serial.println(WiFi.localIP());

  // Start HTTP server
  server.on("/", handleRoot);
  server.on("/reset", handleResetSettings);
  server.onNotFound(handleNotFound);
  server.begin();

  // Start websocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  /* Setup connection for MQTT and for subscribed
    messages IF a server address has been entered
  */
  if (String(mqtt_server) != "") {
    Serial.println("Registering MQTT server");
    psclient.setServer(mqtt_server, String(mqtt_port).toInt());
    psclient.setCallback(mqttCallback);

  } else {
    mqttActive = false;
    Serial.println("NOTE: No MQTT server address has been registered. Only "
                   "using websockets");
  }

  /* Set rotation direction of the blinds */
  if (String(config_rotation) == "false") {
    ccw = true;
  } else {
    ccw = false;
  }

  // Update webpage
  INDEX_HTML.replace("{VERSION}", "V" + version);
  INDEX_HTML.replace("{NAME}", String(config_name));

  // Setup OTA
  // helper.ota_setup(config_name);
  {
    // Authentication to avoid unauthorized updates
    // ArduinoOTA.setPassword(OTA_PWD);

    ArduinoOTA.setHostname(config_name);

    ArduinoOTA.onStart([]() { Serial.println("Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR)
        Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)
        Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR)
        Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)
        Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)
        Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  }
  ESP.wdtDisable();
  DisableLeds();
}

void loop(void) {
  // OTA client code
  ArduinoOTA.handle();

  // Websocket listner
  webSocket.loop();

  ESP.wdtFeed();

  /**
    Serving the webpage
  */
  server.handleClient();

  // MQTT client
  if (mqttActive) {
    helper.mqtt_reconnect(psclient, mqtt_uid, mqtt_pwd, {inputTopic1.c_str()});
  }

  /**
    Storing positioning data and turns off the power to the coils
  */
  if (saveItNow) {
    saveConfig();
    saveItNow = false;

    /*
      If no action is required by the motor make sure to
      turn off all coils to avoid overheating and less energy
      consumption
    */
    motorDisable();
  }

  /**
    Manage actions. Steering of the blind
  */
  if (action1 == "auto") {
    /*
       Automatically open or close blind
    */
    motorEnable();
    if (currentPosition1 > path1) {
      Stepper1.move(ccw ? -1 : 1);
      isMoving = true;
      currentPosition1 = currentPosition1 - 1;
    } else if (currentPosition1 < path1) {
      Stepper1.move(ccw ? 1 : -1);
      currentPosition1 = currentPosition1 + 1;
      isMoving = true;
    } else {
      isMoving = false;
      path1 = 0;
      action1 = "";
      set1 = (setPos1 * 100) / maxPosition1;
      pos1 = (currentPosition1 * 100) / maxPosition1;
      sendmsg(outputTopic);
      Serial.println("Stopped 1. Reached wanted position");
      saveItNow = true;
    }
    if (isMoving && ((((currentPosition1 * 100) / maxPosition1) > lastMsgPosSend) || (((currentPosition1 * 100) / maxPosition1) < lastMsgPosSend)))
    {
     lastMsgPosSend = ((currentPosition1 * 100) / maxPosition1);
     sendmsg(outputTopic);
    }
    
  } else if (action1 == "manual" && path1 != 0) {
    /*
       Manually running the blind
    */
    motorEnable();
    Stepper1.move(ccw ? path1 : -path1);
    currentPosition1 = currentPosition1 + path1;
  }

  
  while (Stepper1.distanceToGo() != 0)
  {
    Stepper1.run();
    delay(0); // wdt reset 
  }
  
  /*
     After running setup() the motor might still have
     power on some of the coils. This is making sure that
     power is off the first time loop() has been executed
     to avoid heating the stepper motor draining
     unnecessary current
  */
  if (initLoop) {
    initLoop = false;
    motorDisable();
  }
}
