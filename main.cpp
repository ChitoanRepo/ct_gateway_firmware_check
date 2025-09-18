/* Full ESP32 program - feature update per user:
   - delete/edit/add node by ID via webportal
   - add long-press 2s on 4 buttons to toggle 4 relays (SELECT keeps 5s menu)
   - Implement long-press 2s BOOT button for menu.
   - display node info (ID, dimming, relay) on data screen, rotate through nodes
   - relayStatusTask prints node info to Serial
   - persist node IDs to Preferences
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <webportal.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
// ---------------- Hardware pins --------------------
#define BT_BOOT 0
#define BT_UP 35   // UP
#define BT_SEL 32  // SELECT
#define BT_DN 33   // DOWN
#define BT_BACK 25 // BACK
#define BUZ_PIN 13
#define FAN_PIN 15
// ---------------- Relay OUTPUT ---------------------
#define RL1 12
#define RL2 14
#define RL3 27
#define RL4 26
#define total_Slave 10
// ---------------- buzzer LEDC channel --------------
#define BUZ_CHANNEL 0
// ---------------- LoRa pins ------------------------
#define LORA_SS 5
#define LORA_RST 17
#define LORA_DIO 2
// ---------------- watchdog timmer-------------------
#define WDT_TIMEOUT 10
// ---------------- Globals --------------------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
RTC_DS3231 rtc;
Preferences prefs;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer statusServer(8080);
// ---------------- Init Task ---------------------
TaskHandle_t displayTaskHandle;
TaskHandle_t ioTaskHandle;
TaskHandle_t loraTaskHandle;
TaskHandle_t relaytaskhandle;
// ----------- Always-on AP (status) --------------
const char *AP_SSID = "ESP MASTER";
const char *AP_PASS = "12345678";
// ---------------- Struct-------------------------
struct LoRaPacket
{
    int id;     // ID node
    bool data1; // relay state
    int data2;  // PWM / Slider value
};
LoRaPacket packetToSend;

struct LoRaPacketRec
{
    int id;
    float data1;         // temperature
    unsigned long data2; // timestamp or uptime
};
LoRaPacketRec receivedPacket;

struct MasterStation
{
    float sensorValues[6];
    String onTime;
    String offTime;
};
MasterStation master;

struct SlaveStation
{
    int id;
    float temperature;
    int time;
    int sliderValue;
    bool isOn;
    bool isConnected;
    String timeString;
};
SlaveStation slaves[total_Slave];

struct LoRaPacketSend {
  int id;
  float data1;
  long data2;
};
// ---------------- mqtt Server define -------------
const char *mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
bool Lora_status = true;
// ---------------- menu state ---------------------
const char *mainMenu[] = {"Time Setting", "Screen Setting", "Internet Setting", "Exits"};
const int mainMenuCount = 4;
volatile bool inMenu = false;
int menuLevel = 0;
int menuCursor = 0;
int submenuSelected = -1;
// --------------- time setting --------------------
int setHour = 12, setMinute = 0, setSecond = 0;
int cursorPos = 0;
bool editingTime = false;
bool screenFlip = false;
// --------------- relays (local) ------------------
bool relayState[4] = {false, false, false, false};
const int relayPins[4] = {RL1, RL2, RL3, RL4};
// --------------- fan control ---------------------
int fanThreshold = 50;
bool fanState = false;
// --------------- UI timing -----------------------
bool datascreenflag = true;
unsigned long lastActivity = 0;
const unsigned long standbyTimeout = 15000UL; // ms
unsigned long timmerAllert = 0;
// --------------- buzzer (non-blocking) -----------
unsigned long buzzerStart = 0;
unsigned long buzzerDuration = 0;
bool buzzerActive = false;
int buzzerFrequency = 1000;
// --------------- icons ----------------------------
static const unsigned char icon_connected[] = {0x7c, 0x00, 0x82, 0x00, 0x01, 0x01, 0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x10, 0x00};
static const unsigned char icon_disconnect[] = {0x00, 0x01, 0xfc, 0x00, 0xc2, 0x00, 0x21, 0x01, 0x38, 0x00, 0x4c, 0x00, 0x04, 0x00, 0x12, 0x00};
static const unsigned char icon_Thermal[] = {0xc6, 0x01, 0x29, 0x02, 0x29, 0x00, 0x26, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x02, 0xc0, 0x01};
static const unsigned char image_weather_temperature_bits[] = {0x38, 0x00, 0x44, 0x40, 0xd4, 0xa0, 0x54, 0x40, 0xd4, 0x1c, 0x54, 0x06, 0xd4, 0x02, 0x54, 0x02, 0x54, 0x06, 0x92, 0x1c, 0x39, 0x01, 0x75, 0x01, 0x7d, 0x01, 0x39, 0x01, 0x82, 0x00, 0x7c, 0x00};
// ---------------- Node model -------------------
#define MAX_NODES 24
struct Node
{
    int id;
    String label;
    float voltage;
    float current;
    bool relay;
    bool online;
};
Node nodes[MAX_NODES];
int nodeCount = 0;
int nextNodeId = 1;
// ------- forward declarations functions -----------
void startStatusServer();
void setRelayLocal(int idx, bool on);
void initBuzzer();
void buzzerBeep(int frequency, unsigned long duration);
void buzzerUpdate();
void saveToEEPROM(int slaveId, bool isOn, int sliderValue);
void sendLora(int ID, int stateLed, int valvePwm);
void handle_api_status();
void handle_api_relay();
void handle_api_node_add();
void handle_api_node_remove();
void handle_api_node_relay();
void handle_api_node_dim();
void handle_api_node_edit();
void handle_api_config_save();
void saveNodesPrefs();
void loadNodesPrefs();
void addNodeWithId(int id, const String &name, bool relay);
void addNode(const String &name, float voltage = 0.0, float current = 0.0, bool relay = false);
void updateNodeFromLoRa(int id, float voltage, float current, bool relay);
int findNodeIndexById(int id);
bool removeNodeById(int id);
float readInternalTemp();
// ---------------- Init buzzer ---------------------
void initBuzzer()
{
    pinMode(BUZ_PIN, OUTPUT);
    digitalWrite(BUZ_PIN, LOW);
}
// ---------------- Beep buzzer ---------------------
void buzzerBeep(int frequency, unsigned long duration)
{
    // start buzzer non-blocking using ledc
    buzzerFrequency = frequency;
    buzzerDuration = duration;
    buzzerStart = millis();
    buzzerActive = true;
    ledcAttachPin(BUZ_PIN, BUZ_CHANNEL);
    ledcSetup(BUZ_CHANNEL, frequency, 8);
    ledcWriteTone(BUZ_CHANNEL, frequency);
}
// ---------------- Update buzzer -------------------
void buzzerUpdate()
{
    if (buzzerActive && millis() - buzzerStart >= buzzerDuration)
    {
        ledcWriteTone(BUZ_CHANNEL, 0); // stop
        ledcDetachPin(BUZ_PIN);
        buzzerActive = false;
    }
}
// ---------------- Relay status task ---------------
void relayStatusTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        Serial.println("=== Relay Status & Nodes ===");
        for (int i = 0; i < 4; i++)
        {
            Serial.printf("Relay %d: %s |", i + 1, relayState[i] ? "ON" : "OFF");
        }
        Serial.print("\n");
        Serial.println("--- Nodes ---");
        for (int i = 0; i < nodeCount; i++)
        {
            int nid = nodes[i].id;
            bool online = nodes[i].online;
            int slider = 0;
            bool s_on = false;
            bool s_conn = false;
            if (nid > 0 && nid <= total_Slave)
            {
                int si = nid - 1;
                slider = slaves[si].sliderValue;
                s_on = slaves[si].isOn;
                s_conn = slaves[si].isConnected;
            }
            Serial.printf("Node[%d] id=%d label=\"%s\" relay=%s online=%s slider=%d s_on=%s s_conn=%s\n",
                          i, nodes[i].id, nodes[i].label.c_str(),
                          nodes[i].relay ? "ON" : "OFF",
                          nodes[i].online ? "Y" : "N",
                          slider,
                          s_on ? "ON" : "OFF",
                          s_conn ? "Y" : "N");
        }
        Serial.println("============================\n");
        Serial.printf("inmenu:%d|menulevel:%d|menucursor:%d|submenu:%d\n", inMenu, menuLevel, menuCursor, submenuSelected);
        Serial.print("============================\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
// ---------------- Helper: RTC/time ----------------
extern "C" uint8_t temprature_sens_read();
// ---------------- Read internal temp --------------
float readInternalTemp()
{
    return (temprature_sens_read() - 32) / 1.8;
}
// ---------------- Helper: relays ------------------
void setRelayLocal(int idx, bool on)
{
    if (idx < 0 || idx > 3)
        return;
    relayState[idx] = on;
    digitalWrite(relayPins[idx], on ? HIGH : LOW);
    buzzerBeep(2000, 80);
}
// ---------------- Node helpers --------------------
void initNodes()
{
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa initialization failed!");
        Lora_status = false;
    }
    LoRa.setSyncWord(0xF3);
    nodeCount = 0;
    nextNodeId = 1;
    for (int i = 0; i < MAX_NODES; i++)
    {
        nodes[i].id = 0;
        nodes[i].label = "";
        nodes[i].voltage = 0;
        nodes[i].current = 0;
        nodes[i].relay = false;
        nodes[i].online = false;
    }
    for (int i = 0; i < total_Slave; i++)
    {
        slaves[i].id = 0;
        slaves[i].temperature = 0;
        slaves[i].time = 0;
        slaves[i].sliderValue = 0;
        slaves[i].isOn = false;
        slaves[i].isConnected = false;
        slaves[i].timeString = "";
    }
}
// ---------------- Find node by ID -----------------
int findNodeIndexById(int id)
{
    for (int i = 0; i < nodeCount; i++)
        if (nodes[i].id == id)
            return i;
    return -1;
}
// ---------------- Add node with ID ----------------
void addNodeWithId(int id, const String &name, bool relay)
{
    if (nodeCount >= MAX_NODES)
        return;
    // ensure id not duplicate
    if (findNodeIndexById(id) >= 0)
        return;
    nodes[nodeCount].id = id;
    nodes[nodeCount].label = name;
    nodes[nodeCount].voltage = 0;
    nodes[nodeCount].current = 0;
    nodes[nodeCount].relay = relay;
    nodes[nodeCount].online = true;
    nodeCount++;
    if (id >= nextNodeId)
        nextNodeId = id + 1;

    // if maps to slave slot, init slave
    if (id > 0 && id <= total_Slave)
    {
        int si = id - 1;
        slaves[si].id = id;
        // keep existing slider/isOn if previously stored in EEPROM
        slaves[si].isConnected = false;
    }
    saveNodesPrefs();
}
// ---------------- Add node ------------------------
void addNode(const String &name, float voltage, float current, bool relay)
{
    // fallback auto-id behavior: assign nextNodeId
    addNodeWithId(nextNodeId++, name, relay);
}
// ---------------- remove node by ID ---------------
bool removeNodeById(int id)
{
    int idx = findNodeIndexById(id);
    if (idx < 0)
        return false;
    for (int i = idx; i < nodeCount - 1; i++)
        nodes[i] = nodes[i + 1];
    nodeCount--;
    // if mapped to slave, clear slave mapping but keep EEPROM stored data
    if (id > 0 && id <= total_Slave)
    {
        int s = id - 1;
        slaves[s].id = 0;
        slaves[s].isConnected = false;
        // don't erase EEPROM: keep dim/relay state persisted if desired
    }
    saveNodesPrefs();
    return true;
}
// ---------------- Update node from LoRa -----------
void updateNodeFromLoRa(int id, float voltage, float current, bool relay)
{
    int idx = findNodeIndexById(id);
    if (idx >= 0)
    {
        nodes[idx].voltage = voltage;
        nodes[idx].current = current;
        nodes[idx].relay = relay;
        nodes[idx].online = true;
    }
    else
    {
        // if node not present, auto-add with that id and default label
        if (nodeCount < MAX_NODES)
        {
            nodes[nodeCount].id = id;
            nodes[nodeCount].label = "Node " + String(id);
            nodes[nodeCount].voltage = voltage;
            nodes[nodeCount].current = current;
            nodes[nodeCount].relay = relay;
            nodes[nodeCount].online = true;
            nodeCount++;
            if (id >= nextNodeId)
                nextNodeId = id + 1;
            saveNodesPrefs();
        }
    }
}
// ---------------- Save node into prefs ------------
void saveNodesPrefs()
{
    prefs.begin("nodes", false);
    prefs.putInt("count", nodeCount);
    prefs.putInt("nextId", nextNodeId);
    for (int i = 0; i < nodeCount; i++)
    {
        prefs.putInt(("nid" + String(i)).c_str(), nodes[i].id);
        prefs.putString(("nname" + String(i)).c_str(), nodes[i].label);
        prefs.putFloat(("nv" + String(i)).c_str(), nodes[i].voltage);
        prefs.putFloat(("nc" + String(i)).c_str(), nodes[i].current);
        prefs.putInt(("nr" + String(i)).c_str(), nodes[i].relay ? 1 : 0);
    }
    prefs.end();

    // Also save per-slave slider & isOn into EEPROM for persistence
    for (int i = 0; i < total_Slave; i++)
    {
        int addr = i * 8;
        EEPROM.write(addr, slaves[i].isOn ? 1 : 0);
        EEPROM.write(addr + 4, slaves[i].sliderValue & 0xFF);
    }
    EEPROM.commit();
}
// ---------------- Load node Prefs -----------------
void loadNodesPrefs()
{
    prefs.begin("nodes", true);
    int cnt = prefs.getInt("count", 0);
    nextNodeId = prefs.getInt("nextId", 1);
    if (cnt < 0)
        cnt = 0;
    nodeCount = 0;
    for (int i = 0; i < cnt && i < MAX_NODES; i++)
    {
        int nid = prefs.getInt(("nid" + String(i)).c_str(), 0);
        String name = prefs.getString(("nname" + String(i)).c_str(), "");
        float v = prefs.getFloat(("nv" + String(i)).c_str(), 0.0f);
        float c = prefs.getFloat(("nc" + String(i)).c_str(), 0.0f);
        int r = prefs.getInt(("nr" + String(i)).c_str(), 0);
        nodes[nodeCount].id = nid > 0 ? nid : nextNodeId++;
        nodes[nodeCount].label = name.length() ? name : String("Node") + String(nodeCount + 1);
        nodes[nodeCount].voltage = v;
        nodes[nodeCount].current = c;
        nodes[nodeCount].relay = (r != 0);
        nodes[nodeCount].online = true;
        nodeCount++;

        // map into slaves array if within range
        int idAssigned = nodes[nodeCount - 1].id;
        if (idAssigned > 0 && idAssigned <= total_Slave)
        {
            int si = idAssigned - 1;
            slaves[si].id = idAssigned;
            slaves[si].isOn = nodes[nodeCount - 1].relay;
            slaves[si].isConnected = false;
        }
    }
    prefs.end();

    // load saved slider & isOn from EEPROM
    for (int i = 0; i < total_Slave; i++)
    {
        int addr = i * 8;
        uint8_t on = EEPROM.read(addr);
        uint8_t slider = EEPROM.read(addr + 4);
        slaves[i].isOn = (on != 0);
        slaves[i].sliderValue = slider;
    }
}
// ---------------- Status server endpoints ---------
void handle_api_status()
{
    DynamicJsonDocument doc(16384);
    float t = readInternalTemp();
    DateTime now = rtc.now();
    doc["temp"] = t;
    doc["fan"] = fanState ? 1 : 0;
    char timestr[16];
    sprintf(timestr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    doc["time"] = timestr;

    JsonArray r = doc.createNestedArray("relays");
    for (int i = 0; i < 4; i++)
        r.add(relayState[i] ? 1 : 0);

    JsonArray arr = doc.createNestedArray("nodes");
    for (int i = 0; i < nodeCount; i++)
    {
        JsonObject n = arr.createNestedObject();
        n["id"] = nodes[i].id;
        n["label"] = nodes[i].label;
        n["voltage"] = nodes[i].voltage;
        n["current"] = nodes[i].current;
        n["relay"] = nodes[i].relay ? 1 : 0;
        n["online"] = nodes[i].online ? 1 : 0;
    }

    // slaves array
    JsonArray sarr = doc.createNestedArray("slaves");
    for (int i = 0; i < total_Slave; i++)
    {
        if (slaves[i].id != 0)
        {
            JsonObject s = sarr.createNestedObject();
            s["id"] = slaves[i].id;
            s["temperature"] = slaves[i].temperature;
            s["time"] = slaves[i].time;
            s["slider"] = slaves[i].sliderValue;
            s["isOn"] = slaves[i].isOn ? 1 : 0;
            s["connected"] = slaves[i].isConnected ? 1 : 0;
        }
    }

    prefs.begin("wifi", true);
    String savedSsid = prefs.getString("ssid", "");
    int savedFan = prefs.getInt("fanThreshold", fanThreshold);
    prefs.end();

    doc["ssid"] = savedSsid.c_str();
    doc["fanThreshold"] = savedFan;

    String out;
    serializeJson(doc, out);
    statusServer.send(200, "application/json", out);
}
// ---------------- Relay API -----------------------
void handle_api_relay()
{
    if (!statusServer.hasArg("ch"))
    {
        statusServer.send(400, "text/plain", "missing ch");
        return;
    }

    String arg = statusServer.arg("ch");

    if (arg == "all")
    {
        // Toggle tất cả relay cùng lúc
        bool anyOn = false;
        for (int i = 0; i < 4; i++)
        {
            if (relayState[i])
            {
                anyOn = true;
                break;
            }
        }
        // Nếu có ít nhất 1 relay ON thì tắt hết, ngược lại bật hết
        bool newState = !anyOn;
        for (int i = 0; i < 4; i++)
        {
            setRelayLocal(i, newState);
        }
        statusServer.send(200, "application/json", "{\"ok\":1}");
        return;
    }

    // Xử lý từng relay riêng lẻ
    int ch = arg.toInt();
    if (ch < 0 || ch > 3)
    {
        statusServer.send(400, "text/plain", "invalid ch");
        return;
    }

    setRelayLocal(ch, !relayState[ch]);
    statusServer.send(200, "application/json", "{\"ok\":1}");
}
// ---------------- Add node ------------------------
void handle_api_node_add()
{
    if (!statusServer.hasArg("id"))
    {
        statusServer.send(400, "application/json", "{\"ok\":0,\"msg\":\"Missing id\"}");
        return;
    }
    int id = statusServer.arg("id").toInt();
    String label = "Node " + String(id);

    // Gửi gói test đến node
    LoRaPacketRec testCmd;
    testCmd.id = id;
    testCmd.data1 = false;
    testCmd.data2 = 0;
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&testCmd, sizeof(testCmd));
    LoRa.endPacket();

    unsigned long start = millis();
    bool found = false;

    // Chờ node trả lời trong 500ms
    while (millis() - start < 500)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize == sizeof(LoRaPacketSend))
        {
            LoRaPacketSend pkt;
            LoRa.readBytes((uint8_t *)&pkt, sizeof(pkt));
            if (pkt.id == id)
            {
                addNodeWithId(id, label, 0);
                slaves[id - 1].id = id;
                slaves[id - 1].temperature = pkt.data1;
                slaves[id - 1].time = pkt.data2;
                slaves[id - 1].isConnected = true;
                found = true;
                break;
            }
        }
    }

    DynamicJsonDocument doc(128);
    if (found)
    {
        doc["ok"] = 1;
        doc["msg"] = "Node added";
    }
    else
    {
        doc["ok"] = 0;
        doc["msg"] = "No response from node";
    }
    String out;
    serializeJson(doc, out);
    statusServer.send(200, "application/json", out);
}
// ---------------- remove node ---------------------
void handle_api_node_remove()
{
    if (!statusServer.hasArg("node"))
    {
        statusServer.send(400, "text/plain", "missing node");
        return;
    }
    int id = statusServer.arg("node").toInt();
    bool ok = removeNodeById(id);
    if (!ok)
        statusServer.send(404, "application/json", "{\"ok\":0, \"err\":\"not found\"}");
    else
        statusServer.send(200, "application/json", "{\"ok\":1}");
}
// ---------------- edit node -----------------------
void handle_api_node_edit()
{
    if (!statusServer.hasArg("node"))
    {
        statusServer.send(400, "text/plain", "missing node");
        return;
    }
    int nodeId = statusServer.arg("node").toInt();
    int idx = findNodeIndexById(nodeId);
    if (idx < 0)
    {
        statusServer.send(404, "application/json", "{\"ok\":0, \"err\":\"node not found\"}");
        return;
    }
    String newLabel = statusServer.arg("name");
    int newId = nodeId;
    if (statusServer.hasArg("id"))
        newId = statusServer.arg("id").toInt();

    // if id changed, ensure not duplicate
    if (newId != nodeId)
    {
        if (findNodeIndexById(newId) >= 0)
        {
            statusServer.send(400, "application/json", "{\"ok\":0, \"err\":\"id exists\"}");
            return;
        }
    }

    // apply changes
    int oldId = nodes[idx].id;
    nodes[idx].label = newLabel.length() ? newLabel : nodes[idx].label;
    nodes[idx].id = newId;

    // remap slave info if id changed and within range
    if (oldId != newId)
    {
        // clear old slave mapping if existed
        if (oldId > 0 && oldId <= total_Slave)
        {
            int oldsi = oldId - 1;
            slaves[oldsi].id = 0;
            slaves[oldsi].isConnected = false;
        }
        if (newId > 0 && newId <= total_Slave)
        {
            int newsi = newId - 1;
            slaves[newsi].id = newId;
            // slider and isOn persisted in EEPROM remain; we leave them.
            slaves[newsi].isConnected = false;
        }
    }

    // ensure nextNodeId updated
    if (newId >= nextNodeId)
        nextNodeId = newId + 1;

    saveNodesPrefs();

    statusServer.send(200, "application/json", "{\"ok\":1}");
}
// ---------------- Relay node ----------------------
void handle_api_node_relay()
{
    if (!statusServer.hasArg("node"))
    {
        statusServer.send(400, "text/plain", "missing node");
        return;
    }
    int id = statusServer.arg("node").toInt();
    int idx = findNodeIndexById(id);
    if (idx < 0)
    {
        statusServer.send(404, "application/json", "{\"ok\":0, \"err\":\"node not found\"}");
        return;
    }
    nodes[idx].relay = !nodes[idx].relay;
    if (id > 0 && id <= total_Slave)
    {
        int s = id - 1;
        slaves[s].isOn = nodes[idx].relay;
        int addr = s * 8;
        EEPROM.write(addr, slaves[s].isOn ? 1 : 0);
        EEPROM.commit();
        if (slaves[s].id != 0)
            sendLora(slaves[s].id, slaves[s].isOn ? 1 : 0, slaves[s].sliderValue);
    }
    saveNodesPrefs();
    statusServer.send(200, "application/json", "{\"ok\":1}");
}
// ---------------- dimming node --------------------
void handle_api_node_dim()
{
    if (!statusServer.hasArg("node") || !statusServer.hasArg("value"))
    {
        statusServer.send(400, "text/plain", "missing params");
        return;
    }
    int id = statusServer.arg("node").toInt();
    int val = statusServer.arg("value").toInt();
    if (id <= 0 || id > total_Slave)
    {
        statusServer.send(400, "application/json", "{\"ok\":0, \"err\":\"invalid node id\"}");
        return;
    }
    int sidx = id - 1;
    slaves[sidx].sliderValue = constrain(val, 0, 255);
    saveToEEPROM(sidx, slaves[sidx].isOn, slaves[sidx].sliderValue);
    sendLora(id, slaves[sidx].isOn ? 1 : 0, slaves[sidx].sliderValue);

    statusServer.send(200, "application/json", "{\"ok\":1}");
}
// ---------------- config save ---------------------
void handle_api_config_save()
{
    String ssid = statusServer.arg("ssid");
    String pass = statusServer.arg("pass");
    int fan = fanThreshold;
    if (statusServer.hasArg("fan"))
        fan = statusServer.arg("fan").toInt();

    prefs.begin("wifi", false);
    if (ssid.length())
        prefs.putString("ssid", ssid.c_str());
    if (pass.length())
        prefs.putString("pass", pass.c_str());
    prefs.putInt("fanThreshold", fan);
    prefs.end();

    fanThreshold = fan;

    if (ssid.length())
    {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(AP_SSID, AP_PASS);
        WiFi.begin(ssid.c_str(), pass.c_str());
    }

    statusServer.send(200, "application/json", "{\"ok\":1}");
}
// ---------------- Start server --------------------
void startStatusServer()
{
    statusServer.on("/", HTTP_GET, []()
                    { statusServer.send(200, "text/html", status_html); });
    statusServer.on("/api/status", HTTP_GET, handle_api_status);
    statusServer.on("/api/relay", HTTP_POST, handle_api_relay);
    statusServer.on("/api/node/add", HTTP_POST, handle_api_node_add);
    statusServer.on("/api/node/remove", HTTP_POST, handle_api_node_remove);
    statusServer.on("/api/node/edit", HTTP_POST, handle_api_node_edit);
    statusServer.on("/api/node/relay", HTTP_POST, handle_api_node_relay);
    statusServer.on("/api/node/dim", HTTP_POST, handle_api_node_dim);
    statusServer.on("/api/config/save", HTTP_POST, handle_api_config_save);
    statusServer.begin();
}
// ---------------- save to EEPROM ------------------
void saveToEEPROM(int slaveId, bool isOn, int sliderValue)
{
    if (slaveId < 0 || slaveId >= total_Slave)
        return;

    int addr = slaveId * 8;
    EEPROM.write(addr, isOn ? 1 : 0);
    EEPROM.write(addr + 4, sliderValue & 0xFF);
    EEPROM.commit();
}
// ---------------- Send data to LoRa nodes ---------
void sendLora(int ID, int stateLed, int valvePwm)
{
    if (ID <= 0 || ID > total_Slave)
        return;
    int idx = ID - 1;
    if (idx < 0 || idx >= total_Slave)
        return;

    saveToEEPROM(idx, slaves[idx].isOn, slaves[idx].sliderValue);
    slaves[idx].isConnected = false;

    packetToSend.id = ID;
    packetToSend.data1 = stateLed ? 1 : 0;
    packetToSend.data2 = valvePwm;

    LoRa.beginPacket();
    LoRa.write((uint8_t *)&packetToSend, sizeof(packetToSend));
    LoRa.endPacket();
}
// ================ Connect to WiFi =================
void connectWiFiSTA()
{
    Preferences prefs;
    prefs.begin("settings", false);
    String ssid = prefs.getString("sta_ssid", "");
    String pass = prefs.getString("sta_pass", "");
    prefs.end();

    if (ssid == "" || pass == "")
    {
        Serial.println("[MQTT] Chưa có SSID/PASS trong Preferences!");
        return;
    }

    WiFi.mode(WIFI_AP_STA); // vừa AP cho WebPortal, vừa STA để lên internet
    WiFi.begin(ssid.c_str(), pass.c_str());

    Serial.printf("[MQTT] Connecting to SSID: %s\n", ssid.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
    {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\n[MQTT] WiFi STA connected: " + WiFi.localIP().toString());
    }
    else
    {
        Serial.println("\n[MQTT] WiFi STA connect failed!");
    }
}
// ================ connect to MQTT =================
void connectMQTT()
{
    while (!mqttClient.connected())
    {
        Serial.print("Connecting MQTT...");
        if (mqttClient.connect("ESP32Master"))
        {
            Serial.println("connected!");
            mqttClient.subscribe("esp32/relay/cmd"); // lệnh điều khiển relay
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}
// ================ Publish Status ==================
void publishStatus()
{
    StaticJsonDocument<512> doc;
    doc["temp"] = readInternalTemp();
    doc["fan"] = fanState;

    JsonArray rel = doc.createNestedArray("relays");
    for (int i = 0; i < 4; i++)
        rel.add(relayState[i]);

    JsonArray sl = doc.createNestedArray("slaves");
    for (int i = 0; i < total_Slave; i++)
    {
        if (slaves[i].id != 0)
        {
            JsonObject s = sl.createNestedObject();
            s["id"] = slaves[i].id;
            s["relay"] = slaves[i].isOn;
            s["dimming"] = slaves[i].sliderValue;
            s["connected"] = slaves[i].isConnected;
        }
    }

    char buffer[512];
    size_t n = serializeJson(doc, buffer);
    mqttClient.publish("esp32/status", buffer, n);
}
// ================ MQTT callback ===================
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    payload[length] = '\0';
    String msg = String((char *)payload);

    if (String(topic) == "esp32/relay/cmd")
    {
        if (msg == "all_on")
        {
            for (int i = 0; i < 4; i++)
                setRelayLocal(i, true);
        }
        else if (msg == "all_off")
        {
            for (int i = 0; i < 4; i++)
                setRelayLocal(i, false);
        }
        else
        {
            int ch = msg.toInt(); // "0","1","2","3"
            if (ch >= 0 && ch < 4)
            {
                setRelayLocal(ch, !relayState[ch]);
            }
        }
        Serial.printf("[MQTT] Relay cmd: %s\n", msg.c_str());
    }
}
// ---------------- Standby Screen ------------------
void standby_screen()
{
    DateTime now = rtc.now();
    char tbuf[16];
    sprintf(tbuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    u8g2.setFont(u8g2_font_ncenB18_te);
    u8g2.setCursor(18, 38);
    u8g2.print(tbuf);
}
// ---------------- Data Screen ---------------------
void data_screen()
{
    // basic data screen + rotating node info
    u8g2.setCursor(52, 33);
    u8g2.drawXBM(36, 17, 16, 16, image_weather_temperature_bits);
    u8g2.setFont(u8g2_font_profont12_tr);
    u8g2.printf("%.1f", readInternalTemp());
    u8g2.drawXBM(80, 25, 10, 8, icon_Thermal);
    DateTime now = rtc.now();
    char tbuf[16];
    sprintf(tbuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    u8g2.setFont(u8g2_font_12x6LED_mn);
    u8g2.setCursor(33, 12);
    u8g2.print(tbuf);
    u8g2.drawLine(0, 15, 127, 15);
    u8g2.drawLine(34, 15, 34, 63);
    u8g2.drawLine(92, 15, 92, 63);
    u8g2.drawLine(34, 40, 0, 40);
    u8g2.drawLine(92, 40, 127, 40);

    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.setColorIndex(digitalRead(RL1) == HIGH ? 0 : 1);
    u8g2.drawStr(5, 34, "RL1");
    u8g2.setColorIndex(digitalRead(RL2) == HIGH ? 0 : 1);
    u8g2.drawStr(5, 58, "RL2");
    u8g2.setColorIndex(digitalRead(RL3) == HIGH ? 0 : 1);
    u8g2.drawStr(99, 34, "RL3");
    u8g2.setColorIndex(digitalRead(RL4) == HIGH ? 0 : 1);
    u8g2.drawStr(99, 58, "RL4");
    u8g2.setColorIndex(1);

    if (WiFi.status())
    {
        u8g2.drawXBM(115, 4, 9, 7, icon_connected);
    }
    else
    {
        u8g2.drawXBM(115, 4, 9, 7, icon_disconnect);
    }

    // rotate through nodes if exist
    static unsigned long lastSwitch = 0;
    static int displayIndex = 0;
    int validCount = 0;
    for (int i = 0; i < nodeCount; i++)
        if (nodes[i].id > 0)
            validCount++;
    if (validCount > 0)
    {
        if (millis() - lastSwitch >= 3000)
        {
            displayIndex = (displayIndex + 1) % validCount;
            lastSwitch = millis();
        }
        // find displayIndex-th valid node
        int found = -1;
        int cnt = 0;
        for (int i = 0; i < nodeCount; i++)
        {
            if (nodes[i].id <= 0)
                continue;
            if (cnt == displayIndex)
            {
                found = i;
                break;
            }
            cnt++;
        }
        if (found >= 0)
        {
            int nid = nodes[found].id;
            int slider = 0;
            bool s_on = false;
            bool s_conn = false;
            if (nid > 0 && nid <= total_Slave)
            {
                int si = nid - 1;
                slider = slaves[si].sliderValue;
                s_on = slaves[si].isOn;
                s_conn = slaves[si].isConnected;
            }
            char buf[32];
            char buf1[32];
            snprintf(buf, sizeof(buf), "ID:%d", nid);
            snprintf(buf1, sizeof(buf1), "Tag: %s", nodes[found].label.c_str());
            u8g2.setFont(u8g2_font_5x7_mf);
            u8g2.setCursor(36, 43);
            u8g2.printf(buf);
            u8g2.setCursor(36, 51);
            u8g2.printf(buf1);
            char buf2[64];
            snprintf(buf2, sizeof(buf2), "D:%d RL:%s", slider, nodes[found].relay ? "ON" : "OFF", s_conn ? "C" : "D");
            u8g2.setCursor(36, 60);
            u8g2.print(buf2);
        }
    }
}
// ---------------- Display Task --------------------
void displayTask(void *pvParameters)
{
    // esp_task_wdt_init(WDT_TIMEOUT, true);
    // esp_task_wdt_add(NULL);
    (void)pvParameters;
    u8g2.begin();
    while (1)
    {
        u8g2.clearBuffer();
        if (!inMenu)
        {
            if (millis() - lastActivity >= standbyTimeout)
            {
                datascreenflag = false;
                standby_screen();
            }
            else
            {
                datascreenflag = true;
                data_screen();
            }
        }
        else
        {
            if (menuLevel == 1)
            {
                u8g2.setFont(u8g2_font_ncenB08_tr);
                for (int i = 0; i < mainMenuCount; i++)
                {
                    if (i == menuCursor)
                        u8g2.drawRFrame(0, i * 15, 128, 15, 3);
                    u8g2.setCursor(6, (i + 1) * 15 - 3);
                    u8g2.print(mainMenu[i]);
                }
            }
            else if (menuLevel == 2)
            {
                if (submenuSelected == 0)
                {
                    u8g2.setFont(u8g2_font_ncenB12_te);
                    u8g2.drawStr(6, 14, "Time Setup");
                    char buf[20];
                    sprintf(buf, "%02d:%02d:%02d", setHour, setMinute, setSecond);
                    u8g2.setCursor(18, 32);
                    u8g2.print(buf);
                    if (editingTime)
                    {
                        int xPos[] = {18, 42, 66};
                        u8g2.drawRFrame(xPos[cursorPos], 18, 20, 18, 3);
                        u8g2.setFont(u8g2_font_6x10_tr);
                    }
                }
                else if (submenuSelected == 1)
                {
                    u8g2.setFont(u8g2_font_ncenB08_tr);
                    u8g2.drawStr(6, 14, "Flip Screen");
                    u8g2.setCursor(6, 28);
                    u8g2.print("Enable");
                    u8g2.setCursor(6, 40);
                    u8g2.print("Disable");
                    if (screenFlip)
                        u8g2.drawRFrame(0, 18, 128, 12, 3);
                    else
                        u8g2.drawRFrame(0, 30, 128, 12, 3);
                }
                else if (submenuSelected == 2)
                {
                    u8g2.setFont(u8g2_font_ncenB08_tr);
                    u8g2.drawStr(6, 14, "Internet");
                    prefs.begin("wifi", true);
                    String ss = prefs.getString("ssid", "");
                    prefs.end();
                    u8g2.setCursor(2, 20 + 12);
                    if (ss.length() == 0)
                        u8g2.print("No SSID configured");
                    else
                    {
                        u8g2.setFont(u8g2_font_6x10_tr);
                        u8g2.setCursor(6, 24);
                        u8g2.printf("SSID: %s", WiFi.SSID().c_str());
                        u8g2.setCursor(6, 36);
                        u8g2.printf("IP:%s", WiFi.localIP().toString().c_str());
                    }
                }
                else if (submenuSelected == 3)
                {
                    inMenu = false;
                    menuLevel = 0;
                }
            }
        }
        u8g2.sendBuffer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // esp_task_wdt_reset();
}
// ---------------- LoRa Task -----------------------
void loraTask(void *pvParameters)
{
    // esp_task_wdt_init(WDT_TIMEOUT, true);
    // esp_task_wdt_add(NULL);
    (void)pvParameters;
    unsigned long lastSend = 0;
    while (1)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            if (packetSize == sizeof(LoRaPacketRec))
            {
                LoRa.readBytes((uint8_t *)&receivedPacket, sizeof(receivedPacket));

                int id = receivedPacket.id;
                if (id > 0 && id <= total_Slave)
                {
                    int sidx = id - 1;
                    slaves[sidx].id = id;
                    slaves[sidx].temperature = receivedPacket.data1;
                    slaves[sidx].time = (int)receivedPacket.data2;
                    slaves[sidx].isConnected = true;
                    slaves[sidx].timeString = String(millis() / 1000) + "s";

                    updateNodeFromLoRa(id, slaves[sidx].temperature, (float)slaves[sidx].time, slaves[sidx].isOn);
                }
                Serial.printf("[LoRa RX] id=%d temp=%.2f time=%lu\n", receivedPacket.id, receivedPacket.data1, receivedPacket.data2);
            }
            else
            {
                uint8_t buf[256];
                int toRead = min(packetSize, (int)sizeof(buf));
                LoRa.readBytes(buf, toRead);
                Serial.printf("[LoRa RX] unexpected size: %d\n", packetSize);
            }
        }

        if (millis() - lastSend >= 5000)
        {
            for (int i = 0; i < total_Slave; i++)
            {
                if (slaves[i].id != 0)
                {
                    sendLora(slaves[i].id, slaves[i].isOn ? 1 : 0, slaves[i].sliderValue);
                }
            }
            lastSend = millis();
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    // esp_task_wdt_reset();
}
// ---------------- IO task (core 1) ----------------
void ioTask(void *pvParameters)
{
    // esp_task_wdt_init(WDT_TIMEOUT, true);
    // esp_task_wdt_add(NULL);
    (void)pvParameters;
    pinMode(BT_UP, INPUT_PULLUP);
    pinMode(BT_SEL, INPUT_PULLUP);
    pinMode(BT_DN, INPUT_PULLUP);
    pinMode(BT_BACK, INPUT_PULLUP);
    pinMode(BT_BOOT, INPUT_PULLUP);
    pinMode(BUZ_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);

    for (int i = 0; i < 4; i++)
    {
        pinMode(relayPins[i], OUTPUT);
        setRelayLocal(i, relayState[i]);
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    startStatusServer();

    lastActivity = millis();

    unsigned long pressStart[5] = {0, 0, 0, 0, 0};
    bool pressed[5] = {false, false, false, false, false};

    while (1)
    {
        statusServer.handleClient();
        buzzerUpdate();

        int states[5] = {
            digitalRead(BT_UP),
            digitalRead(BT_SEL),
            digitalRead(BT_DN),
            digitalRead(BT_BACK),
            digitalRead(BT_BOOT)};

        if (!inMenu)
        {
            for (int b = 0; b < 5; b++)
            {
                if (states[b] == LOW)
                {
                    if (!pressed[b])
                    {
                        pressed[b] = true;
                        lastActivity = millis();
                        pressStart[b] = millis();
                    }
                    else
                    {
                        unsigned long held = millis() - pressStart[b];

                        if (b == 4 && held >= 2000)
                        {
                            inMenu = true;
                            menuLevel = 1;
                            menuCursor = 0;
                            submenuSelected = -1;
                            buzzerBeep(1500, 120);
                            pressed[b] = false;
                        }
                        else if (held >= 2000)
                        {
                            setRelayLocal(b, !relayState[b]);
                            buzzerBeep(1500, 120);
                            pressed[b] = false;
                        }
                    }
                }
                else
                {
                    pressed[b] = false;
                }
            }
        }
        if (inMenu)
        {
            if (menuLevel == 2 && submenuSelected == 0)
            {
                // Time Setting
                if (digitalRead(BT_UP) == LOW)
                {
                    buzzerBeep(1200, 60);
                    if (cursorPos == 0)
                        setHour = (setHour + 1) % 24;
                    else if (cursorPos == 1)
                        setMinute = (setMinute + 1) % 60;
                    else
                        setSecond = (setSecond + 1) % 60;
                    while (digitalRead(BT_UP) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_DN) == LOW)
                {
                    buzzerBeep(1200, 60);
                    if (cursorPos == 0)
                        setHour = (setHour + 23) % 24;
                    else if (cursorPos == 1)
                        setMinute = (setMinute + 59) % 60;
                    else
                        setSecond = (setSecond + 59) % 60;
                    while (digitalRead(BT_DN) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_SEL) == LOW)
                {
                    buzzerBeep(1000, 60);
                    cursorPos = (cursorPos + 1) % 3;
                    editingTime = true;
                    while (digitalRead(BT_SEL) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_BACK) == LOW)
                {
                    buzzerBeep(900, 80);
                    DateTime now = rtc.now();
                    rtc.adjust(DateTime(now.year(), now.month(), now.day(), setHour, setMinute, setSecond));
                    menuLevel = 1;
                    submenuSelected = -1;
                    editingTime = false;
                    cursorPos = 0;
                    while (digitalRead(BT_BACK) == LOW)
                        vTaskDelay(10);
                }
            }
            else if (menuLevel == 2 && submenuSelected == 1)
            {
                // Screen Flip
                if (digitalRead(BT_UP) == LOW || digitalRead(BT_DN) == LOW)
                {
                    buzzerBeep(1000, 50);
                    screenFlip = !screenFlip;
                    while (digitalRead(BT_UP) == LOW || digitalRead(BT_DN) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_SEL) == LOW)
                {
                    buzzerBeep(1000, 80);
                    if (screenFlip)
                        u8g2.setDisplayRotation(U8G2_R2);
                    else
                        u8g2.setDisplayRotation(U8G2_R0);
                    while (digitalRead(BT_SEL) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_BACK) == LOW)
                {
                    buzzerBeep(900, 80);
                    menuLevel = 1;
                    submenuSelected = -1;
                    while (digitalRead(BT_BACK) == LOW)
                        vTaskDelay(10);
                }
            }
            else
            {
                // Menu navigation
                if (digitalRead(BT_UP) == LOW)
                {
                    buzzerBeep(1000, 50);
                    menuCursor = (menuCursor - 1 + mainMenuCount) % mainMenuCount;
                    while (digitalRead(BT_UP) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_DN) == LOW)
                {
                    buzzerBeep(1000, 50);
                    menuCursor = (menuCursor + 1) % mainMenuCount;
                    while (digitalRead(BT_DN) == LOW)
                        vTaskDelay(10);
                }
                if (digitalRead(BT_SEL) == LOW)
                {
                    buzzerBeep(1000, 80);
                    if (menuLevel == 1)
                    {
                        submenuSelected = menuCursor;
                        menuLevel = 2;
                        menuCursor = 0;
                        if (submenuSelected == 0)
                        {
                            DateTime now = rtc.now();
                            setHour = now.hour();
                            setMinute = now.minute();
                            setSecond = now.second();
                            editingTime = true;
                        }
                    }
                    while (digitalRead(BT_SEL) == LOW)
                        vTaskDelay(10);
                }

                if (digitalRead(BT_BACK) == LOW)
                {
                    buzzerBeep(1000, 80);
                    if (menuLevel == 2)
                    {
                        menuLevel = 1;
                        submenuSelected = -1;
                    }
                    else
                    {
                        inMenu = false;
                        menuLevel = 0;
                        submenuSelected = -1;
                    }
                    while (digitalRead(BT_BACK) == LOW)
                        vTaskDelay(10);
                }
            }
        }

        // ----------------------
        // Fan theo nhiệt độ
        // ----------------------
        float t = readInternalTemp();
        if (t >= fanThreshold)
        {
            digitalWrite(FAN_PIN, HIGH);
            fanState = true;
        }
        else
        {
            digitalWrite(FAN_PIN, LOW);
            fanState = false;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    // esp_task_wdt_reset();
}
// ---------------- MQTT Task -----------------------
void mqttTask(void *pvParameters)
{
    // esp_task_wdt_init(WDT_TIMEOUT, true);
    // esp_task_wdt_add(NULL);
    connectWiFiSTA();

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    unsigned long lastPub = 0;

    for (;;)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            if (!mqttClient.connected())
            {
                connectMQTT();
            }
            mqttClient.loop();

            if (millis() - lastPub > 5000)
            {
                publishStatus();
                lastPub = millis();
            }
        }
        else
        {
            connectWiFiSTA();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // esp_task_wdt_reset();
}
// ---------------- Setup / Loop --------------------
void setup()
{
    Serial.begin(115200);
    EEPROM.begin(512);
    if (!rtc.begin())
    {
        for (int i = 0; i < 3; i++)
        {
            buzzerBeep(2000, 100);
            vTaskDelay(150 / portTICK_PERIOD_MS);
        }
    }

    prefs.begin("wifi", true);
    if (prefs.isKey("fanThreshold"))
    {
        fanThreshold = prefs.getInt("fanThreshold", fanThreshold);
    }
    String savedSsid = prefs.getString("ssid", "");
    String savedPass = prefs.getString("pass", "");
    prefs.end();

    if (savedSsid.length())
    {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(AP_SSID, AP_PASS);
        WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    }
    initNodes();
    initBuzzer();
    loadNodesPrefs();

    prefs.begin("relay", true);
    if (prefs.isKey("r0"))
    {
        relayState[0] = prefs.getInt("r0", 0);
    }
    if (prefs.isKey("r1"))
    {
        relayState[1] = prefs.getInt("r1", 0);
    }
    if (prefs.isKey("r2"))
    {
        relayState[2] = prefs.getInt("r2", 0);
    }
    if (prefs.isKey("r3"))
    {
        relayState[3] = prefs.getInt("r3", 0);
    }
    prefs.end();

    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);

    for (int i = 0; i < 4; i++)
    {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], relayState[i] ? HIGH : LOW);
    }

    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &displayTaskHandle, 0);
    xTaskCreatePinnedToCore(ioTask, "IOTask", 8192, NULL, 1, &ioTaskHandle, 1);
    xTaskCreatePinnedToCore(loraTask, "LoRaTask", 4096, NULL, 1, &loraTaskHandle, 1);
    // xTaskCreatePinnedToCore(relayStatusTask, "RelayStatus", 4096, NULL, 1, &relaytaskhandle, 1);
    xTaskCreatePinnedToCore(mqttTask, "MQTTTask", 4096, NULL, 1, NULL, 1); // chạy core1

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    startStatusServer();
}
// ---------------- void loop -----------------------
void loop() {}