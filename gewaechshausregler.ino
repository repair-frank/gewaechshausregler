
#include <string.h>
#include <string>
#include <WiFi.h>
#include <WiFiAP.h>
#include <HTTPClient.h>
#include <HTTPServer.hpp>
#include <HTTPRequest.hpp>
#include <HTTPResponse.hpp>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

#define TOLLER_TITEL              "Regler"
// input (sensors)
#define PIN_BODEN_FEUCHT          32
#define PIN_BODEN_TEMP            14                                            // one wire
#define PIN_DHT22                 13
#define PIN_LICHT                 33
// output
#define PIN_LUEFTER               5
#define PIN_PUMPE                 4
// task intervalls
#define MDNS_RETRIES              5
#define INTERVALL_READ_INPUT      (500 / portTICK_RATE_MS)
#define INTERVALL_UPDATE_LAMPE    (3000 / portTICK_RATE_MS)
#define INTERVALL_UPDATE_LUEFTER  (2000 / portTICK_RATE_MS)
#define INTERVALL_UPDATE_PUMPE    (500 / portTICK_RATE_MS)
// minimum value from light sensor to count as "sunlight"
#define LIGHT_MINIMUM             (0.5)
// minimum time of being lit to count as dawn (seconds)
#define DAWN_MINIMUM_LENGTH       (60 * 30)
// path to config file in SPIFFS
#define CONFIG_PATH               "/config"

#define HOUR_IN_SECS              (60 * 60)
#define DAY_IN_SECS               (24 * HOUR_IN_SECS)

using namespace std;
using namespace httpsserver;

struct sensorValue
{
  volatile float value;
  volatile bool valid;
};

const char* ssid = "SSID des AP";
const char* passwort = "mindestens8zeichen";

extern const char* error404Html;
extern const char* rootHtml;

OneWire oneWireInstance(PIN_BODEN_TEMP);
DallasTemperature bodenTempSensor(&oneWireInstance);
DHT dht22Sensor(PIN_DHT22, DHT22);

volatile IPAddress lampeIP = (uint32_t) 0;

//zustand input     wert bzw. nicht erreichbar
volatile struct sensorValue currBodenFeucht;
volatile struct sensorValue currBodenTemp;
volatile struct sensorValue currLuftFeucht;
volatile struct sensorValue currLuftTemp;
volatile struct sensorValue currLicht;

//zustand output    läuft, läuft nicht, fehler
volatile bool stateLampe = false;
volatile bool lampeManualOn = false;
volatile bool stateLuefter = false;
volatile bool luefterManualOn = false;
volatile bool statePumpe = false;
volatile bool pumpeManualOn = false;

//zielzustand       wert bzw. egal
volatile struct sensorValue targetBodenFeucht;
volatile struct sensorValue targetBodenTemp;
volatile struct sensorValue targetLuftFeucht;
volatile struct sensorValue targetLuftTemp;

volatile struct sensorValue targetSonnenstunden;

HTTPServer httpServer = HTTPServer();

// forward declarations, definitions below loop()
// tasks
void readInputTask(void* param);
void updateLampeTask(void* param);
void updateLuefterTask(void* param);
void updatePumpeTask(void* param);
// eventhandler für steckdose
void wiFiStaConnectHandler(WiFiEvent_t, WiFiEventInfo_t);
void wiFiStaDisconnectHandler(WiFiEvent_t, WiFiEventInfo_t);
// speichern
int configToSPIFFS(const char* path);
int configFromSPIFFS(const char* path);
// httpserver kram
void httpServerTask(void* param);
void httpRootGetHandler(HTTPRequest* request, HTTPResponse* response);
void httpConfigGetHandler(HTTPRequest* request, HTTPResponse* response);
void httpConfigPostHandler(HTTPRequest* request, HTTPResponse* response);
void httpStateGetHandler(HTTPRequest* request, HTTPResponse* response);
void httpLampePostHandler(HTTPRequest* request, HTTPResponse* response);
void httpLuefterPostHandler(HTTPRequest* request, HTTPResponse* response);
void httpPumpePostHandler(HTTPRequest* request, HTTPResponse* response);
void httpError404Handler(HTTPRequest* request, HTTPResponse* response);

void setup()
{
  // initialize log
  Serial.begin(115200);
  Serial.println("setup()...");
  
  pinMode(PIN_LUEFTER, OUTPUT);
  pinMode(PIN_PUMPE, OUTPUT);
  
  // load last config from SPIFFS
  if (!SPIFFS.begin(false))
    Serial.println("SPIFFS konnte nicht initialisiert werden.");
  else
    configFromSPIFFS(CONFIG_PATH);

  // start WiFi AP
  Serial.println("WiFi AP wird gestartet...");
  WiFi.softAP(ssid, passwort, 6);
  Serial.print("WiFi AP gestartet, SSID <");
  Serial.print(ssid);
  Serial.print(">, Passwort <");
  Serial.print(passwort);
  Serial.print(">, IP ");
  Serial.print(WiFi.localIP());
  Serial.println(".");
  
  // initialize MDNS
  for (int i = 0; !MDNS.begin(TOLLER_TITEL) && i < MDNS_RETRIES; i++)
  {
    Serial.print("MDNS konnte nicht initialisiert werden, Versuch Nummer ");
    Serial.print(i + 1);
    Serial.println(".");
  }

  // register WiFi event handler
  WiFi.onEvent(wiFiStaConnectHandler, SYSTEM_EVENT_AP_STAIPASSIGNED);           // erst wenn DHCP fertig ist
  WiFi.onEvent(wiFiStaDisconnectHandler, SYSTEM_EVENT_AP_STADISCONNECTED);      // wiFiEventHandlers will get called when a station connects or disconnects to/from the AP
  Serial.println("WiFi-Eventhandler wurden registriert.");

  // start sensors
  bodenTempSensor.begin();
  dht22Sensor.begin();

  // create http server task
  xTaskCreatePinnedToCore(httpServerTask, "httpServerTask", 4096, NULL, 1, NULL, 0);

  // create IO tasks
  xTaskCreatePinnedToCore(readInputTask, "readInputTask", 4096, NULL, 3, NULL, 1);
  xTaskCreate(updateLampeTask, "updateLampeTask", 2048, NULL, 1, NULL);
  xTaskCreate(updateLuefterTask, "updateLuefterTask", 2048, NULL, 1, NULL);
  xTaskCreate(updatePumpeTask, "updatePumpeTask", 2048, NULL, 1, NULL);
}

void loop()
{
  delay(2000);
}





volatile bool targetLightState = false;

bool isLit(void)
{
  return currLicht.valid && currLicht.value > LIGHT_MINIMUM;
}


void readInputTask(void* param)
{
  TickType_t lastTime = xTaskGetTickCount();
  while (true)
  {
    float temp;
    //bodenfeuchte lesen
    currBodenFeucht.value = 1f - (analogRead(PIN_BODEN_FEUCHT) / 4095f);
    currBodenFeucht.valid = true;
    // bodentemp lesen
    bodenTempSensor.requestTemperatures();
    DeviceAddress address;
    if (bodenTempSensor.getAddress(address, 0) &&
      (temp = bodenTempSensor.getTempC(address)) != DEVICE_DISCONNECTED_C)
    {
      currBodenTemp.value = temp;
      currBodenTemp.valid = true;
    }
    else
    {
      currBodenTemp.valid = false;
      Serial.println("Konnte Bodentemperatur nicht messen!");
    }
    // luftfeuchte lesen
    if (!isnan(temp = dht22Sensor.readHumidity()))
    {
      currLuftFeucht.value = temp;
      currLuftFeucht.valid = true;
    }
    else
    {
      currLuftFeucht.valid = false;
      Serial.println("Konnte Luftfeuchte nicht messen!");
    }
    // lufttemp lesen
    if (!isnan(temp = dht22Sensor.readTemperature()))
    {
      currLuftTemp.value = temp;
      currLuftTemp.valid = true;
    }
    else
    {
      currLuftTemp.valid = false;
      Serial.println("Konnte Lufttemperatur nicht messen!");
    }
    // licht lesen
    currLicht.value = (analogRead(PIN_LICHT) / 4095f);
    currLicht.valid = true;
    vTaskDelayUntil(&lastTime, INTERVALL_READ_INPUT);
  }
}

bool lampeTargetState()
{
  return targetLightState && !isLit();
}

void updateLampeTask(void* param)
{
  time_t endOfDay = 0;
  time_t lightBegin = -1;
  time_t sonnenstundenBegin = -1;
  time_t sonnenstundenToday = 0;
  time_t sonnenstundenCarry = 0;
  HTTPClient webClient;
  TickType_t lastTime = xTaskGetTickCount();
  while (true)
  {
    // lichtzyklus kram
    time_t now = time(NULL);
    if (now >= endOfDay)
    {
      if (sonnenstundenBegin >= 0)
      {
        sonnenstundenToday += endOfDay - sonnenstundenBegin;
        sonnenstundenBegin = endOfDay;
      }
      sonnenstundenCarry = sonnenstundenToday -
        targetSonnenstunden.valid * targetSonnenstunden.value * HOUR_IN_SECS;
      if (sonnenstundenCarry < 0)
        sonnenstundenCarry = 0;
      sonnenstundenToday = 0;
      endOfDay += DAY_IN_SECS;
    }
    if (sonnenstundenBegin >= 0)
    {
      sonnenstundenToday += now - sonnenstundenBegin;
    }
    targetLightState = targetSonnenstunden.valid &&
      ((sonnenstundenToday >= DAWN_MINIMUM_LENGTH &&
        sonnenstundenToday + sonnenstundenCarry < targetSonnenstunden.value * HOUR_IN_SECS) ||
      (now >=
        endOfDay + sonnenstundenToday + sonnenstundenCarry -
        targetSonnenstunden.value * HOUR_IN_SECS))
    if (isLit() || targetLightState)
      sonnenstundenBegin = now;
    else
      sonnenstundenBegin = -1;

    //Serial.println("Lampe wird geupdatet...");
    if (lampeManualOn || lampeTargetState())
    {
      if (!stateLampe && lampeIP != (uint32_t) 0)
      {
        Serial.println("Lampe wird angeschaltet...");
        // lampe anschalten, stateLampe updaten
        webClient.begin(("http://" + lampeIP.toString() + "/cm?cmnd=Power%20On").c_str());
        stateLampe = (webClient.GET() == 200);
        webClient.end();
      }
    }
    else if (stateLampe && lampeIP != (uint32_t) 0)
    {
      Serial.println("Lampe wird ausgeschaltet...");
      // lampe ausschalten, stateLampe updaten
      webClient.begin(("http://" + lampeIP.toString() + "/cm?cmnd=Power%20off").c_str());
      stateLampe = !(webClient.GET() == 200);
      webClient.end();
    }
    vTaskDelayUntil(&lastTime, INTERVALL_UPDATE_LAMPE);
  }
}

bool luefterTargetState()
{
  return (targetLuftFeucht.valid && currLuftFeucht.valid &&
      currLuftFeucht.value > targetLuftFeucht.value) ||
    (targetLuftTemp.valid && currLuftTemp.valid &&
      currLuftTemp.value > targetLuftTemp.value);
}

void updateLuefterTask(void* param)
{
  TickType_t lastTime = xTaskGetTickCount();
  while (true)
  {
    //Serial.println("Lüfter wird geupdatet...");
    if (luefterManualOn || luefterTargetState())
    {
      if (!stateLuefter)
      {
        Serial.println("Lüfter wird angeschaltet...");
        // luefter anschalten, stateLuefter updaten
        digitalWrite(PIN_LUEFTER, HIGH);
        stateLuefter = true;
      }
    }
    else if (stateLuefter)
    {
      Serial.println("Lüfter wird ausgeschaltet...");
      // luefter ausschalten, stateLuefter updaten
      digitalWrite(PIN_LUEFTER, LOW);
      stateLuefter = false;
    }
    vTaskDelayUntil(&lastTime, INTERVALL_UPDATE_LUEFTER);
  }
}

bool pumpeTargetState()
{
  return targetBodenFeucht.valid && currBodenFeucht.valid &&
    currBodenFeucht.value < targetBodenFeucht.value;
}

void updatePumpeTask(void* param)
{
  TickType_t lastTime = xTaskGetTickCount();
  while (true)
  {
    //Serial.println("Pumpe wird geupdatet...");
    if (pumpeManualOn || pumpeTargetState())
    {
      if (!statePumpe)
      {
        Serial.println("Pumpe wird angeschaltet...");
        // pumpe anschalten, statePumpe updaten
        digitalWrite(PIN_PUMPE, HIGH);
        statePumpe = true;
      }
    }
    else if (statePumpe)
    {
      Serial.println("Pumpe wird ausgeschaltet...");
      // pumpe ausschalten, statePumpe updaten
      digitalWrite(PIN_PUMPE, LOW);
      statePumpe = false;
    }
    vTaskDelayUntil(&lastTime, INTERVALL_UPDATE_PUMPE);
  }
}





// configfile ist ein char mit 5 flags, ob werte danach valid sind, danach 5 floats
// also char validflags, float bf, float bt, float lf, float lt, float ss

const int cfgFileSize = sizeof(unsigned char) + 5 * sizeof(float);

int readConfig(unsigned char* cfgFile, int length)
{
  if (length < cfgFileSize)
    return -1;
  float* values = (float*) &cfgFile[1];
  int n = 0;
  if (cfgFile[0] & 1U)
  {
    targetBodenFeucht.value = values[0];
    targetBodenFeucht.valid = true;
    n++;
  }
  if (cfgFile[0] & (1U << 1))
  {
    targetBodenTemp.value = values[1];
    targetBodenTemp.valid = true;
    n++;
  }
  if (cfgFile[0] & (1U << 2))
  {
    targetLuftFeucht.value = values[2];
    targetLuftFeucht.valid = true;
    n++;
  }
  if (cfgFile[0] & (1U << 3))
  {
    targetLuftTemp.value = values[3];
    targetLuftTemp.valid = true;
    n++;
  }
  if (cfgFile[0] & (1U << 4))
  {
    targetSonnenstunden.value = values[4];
    targetSonnenstunden.valid = true;
    n++;
  }
  return n;
}

int writeConfig(unsigned char* buffer, int length)
{
  if (length < cfgFileSize)
    return -1;
  float* values = (float*) &buffer[1];
  int n = 0;
  if (targetBodenFeucht.valid)
  {
    values[0] = targetBodenFeucht.value;
    buffer[0] |= 1U;
    n++;
  }
  if (targetBodenTemp.valid)
  {
    values[1] = targetBodenTemp.value;
    buffer[0] |= 1U << 1;
    n++;
  }
  if (targetLuftFeucht.valid)
  {
    values[2] = targetLuftFeucht.value;
    buffer[0] |= 1U << 2;
    n++;
  }
  if (targetLuftTemp.valid)
  {
    values[3] = targetLuftTemp.value;
    buffer[0] |= 1U << 3;
    n++;
  }
  if (targetSonnenstunden.valid)
  {
    values[4] = targetSonnenstunden.value;
    buffer[0] |= 1U << 4;
    n++;
  }
  return n;
}

int configToSPIFFS(const char* path)
{
  Serial.print("Konfiguration wird im SPIFFS gespeichert (");
  Serial.print(path);
  Serial.println(").");
  File cfgFile = SPIFFS.open(path, FILE_WRITE);
  if (!cfgFile || cfgFile.isDirectory())
  {
    Serial.print(path);
    Serial.println(" konnte nicht geöffnet werden.");
    return -1;
  }
  unsigned char cfgBytes[cfgFileSize];
  cfgBytes[0] = 0;
  if (writeConfig(cfgBytes, cfgFileSize) < 0)
  {
    Serial.println("Konfigurationsdatei konnte nicht generiert werden.");
    cfgFile.close();
    return -1;
  }
  if (!cfgFile.write(cfgBytes, cfgFileSize))
  {
    Serial.println("Konfigurationsdatei konnte nicht geschrieben werden.");
    cfgFile.close();
    return -1;
  }
  Serial.println("Konfiguration wurde erfolgreich gespeichert.");
  cfgFile.close();
  return 0;
}

int configFromSPIFFS(const char* path)
{
  Serial.print("Konfiguration wird aus SPIFFS gelesen (");
  Serial.print(path);
  Serial.println(").");
  File cfgFile = SPIFFS.open(path, FILE_READ);
  if (!cfgFile || cfgFile.isDirectory())
  {
    Serial.print(path);
    Serial.println(" konnte nicht geöffnet werden.");
    return -1;
  }
  if (cfgFile.size() < cfgFileSize)
  {
    Serial.println("Konfigurationsdatei ist zu klein.");
    cfgFile.close();
    return -1;
  }
  unsigned char cfgBytes[cfgFileSize];
  cfgBytes[0] = 0;
  if (!cfgFile.read(cfgBytes, cfgFileSize))
  {
    Serial.println("Konfigurationsdatei konnte nicht gelesen werden.");
    cfgFile.close();
    return -1;
  }
  if (readConfig(cfgBytes, cfgFileSize) < 0)
  {
    Serial.println("Konfigurationsdatei konnte nicht ausgelesen werden.");
    cfgFile.close();
    return -1;
  }
  Serial.println("Konfiguration wurde erfolgreich gelesen.");
  cfgFile.close();
  return 0;
}




void wiFiStaConnectHandler(WiFiEvent_t, WiFiEventInfo_t)
{
  Serial.println("WiFiStaConnectHandler wurde aufgerufen.");
  if (!(lampeIP != (uint32_t) 0))
  {
    int n = MDNS.queryService("http", "tcp");
    for (int i = 0; i < n; i++)
    {
      if (strncmp("delock-", MDNS.hostname(i).c_str(), 7) == 0)
      {
        lampeIP = MDNS.IP(i);
        Serial.print("Neue IP für Lampe: ");
        Serial.print(MDNS.IP(i));
        Serial.println(".");
        // lampe ausschalten
        HTTPClient webClient;
        webClient.begin(("http://" + lampeIP.toString() + "/cm?cmnd=Power%20off").c_str());
        if (webClient.GET() == 200)
        {
          stateLampe = false;
        }
        webClient.end();
        return;
      }
    }
  }
}

void wiFiStaDisconnectHandler(WiFiEvent_t, WiFiEventInfo_t)
{
  Serial.println("WiFiStaDisconnectHandler wurde aufgerufen.");
  if (lampeIP != (uint32_t) 0)
  {
    int n = MDNS.queryService("http", "tcp");
    for (int i = 0; i < n; i++)
    {
      if (strncmp("delock-", MDNS.hostname(i).c_str(), 7) == 0)
      {
        if (lampeIP != MDNS.IP(i))
        {
          lampeIP = MDNS.IP(i);
          Serial.print("Neue IP für Lampe: ");
          Serial.print(MDNS.IP(i));
          Serial.println(".");
          // lampe ausschalten
          HTTPClient webClient;
          webClient.begin("http://" + lampeIP.toString() + "/cm?cmnd=Power%20off");
          if (webClient.GET() == 200)
          {
            stateLampe = false;
          }
          webClient.end();
        }
        return;
      }
    }
    lampeIP = (uint32_t) 0;
    Serial.println("Lampe hat die Verbindung zum AP unterbrochen.");
  }
}



/* ------------------------------- HTTPserver ------------------------------- */

void httpServerTask(void* param)
{
  Serial.println("Webserver-Task wurde gestartet, Resource-Nodes werden registriert...");
  // create and register resource nodes
  ResourceNode* rootGetNode = new ResourceNode("/", "GET", httpRootGetHandler);
  ResourceNode* configGetNode = new ResourceNode("/config", "GET", httpConfigGetHandler);
  ResourceNode* configPostNode = new ResourceNode("/config", "POST", httpConfigPostHandler);
  ResourceNode* stateGetNode = new ResourceNode("/state", "GET", httpStateGetHandler);
  ResourceNode* lampePostNode = new ResourceNode("/lampe", "POST", httpLampePostHandler);
  ResourceNode* luefterPostNode = new ResourceNode("/luefter", "POST", httpLuefterPostHandler);
  ResourceNode* pumpePostNode = new ResourceNode("/pumpe", "POST", httpPumpePostHandler);
  ResourceNode* error404Node = new ResourceNode("", "GET", httpError404Handler);
  httpServer.registerNode(rootGetNode);
  httpServer.registerNode(configGetNode);
  httpServer.registerNode(configPostNode);
  httpServer.registerNode(stateGetNode);
  httpServer.registerNode(lampePostNode);
  httpServer.registerNode(luefterPostNode);
  httpServer.registerNode(pumpePostNode);
  httpServer.setDefaultNode(error404Node);

  // start and run http server
  Serial.println("Webserver wird gestartet...");
  httpServer.start();
  if (httpServer.isRunning())
  {
    Serial.println("Webserver erfolgreich gestartet.");
    while (true)
    {
      httpServer.loop();
      delay(1);
    }
  }
  Serial.println("Etwas hat nicht funktioniert, Webserver-Task wird beendet.");
}

void httpRootGetHandler(HTTPRequest* request, HTTPResponse* response)
{
  Serial.println("Konfigurationsseite wurde aufgerufen.");
  request->discardRequestBody();
  response->setHeader("Content-Type", "text/html");
  response->println(rootHtml);
}

void httpConfigGetHandler(HTTPRequest* request, HTTPResponse* response)
{
  Serial.println("Aktuelle Konfiguration wurde abgefragt.");
  request->discardRequestBody();
  response->setHeader("Content-Type", "application/json");
  DynamicJsonDocument cfgJson(256);
  if (targetBodenFeucht.valid)
    cfgJson["bf"] = targetBodenFeucht.value;
  else
    cfgJson["bf"] = false;

  if (targetBodenTemp.valid)
    cfgJson["bt"] = targetBodenTemp.value;
  else
    cfgJson["bt"] = false;

  if (targetLuftTemp.valid)
    cfgJson["lt"] = targetLuftTemp.value;
  else
    cfgJson["lt"] = false;

  if (targetLuftFeucht.valid)
    cfgJson["lf"] = targetLuftFeucht.value;
  else
    cfgJson["lf"] = false;

  if (targetSonnenstunden.valid)
    cfgJson["ss"] = targetSonnenstunden.value;
  else
    cfgJson["ss"] = false;

  serializeJson(cfgJson, *response);
}

void httpConfigPostHandler(HTTPRequest* request, HTTPResponse* response)
{
  Serial.println("Neue Konfiguration wurde hochgeladen:");
  char* buffer = (char*) malloc(257);
  int n = request->readBytes((byte*) buffer, 256);
  if (n < 0 || n > 256 || !(request->requestComplete()))
  {
    free(buffer);
    Serial.println("Empfangenes JSON ist zu groß oder konnte nicht gelesen werden.");
    response->setStatusCode(500);
    response->setStatusText("Could not read body, maybe it is too large?");
    return;
  }
  buffer[n] = '\0';
  Serial.println(buffer);
  DynamicJsonDocument cfgJson(256);
  if (deserializeJson(cfgJson, buffer))
  {
    free(buffer);
    Serial.println("Empfangenes JSON konnte nicht deserialisiert werden.");
    response->setStatusCode(500);
    response->setStatusText("Could not parse.");
    return;
  }
  if (cfgJson["bf"].is<float>())
  {
    Serial.println("Empfangenes JSON enthält Bodenfeuchte als float");
    targetBodenFeucht.value = cfgJson["bf"].as<float>();
    targetBodenFeucht.valid = true;
    Serial.print("Sollwert Bodenfeuchte ist ");
    Serial.print(targetBodenFeucht.value);
    Serial.println(".");
  }
  else
    Serial.println("Empfangenes JSON enthält keinen Zahlenwert für Bodenfeuchte.");
  if (cfgJson["bt"].is<float>())
  {
    targetBodenTemp.value = cfgJson["bt"].as<float>();
    targetBodenTemp.valid = true;
    Serial.print("Sollwert Bodentemperatur ist ");
    Serial.print(targetBodenTemp.value);
    Serial.println(".");
  }
  else
    Serial.println("Empfangenes JSON enthält keinen Zahlenwert für Bodentemperatur.");
  if (cfgJson["lf"].is<float>())
  {
    targetLuftFeucht.value = cfgJson["lf"].as<float>();
    targetLuftFeucht.valid = true;
    Serial.print("Sollwert Luftfeuchte ist ");
    Serial.print(targetLuftFeucht.value);
    Serial.println(".");
  }
  else
    Serial.println("Empfangenes JSON enthält keinen Zahlenwert für Luftfeuchte.");
  if (cfgJson["lt"].is<float>())
  {
    targetLuftTemp.value = cfgJson["lt"].as<float>();
    targetLuftTemp.valid = true;
    Serial.print("Sollwert Lufttemperatur ist ");
    Serial.print(targetLuftTemp.value);
    Serial.println(".");
  }
  else
    Serial.println("Empfangenes JSON enthält keinen Zahlenwert für Lufttemperatur.");
  if (cfgJson["ss"].is<float>())
  {
    targetSonnenstunden.value = cfgJson["ss"].as<float>();
    targetSonnenstunden.valid = true;
    Serial.print("Sollwert Sonnenstunden ist ");
    Serial.print(targetSonnenstunden.value);
    Serial.println(".");
  }
  else
    Serial.println("Empfangenes JSON enthält keinen Zahlenwert für Sonnenstunden.");
  free(buffer);
  configToSPIFFS(CONFIG_PATH);
}

void httpStateGetHandler(HTTPRequest* request, HTTPResponse* response)
{
  Serial.println("Aktuelle Sensorwerte wurden abgefragt.");
  request->discardRequestBody();
  response->setHeader("Content-Type", "application/json");
  DynamicJsonDocument cfgJson(256);
  if (currBodenFeucht.valid)
    cfgJson["bf"] = currBodenFeucht.value;
  else
    cfgJson["bf"] = false;

  if (currBodenTemp.valid)
    cfgJson["bt"] = currBodenTemp.value;
  else
    cfgJson["bt"] = false;

  if (currLuftTemp.valid)
    cfgJson["lt"] = currLuftTemp.value;
  else
    cfgJson["lt"] = false;

  if (currLuftFeucht.valid)
    cfgJson["lf"] = currLuftFeucht.value;
  else
    cfgJson["lf"] = false;

  if (currLicht.valid)
    cfgJson["l"] = currLicht.value;
  else
    cfgJson["l"] = false;
  
  cfgJson["la"] = lampeManualOn;
  cfgJson["lu"] = luefterManualOn;
  cfgJson["pu"] = pumpeManualOn;

  serializeJson(cfgJson, *response);
}

void httpLampePostHandler(HTTPRequest* request, HTTPResponse* response)
{
  response->setHeader("Content-Type", "text/html");
  if (request->getContentLength() > 0)
  {
    Serial.println("Lampe wird manuell angeschaltet.");
    lampeManualOn = true;
    response->println("ON");
    if (!stateLampe && lampeIP != (uint32_t) 0)
    {
      HTTPClient webClient;
      webClient.begin(("http://" + lampeIP.toString() + "/cm?cmnd=Power%20On").c_str());
      stateLampe = (webClient.GET() == 200);
      webClient.end();
    }
  }
  else
  {
    Serial.println("Lampe wird manuell ausgeschaltet.");
    lampeManualOn = false;
    response->println("OFF");
    if (!lampeTargetState() && stateLampe && lampeIP != (uint32_t) 0)
    {
      HTTPClient webClient;
      webClient.begin(("http://" + lampeIP.toString() + "/cm?cmnd=Power%20off").c_str());
      stateLampe = !(webClient.GET() == 200);
      webClient.end();
    }
  }
  request->discardRequestBody();
}

void httpLuefterPostHandler(HTTPRequest* request, HTTPResponse* response)
{
  response->setHeader("Content-Type", "text/html");
  if (request->getContentLength() > 0)
  {
    Serial.println("Lüfter wird manuell angeschaltet.");
    luefterManualOn = true;
    response->println("ON");
    if (!stateLuefter)
    {
      digitalWrite(PIN_LUEFTER, HIGH);
      stateLuefter = true;
    }
  }
  else
  {
    Serial.println("Lüfter wird manuell ausgeschaltet.");
    luefterManualOn = false;
    response->println("OFF");
    if (!luefterTargetState() && stateLuefter)
    {
      digitalWrite(PIN_LUEFTER, LOW);
      stateLuefter = false;
    }
  }
  request->discardRequestBody();
}

void httpPumpePostHandler(HTTPRequest* request, HTTPResponse* response)
{
  response->setHeader("Content-Type", "text/html");
  if (request->getContentLength() > 0)
  {
    Serial.println("Pumpe wird manuell angeschaltet.");
    pumpeManualOn = true;
    response->println("ON");
    if (!statePumpe)
    {
      digitalWrite(PIN_PUMPE, HIGH);
      statePumpe = true;
    }
  }
  else
  {
    Serial.println("Pumpe wird manuell ausgeschaltet.");
    pumpeManualOn = false;
    response->println("OFF");
    if (!pumpeTargetState() && statePumpe)
    {
      digitalWrite(PIN_PUMPE, LOW);
      statePumpe = false;
    }
  }
  request->discardRequestBody();
}

void httpError404Handler(HTTPRequest* request, HTTPResponse* response)
{
  request->discardRequestBody();
  response->setStatusCode(404);
  response->setStatusText("Not Found");
  response->setHeader("Content-Type", "text/html");
  response->println(error404Html);
}

const char* error404Html = "<!DOCTYPE html><html><head><title>Not Found</title></head><body>Error 404: Not Found</body></html>";
const char* rootHtml = "<!DOCTYPE html><html><head><meta charset=utf-8><meta name=viewport content=\"width=device-width, initial-scale=1\"><title>" REGLER "</title><script>function setInObj(c,a,b){if(document.getElementById(b).checkValidity()&&document.getElementById(b).value!==\"\"){c[a]=document.getElementById(b).value*1}}function setButtonP(b,a){document.getElementById(b).className=\"bp\";document.getElementById(b).innerHTML=a+\" manuell ausschalten\"}function setButtonN(b,a){document.getElementById(b).className=\"bn\";document.getElementById(b).innerHTML=a+\" manuell anschalten\"}function setConfig(b){var a=JSON.parse(b);if(typeof a.bf===\"number\"){document.getElementById(\"tbf\").value=a.bf}if(typeof a.bt===\"number\"){document.getElementById(\"tbt\").value=a.bt}if(typeof a.lf===\"number\"){document.getElementById(\"tlf\").value=a.lf}if(typeof a.lt===\"number\"){document.getElementById(\"tlt\").value=a.lt}if(typeof a.ss===\"number\"){document.getElementById(\"tss\").value=a.ss}}function setState(b){var a=JSON.parse(b);if(typeof a.bf===\"number\"){document.getElementById(\"cbf\").innerHTML=\"(\"+a.bf.toString()+\"%)\"}if(typeof a.bt===\"number\"){document.getElementById(\"cbt\").innerHTML=\"(\"+a.bt.toFixed(1).toString()+\"°C)\"}if(typeof a.lf===\"number\"){document.getElementById(\"clf\").innerHTML=\"(\"+a.lf+\"%)\"}if(typeof a.lt===\"number\"){document.getElementById(\"clt\").innerHTML=\"(\"+a.lt+\"°C)\"}if(typeof a.l===\"number\"){document.getElementById(\"cl\").innerHTML=\"(\"+a.l+\"%)\"}if(a.la===true){setButtonP(\"la\",\"Lampe\")}else{if(a.la===false){setButtonN(\"la\",\"Lampe\")}}if(a.lu===true){setButtonP(\"lu\",\"Lüfter\")}else{if(a.lu===false){setButtonN(\"lu\",\"Lüfter\")}}if(a.pu===true){setButtonP(\"pu\",\"Pumpe\")}else{if(a.pu===false){setButtonN(\"pu\",\"Pumpe\")}}}function refresh(){var a=new XMLHttpRequest();var b=new XMLHttpRequest();a.onreadystatechange=function(){if(a.readyState==4&&a.status==200){setConfig(a.responseText)}};b.onreadystatechange=function(){if(b.readyState==4&&b.status==200){setState(b.responseText)}};a.open(\"GET\",\"http://\"+window.location.hostname+\"/config\",true);b.open(\"GET\",\"http://\"+window.location.hostname+\"/state\",true);a.send(null);b.send(null)}function submit(){var b={bf:false,bt:false,lt:false,lf:false,ss:false};setInObj(b,\"bf\",\"tbf\");setInObj(b,\"bt\",\"tbt\");setInObj(b,\"lf\",\"tlf\");setInObj(b,\"lt\",\"tlt\");setInObj(b,\"ss\",\"tss\");var a=new XMLHttpRequest();a.open(\"POST\",\"http://\"+window.location.hostname+\"/config\",true);a.setRequestHeader(\"Content-type\",\"application/json\");a.send(JSON.stringify(b));console.log(JSON.stringify(b))}function lampe(){document.getElementById(\"la\").removeAttribute(\"onclick\");var b=\"x\";if(document.getElementById(\"la\").className==\"bp\"){b=null}var a=new XMLHttpRequest();a.onreadystatechange=function(){if(a.readyState==4){document.getElementById(\"la\").onclick=lampe;if(a.responseText.indexOf(\"ON\")!==-1){setButtonP(\"la\",\"Lampe\")}else{if(a.responseText.indexOf(\"OFF\")!==-1){setButtonN(\"la\",\"Lampe\")}}}};a.open(\"POST\",\"http://\"+window.location.hostname+\"/lampe\",true);a.send(b)}function luefter(){document.getElementById(\"lu\").removeAttribute(\"onclick\");var b=\"x\";if(document.getElementById(\"lu\").className==\"bp\"){b=null}var a=new XMLHttpRequest();a.onreadystatechange=function(){if(a.readyState==4){document.getElementById(\"lu\").onclick=luefter;if(a.responseText.indexOf(\"ON\")!==-1){setButtonP(\"lu\",\"Lüfter\")}else{if(a.responseText.indexOf(\"OFF\")!==-1){setButtonN(\"lu\",\"Lüfter\")}}}};a.open(\"POST\",\"http://\"+window.location.hostname+\"/luefter\",true);a.send(b)}function pumpe(){document.getElementById(\"pu\").removeAttribute(\"onclick\");var a=\"x\";if(document.getElementById(\"pu\").className==\"bp\"){a=null}var b=new XMLHttpRequest();b.onreadystatechange=function(){if(b.readyState==4){document.getElementById(\"pu\").onclick=pumpe;if(b.responseText.indexOf(\"ON\")!==-1){setButtonP(\"pu\",\"Pumpe\")}else{if(b.responseText.indexOf(\"OFF\")!==-1){setButtonN(\"pu\",\"Pumpe\")}}}};b.open(\"POST\",\"http://\"+window.location.hostname+\"/pumpe\",true);b.send(a)};</script><style>.b,.bp,.bn{background-color:#e8e8e8;text-align:center;display:block;margin:5px 0 0;width:100%;border-width:0;border-color:black;border-style:solid;border-radius:3px;padding:3px;font-family:Verdana,Geneva,sans-serif;letter-spacing:-0.3px;font-size:11pt}.bp{background-color:red;color:white}.bn{background-color:lightgreen;color:white}h1{text-align:center;font-family:Verdana,Geneva,sans-serif;font-size:20pt;margin:0}.e{margin:7px 5px;height:40px;font-family:Verdana,Geneva,sans-serif;letter-spacing:-0.3px;font-size:11pt}.bl,.br{padding-top:4px}.tl,.bl{text-align:left;float:left;clear:left}.tr,.br{text-align:right;float:right;clear:right}.tl{font-weight:bold}.tr{font-style:italic}.bl{padding-left:6px;width:22%}.br{width:70%}.i{height:inherit;width:90%;float:left;padding:0;padding-right:2px;border-width:0;background-color:#e8e8e8;text-align:inherit;font-family:inherit;font-size:11pt}</style></head><body onload=refresh() style=background-color:#bfb><div style=\"background-color:white;padding:15px 8px 9px;width:300px;margin:auto\"><h1>" REGLER "</h1><div style=margin-top:15px><form><div class=e><div class=tl>Lufttemperatur</div><div class=tr id=clt></div><div class=bl>Sollwert:</div><div class=br><input type=text class=i id=tlt pattern=^\\d*\\.?\\d*$>°C</input></div></div><div class=e><div class=tl>Luftfeuchtigkeit</div><div class=tr id=clf></div><div class=bl>Sollwert:</div><div class=br><input type=text class=i id=tlf pattern=^\\d*\\.?\\d*$>%</input></div></div><div class=e><div class=tl>Bodentemperatur</div><div class=tr id=cbt></div><div class=bl>Sollwert:</div><div class=br><input type=text class=i id=tbt pattern=^\\d*\\.?\\d*$>°C</input></div></div><div class=e><div class=tl>Bodenleitfähigkeit</div><div class=tr id=cbf></div><div class=bl>Sollwert:</div><div class=br><input type=text class=i id=tbf pattern=^\\d*\\.?\\d*$>%</input></div></div><div class=e><div class=tl>Sonnenstunden</div><div class=tr id=cl></div><div class=bl>Sollwert:</div><div class=br><input type=text class=i id=tss pattern=^\\d*\\.?\\d*$>h</input></div></div></form><button type=button class=b onclick=submit() style=margin-top:10px>Speichern</button></div><div style=\"background-color:lightgray;height:1px;margin:10px 1px\"></div><div><button type=button class=b onclick=lampe() id=la>Lampe manuell anschalten</button><button type=button class=b onclick=luefter() id=lu>Lüfter manuell anschalten</button><button type=button class=b onclick=pumpe() id=pu>Pumpe manuell anschalten</button><button type=button class=b onclick=refresh()>Werte neu laden</button></div></div></body></html>";
