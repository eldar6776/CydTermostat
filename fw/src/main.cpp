#include <Arduino.h>
#include <esp32_smartdisplay.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "time.h"
#include <ui/ui.h>

#define ADC_READOUT_PERIOD 2345U  // ntc conversion rate
#define AMBIENT_NTC_RREF 10000U   // 10k NTC value of at 25 degrees
#define AMBIENT_NTC_B_VALUE 3977U // NTC beta parameter
#define AMBIENT_NTC_PULLUP 10000U // 10k pullup resistor

const int pinNTC = 17;    // ntc sensor input
const int pinSTATUS = 11; // heater drive error status
const int pinSTART = 12;  // start/stop heating drive

const int pinDE = 18; // rs485 driver data enable
float setpointTemp;
float ntcTemp;
float offsetTemp;
float hysteresisTemp;
float roomTemp;
bool status_error = false;
bool ntc_error = false;
bool printrd = false;
char printlog[256];
void ADC_Read(void);
void Status(void);
void printLocalTime(void);
float GetTemperature(uint16_t adc_value);

constexpr uint8_t INIT_FLAG = 42; // 😉 "The Hitchhiker's Guide to the Galaxy" (Douglas Adams)
constexpr float_t MIN_TEMP = 10;  // ideal temperatures
constexpr float_t MAX_TEMP = 14;  // for a wine cellar
constexpr uint8_t EEPROM_SIZE = sizeof(uint8_t) + (3 * sizeof(float_t));
constexpr uint8_t ADDR_INIT_FLAG = 0;
constexpr float_t ADDR_SET_TEMP = sizeof(float_t) + sizeof(uint8_t);
constexpr float_t ADDR_NTC_OFFSET = sizeof(uint8_t) + sizeof(float_t) + sizeof(float_t);
constexpr float_t ADDR_HYSTERESIS = sizeof(uint8_t) + sizeof(float_t) + sizeof(float_t) + sizeof(float_t);

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

constexpr char PREAMBLE[] = R"PREAMBLE(

-------------------------------
ESP32 Web Controlled Thermostat
-------------------------------
   © 2020 Stéphane Calderoni
-------------------------------

-------------------------------
     Initialization process
-------------------------------
)PREAMBLE";

constexpr char CLOSING[] = "\n-------------------------------\n";
// Temperature range supported by the thermostat
// ---------------------------------------------

struct TempRange
{
    bool initialized;
    float_t lower;
    float_t upper;
};

TempRange tempRange;
bool readingTemperature; // -> DHT11 LED indicator
uint32_t startRead;      // -> start time of reading
// WiFi credentials
// ----------------

constexpr char WIFI_SSID[] = "WiFi0";
constexpr char WIFI_PASS[] = "23456789";

// Web server listening port
// -------------------------

constexpr uint16_t HTTP_PORT = 80;
AsyncWebServer server(HTTP_PORT); // -> Web server

/**
 * A temperature reading on the sensor will trigger a flash of the LED indicator.
 * It is therefore necessary to store the instant of this reading, in order to
 * later determine when the LED should turn off.
 */

float_t readTemperature()
{
    startRead = millis();
    return roomTemp;
}

void initEEPROM()
{
    Serial.print(F("2. Initializing EEPROM... "));
    if (EEPROM.begin(EEPROM_SIZE))
    {
        // display of the values currently stored in the EEPROM
        Serial.print(F("done\n   -> [ "));
        uint8_t e1 = EEPROM.readByte(ADDR_INIT_FLAG);
        float_t e2 = EEPROM.readFloat(ADDR_SET_TEMP);
        float_t e3 = EEPROM.readFloat(ADDR_NTC_OFFSET);
        float_t e4 = EEPROM.readFloat(ADDR_HYSTERESIS);
        Serial.printf("0x%02x => %u | 0x%02x => %.1f | 0x%02x => %.1f ]\n", ADDR_INIT_FLAG, e1, ADDR_SET_TEMP, e2, ADDR_NTC_OFFSET, e3, ADDR_HYSTERESIS, e4);
    }
    else
    {
        Serial.println("error!");
    }
}

void initTemp()
{
    // the temperature range stored in the EEPROM is read out
    setpointTemp = EEPROM.readFloat(ADDR_SET_TEMP);
    offsetTemp = EEPROM.readFloat(ADDR_NTC_OFFSET);
    hysteresisTemp = EEPROM.readFloat(ADDR_HYSTERESIS);
    // whether these values are to be taken into account
    // (only if they have already been stored in the EEPROM at least once)
    EEPROM.readByte(ADDR_INIT_FLAG) == INIT_FLAG;
    // the temperature range to be taken over by the thermostat is deduced from this:

    Serial.print(F("3. Temperature set to "));
    Serial.printf("[ %.1f°C , %.1f°C , %.1f°C ]\n", setpointTemp, offsetTemp, hysteresisTemp);
}

void saveToEEPROM(void)
{

    EEPROM.writeFloat(ADDR_SET_TEMP, setpointTemp);
    EEPROM.writeFloat(ADDR_NTC_OFFSET, offsetTemp);
    EEPROM.writeFloat(ADDR_HYSTERESIS, hysteresisTemp);
    EEPROM.writeByte(ADDR_INIT_FLAG, INIT_FLAG);
    EEPROM.commit();
    Serial.println(F("-> Has been stored in EEPROM\n"));
}

/**
 * The web user interface will be stored on the ESP32 Flash memory file system
 * as 5 separate files :
 * - index.html  (the interface structure)
 * - index.css   (the graphical layout of the interface)
 * - index.js    (the dynamic interface management program)
 * - D7MR.woff2  (the font used for numeric displays)
 * - favicon.ico (the tiny icon for the browser)
 */

void initSPIFFS()
{
    if (!SPIFFS.begin())
    {
        Serial.println(F("Cannot mount SPIFFS volume..."));
    }
    Serial.println(F("5. SPIFFS volume is mounted"));
}

// WiFi connection initialization
// ------------------------------

/**
 * A connection to the ambient WiFi network is required here to be able to
 * interact with an operator (who will access ESP32 through a web browser).
 */

void initWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("6. Trying to connect to [%s] network ", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.printf("\n7. Connected! => %s\n", WiFi.localIP().toString().c_str());
}

// ----------------------------------------------------------------------------
// HTTP route definition & request processing
// ----------------------------------------------------------------------------

// Processing of the `index.html` template
// ---------------------------------------

/**
 * The HTML page (index.html) that is stored in SPIFFS has generic markers
 * of the form `%TAG%`. This routine is responsible for substituting these
 * markers with the actual values that correspond to them and must be included
 * in the page that is sent to the browser.
 *
 * There are 4 of these markers:
 * - %TEMP%       (the current temperature read by the sensor)
 * - %MIN_TEMP%   (factory setting of the minimum temperature)
 * - %MAX_TEMP%   (Factory setting of the maximum temperature)
 * - %LOWER_TEMP% (the lower limit of the temperature range set by the operator)
 * - %UPPER_TEMP% (the upper limit of the temperature range set by the operator)
 */

String processor(const String &var)
{
    if (var == "TEMP")
    {
        float_t t = readTemperature();
        return isnan(t) ? String("Error") : String(t, 1);
    }
    else if (var == "MIN_TEMP")
    {
        // MIN_TEMP = setpointTemp - roomTemp;
        return String(setpointTemp - roomTemp, 1);
    }
    else if (var == "MAX_TEMP")
    {
        // MAX_TEMP = setpointTemp;
        return String(setpointTemp, 1);
    }
    else if (var == "LOWER_TEMP")
    {
        return String(setpointTemp - roomTemp, 1);
    }
    else if (var == "UPPER_TEMP")
    {
        return String(setpointTemp, 1);
    }

    return String();
}

// Specific treatment of the root page (as a template)
// ---------------------------------------------------

/**
 * When the browser requests access to the main page `index.html`,
 * the server must first replace the generic markers declared above
 * with their respective values.
 */

void onRootRequest(AsyncWebServerRequest *request)
{
    request->send(SPIFFS, "/index.html", "text/html", false, processor);
}

// Method of fallback in case no request could be resolved
// -------------------------------------------------------

void onNotFound(AsyncWebServerRequest *request)
{
    request->send(404);
}

// Sensor temperature reading query manager
// ----------------------------------------

void onTemp(AsyncWebServerRequest *request)
{
    Serial.println(F("Received temperature request\n-> Performs a sensor reading"));
    float_t temp = readTemperature();

    if (isnan(temp))
    {
        Serial.println(F("** Failed to read from DHT sensor!\n"));
        request->send(200, "text/plain", String("Error"));
    }
    else
    {
        // checkForTriggers(temp);
        Serial.print(F("-> DHT sensor readout: "));
        Serial.printf("%.1f°C\n", temp);
        Serial.println(F("-> Sends the data back to the client\n"));
        request->send(200, "text/plain", String(temp));
    }
}

// Factory reset
// -------------

void onReset(AsyncWebServerRequest *request)
{
    char txt[16];
    memset(txt, 0, sizeof(txt));
    // No point in writing in the EEPROM if it's never been done before...
    // if (tempRange.initialized)
    //{
    //    EEPROM.writeByte(ADDR_INIT_FLAG, 0xff);
    //    EEPROM.commit();
    // }

    // tempRange.initialized = false;
    // tempRange.lower = MIN_TEMP;
    // tempRange.upper = MAX_TEMP;

    // Serial.println(F("\nFactory reset\n"));
    // Serial.print(F("-> Temperature range is set to "));
    // Serial.printf("[ %.1f°C , %.1f°C ]\n\n", tempRange.lower, tempRange.upper);

    // Requests are asynchronous and must always be resolved:
    if (setpointTemp >= 10.0)
        setpointTemp -= 1.0;
    lv_slider_set_value(ui_Slider_Speed, setpointTemp, LV_ANIM_OFF);

    int n = sprintf(printlog, "Setpoint %+0.1f", setpointTemp);
    printrd = true;
    int x = (int)setpointTemp;
    n = sprintf(txt, "%d", x);
    lv_label_set_text(ui_Speed_Number_1, txt);
    lv_label_set_text(ui_Speed_Number_2, txt);
    request->send(200);
}

// ESP32 restart request manager
// -----------------------------

void onReboot(AsyncWebServerRequest *request)
{
    char txt[16];
    memset(txt, 0, sizeof(txt));
    if (setpointTemp <= 40.0)
        setpointTemp += 1.0;
    lv_slider_set_value(ui_Slider_Speed, setpointTemp, LV_ANIM_OFF);
    int n = sprintf(printlog, "Setpoint %+0.1f", setpointTemp);
    printrd = true;
    int x = (int)setpointTemp;
    n = sprintf(txt, "%d", x);
    lv_label_set_text(ui_Speed_Number_1, txt);
    lv_label_set_text(ui_Speed_Number_2, txt);
    // Requests are asynchronous and must always be resolved:
    request->send(200);

    // Serial.println(CLOSING);
    // Serial.println(F("Rebooting...\n"));
    // Serial.flush();
    // ESP.restart();
}

// Manager for queries to define the temperature range set by the operator
// -----------------------------------------------------------------------

void onSaveThresholds(AsyncWebServerRequest *request)
{
    if (request->hasParam("lower") && request->hasParam("upper"))
    {
        float_t lower = request->getParam("lower")->value().toFloat();
        float_t upper = request->getParam("upper")->value().toFloat();
        Serial.printf("Temperature range received: [ %.1f°C , %.1f°C ]\n", lower, upper);
        // saveTempRangeToEEPROM(lower, upper);
    }

    // Requests are asynchronous and must always be resolved:
    request->send(200);
}

// Definition of request handlers and server initialization
// --------------------------------------------------------

/**
 * This is where we will define the HTTP routes of the application,
 * as well as the handlers associated with each route.
 */

void initWebServer()
{

    // Routes that simply return one of the files present on SPIFFS:

    /*
     * for some reason, this doesn't work:
     * -> server.serveStatic("/", SPIFFS, "/index.html");
     *
     * but this does:
     * -> server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
     *
     * and this messes up the rendering in the browser:
     * -> server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setTemplateProcessor(processor);
     *
     * So, I prefer to fall back on the classic method:
     */
    server.on("/", onRootRequest);

    server.serveStatic("/index.js", SPIFFS, "/index.js");
    server.serveStatic("/index.css", SPIFFS, "/index.css");
    server.serveStatic("/D7MR.woff2", SPIFFS, "/D7MR.woff2");
    server.serveStatic("/favicon.ico", SPIFFS, "/favicon.ico");

    server.onNotFound(onNotFound);

    // Routes that correspond to dynamic processing by the microcontroller:
    server.on("/temp", onTemp);
    server.on("/reset", onReset);
    server.on("/reboot", onReboot);
    server.on("/savethresholds", onSaveThresholds);

    // Server initialization

    server.begin();
    Serial.println(F("8. Web server started"));
}

// ----------------------------------------------------------------------------
// Temperature handling
// ----------------------------------------------------------------------------

// Sensor reading
// --------------

void setup()
{
    delay(250);
    pinMode(pinNTC, ANALOG);
    pinMode(pinSTATUS, INPUT_PULLUP);
    pinMode(pinSTART, OUTPUT_OPEN_DRAIN);
    digitalWrite(pinSTART, HIGH);
    Serial.begin(115200);
    // Serial.setDebugOutput(true);
    initEEPROM();
    initTemp();
    initSPIFFS();
    initWiFi();
    initWebServer();
    log_i("CPU: %s rev%d, CPU Freq: %d Mhz, %d core(s)", ESP.getChipModel(), ESP.getChipRevision(), getCpuFrequencyMhz(), ESP.getChipCores());
    log_i("Free heap: %d bytes", ESP.getFreeHeap());
    log_i("Free PSRAM: %d bytes", ESP.getPsramSize());
    log_i("SDK version: %s", ESP.getSdkVersion());

    smartdisplay_init();

    auto disp = lv_disp_get_default();
    // lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    // lv_disp_set_rotation(disp, LV_DISP_ROT_180);
    // lv_disp_set_rotation(disp, LV_DISP_ROT_270);

    ui_init();

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
}
bool dir = false;
float duty = 1;
ulong now = 0;

void loop()
{
    Status();
    ADC_Read();
    lv_timer_handler();

    if (millis() - now >= 1000)
    {
        now = millis();
        digitalWrite(pinSTART, !digitalRead(pinSTART));
        printLocalTime();
    }

    if (printrd)
    {
        Serial.println(printlog);
        printrd = false;
    }
}

void printLocalTime(void)
{
    char datetime[64];
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    strftime(datetime, 64, "%A, %B %d %Y", &timeinfo);
    lv_label_set_text(ui_lblDate, datetime);
    strftime(datetime, 10, "%H:%M", &timeinfo);
    lv_label_set_text(ui_lblTime, datetime);
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    Serial.print("Day of week: ");
    Serial.println(&timeinfo, "%A");
    Serial.print("Month: ");
    Serial.println(&timeinfo, "%B");
    Serial.print("Day of Month: ");
    Serial.println(&timeinfo, "%d");
    Serial.print("Year: ");
    Serial.println(&timeinfo, "%Y");
    Serial.print("Hour: ");
    Serial.println(&timeinfo, "%H");
    Serial.print("Hour (12 hour format): ");
    Serial.println(&timeinfo, "%I");
    Serial.print("Minute: ");
    Serial.println(&timeinfo, "%M");
    Serial.print("Second: ");
    Serial.println(&timeinfo, "%S");

    Serial.println("Time variables");
    char timeHour[3];
    strftime(timeHour, 3, "%H", &timeinfo);
    Serial.println(timeHour);
    char timeWeekDay[10];
    strftime(timeWeekDay, 10, "%A", &timeinfo);
    Serial.println(timeWeekDay);
    Serial.println();
}

void Status(void)
{
    if (digitalRead(pinSTATUS) == LOW)
    {
        lv_obj_clear_flag(ui_imgOff, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgOn, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(ui_imgOn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgOff, LV_OBJ_FLAG_HIDDEN);
    }
}

float GetTemperature(uint16_t adc_value)
{
    float temperature;
    float ntc_resistance;
    ntc_resistance = (float)(AMBIENT_NTC_PULLUP * ((4095.0f / (4095.0f - adc_value)) - 1.0f));
    temperature = ((AMBIENT_NTC_B_VALUE * 298.1f) / (AMBIENT_NTC_B_VALUE + (298.1f * log(ntc_resistance / AMBIENT_NTC_RREF))) - 273.1f);
    return (temperature);
}

void ADC_Read(void)
{
    char txt[16] = {0};
    static uint32_t adctmr = 0U;
    static uint32_t sample_cnt = 0U;
    static uint16_t sample_value[10] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    static float adc_calc;

    uint32_t tmp, t;

    if ((millis() - adctmr) >= ADC_READOUT_PERIOD)
    {
        adctmr = millis();
        sample_value[sample_cnt] = analogRead(pinNTC);
        if (++sample_cnt > 9)
            sample_cnt = 0;
        tmp = 0;
        for (t = 0; t < 10; t++)
            tmp += sample_value[t];
        tmp = tmp / 10;
        if ((tmp < 100) || (tmp > 4000))
        {
            if (sample_cnt == 0)
                ntc_error = true;
            ntcTemp = 0;
        }
        else
        {
            if (sample_cnt == 0)
                ntc_error = false;
            ntcTemp = GetTemperature(tmp);
            roomTemp = ntcTemp + offsetTemp;
            lv_slider_set_value(ui_Slider_Battery, roomTemp, LV_ANIM_OFF);
            int n = sprintf(txt, "%+0.1f", roomTemp);
            lv_label_set_text_fmt(ui_valOffsetWithTemperature, txt);
            Serial.print("NTC raw = ");
            Serial.print(tmp);
            Serial.print(" NTC temperature = ");
            Serial.println(roomTemp);
        }
    }
}