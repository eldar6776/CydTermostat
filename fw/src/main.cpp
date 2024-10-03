#include <Arduino.h>
#include <esp32_smartdisplay.h>
#include <EEPROM.h>
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
float GetTemperature(uint16_t adc_value);

constexpr uint8_t INIT_FLAG = 42; // 😉 "The Hitchhiker's Guide to the Galaxy" (Douglas Adams)

constexpr uint8_t EEPROM_SIZE = sizeof(uint8_t) + (3 * sizeof(float_t));
constexpr uint8_t ADDR_INIT_FLAG = 0;
constexpr float_t ADDR_SET_TEMP = sizeof(float_t) + sizeof(uint8_t);
constexpr float_t ADDR_NTC_OFFSET = sizeof(uint8_t) + sizeof(float_t) + sizeof(float_t);
constexpr float_t ADDR_HYSTERESIS = sizeof(uint8_t) + sizeof(float_t) + sizeof(float_t) + sizeof(float_t);

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
    }

    if (printrd)
    {
        Serial.println(printlog);
        printrd = false;
    }
}

void Status(void)
{
    if (digitalRead(pinSTATUS) == LOW)
        lv_obj_clear_state(ui_chkONOFF, LV_STATE_CHECKED);
    else
        lv_obj_add_state(ui_chkONOFF, LV_STATE_CHECKED);
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