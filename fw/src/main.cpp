#include <Arduino.h>

#include <esp32_smartdisplay.h>
#include <ui/ui.h>

#define ADC_READOUT_PERIOD 2345U  // ntc conversion rate
#define AMBIENT_NTC_RREF 10000U   // 10k NTC value of at 25 degrees
#define AMBIENT_NTC_B_VALUE 3977U // NTC beta parameter
#define AMBIENT_NTC_PULLUP 10000U // 10k pullup resistor
#define WIFI_SSID "WiFi0"
#define WIFI_PASSWORD "23456789"
#define RADIO_URL "http://www.wdr.de/wdrlive/media/einslive.m3u"

const int pinNTC = 17;    // ntc sensor input
const int pinSTATUS = 11; // heater drive error status
const int pinSTART = 12;  // start/stop heating drive

const int pinDE = 18; // rs485 driver data enable

float ntc_temp;
bool status_error = false;
bool ntc_error = false;
void ADC_Read(void);
void Status(void);
float GetTemperature(uint16_t adc_value);

void setup()
{
    delay(250);
    pinMode(pinNTC, ANALOG);
    pinMode(pinSTATUS, INPUT_PULLUP);
    pinMode(pinSTART, OUTPUT_OPEN_DRAIN);
    digitalWrite(pinSTART,HIGH);
    Serial.begin(115200);
    // Serial.setDebugOutput(true);

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

    if(millis()-now >= 100){
        now = millis();
        digitalWrite(pinSTART,!digitalRead(pinSTART));
    }
}

void Status(void)
{
    if(digitalRead(pinSTATUS) == LOW) lv_obj_clear_state(ui_chkONOFF, LV_STATE_CHECKED);
    else lv_obj_add_state(ui_chkONOFF, LV_STATE_CHECKED);
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
            ntc_temp = 0;
        }
        else
        {
            if (sample_cnt == 0)
                ntc_error = false;
            ntc_temp = GetTemperature(tmp);
            lv_slider_set_value(ui_Slider_Battery, ntc_temp, LV_ANIM_OFF);
            Serial.print("NTC raw = ");
            Serial.print(tmp);
            Serial.print(" NTC temperature = ");
            Serial.println(ntc_temp);
        }
    }
}