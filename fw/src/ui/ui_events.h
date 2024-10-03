// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.6
// Project name: termostat_V1

#ifndef _UI_EVENTS_H
#define _UI_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif

void initScreen(lv_event_t * e);
void updateSetpoint(lv_event_t * e);
void pin1click(lv_event_t * e);
void pin2click(lv_event_t * e);
void pin3click(lv_event_t * e);
void pin4click(lv_event_t * e);
void pin5click(lv_event_t * e);
void pin6click(lv_event_t * e);
void pin7click(lv_event_t * e);
void pin8click(lv_event_t * e);
void pin9click(lv_event_t * e);
void pinXclick(lv_event_t * e);
void pin0click(lv_event_t * e);
void pinOKclick(lv_event_t * e);
void updateThermostatState(lv_event_t * e);
void updateSettings(lv_event_t * e);
void updateBacklightHigh(lv_event_t * e);
void updateHysteresis(lv_event_t * e);
void updateSensorOffset(lv_event_t * e);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
