//Configuration file

#define NAME "Heater 1"

//Network settings 
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define MQTT_HOST ""
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""

//Home assistant topics
#define STATUS_TOPIC "home/heater/status"
#define DEBUGT "home/heater/debug"
#define current_temperature_topic "home/heater/temp"
#define action_topic "home/heater/action"
#define mode_state_topic "home/heater/mode"
#define mode_command_topic "home/heater/mode/cmd"
#define temperature_command_topic "home/heater/tempset/cmd"
#define temperature_state_topic "home/heater/tempset"
#define away_mode_command_topic "home/heater/away/cmd"
#define away_mode_state_topic "home/heater/away"

//Advanced settings
#define RELAYPIN 13
#define TEMPPIN 12
#define BUTTONIPIN 16
#define BUTTONDPIN 14
#define BUTTONOPIN 2
#define UPDATETEMPI 5000
//
//Temperature
#define TEMPCHECK 3000
#define TEMPOFFSET 0.4
#define MINTEMP 15
#define MAXTEMP 35

//Sync Settings
#define SYNC_ENABLE 1
#define SYNC_mode_command_topic "home/heater2/mode/cmd"
#define SYNC_away_mode_command_topic "home/heater2/away/cmd"
#define SYNC_temperature_command_topic "home/heater2/tempset/cmd"

