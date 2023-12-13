#include <Wire.h>                                       //I2C needed for sensors
#include "SparkFunMPL3115A2.h"                          //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFun_Si7021_Breakout_Library.h"           //Humidity sensor - Search "SparkFun Si7021" and install from Library Manager
#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h" //Weather meter kit - Search "SparkFun Weather Meter" and install from Library Manager
#include "TinyGPSPlus.h"                                //GPS - Search "TinyGPS++" and install from Library Manager
#include <HardwareSerial.h>

#define RAIN_SENSOR_PIN A8

// Declaracion de pines en el hardware
// Digitales
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;
const byte STAT2 = 8;

// Analogos
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;

// Variables para GPS
int hourUTC = 0;
int minuteUTC = 0;
int secondsUTC = 0;
int currentGPSday = 0;
int currentGPSyear = 0;
int currentGPSmonth = 0;
double latitude = 0;
double longitude = 0;
double longNeg = 0;
uint32_t millionResult[3];

int previousHourUTC = -1;
int previousHourUTC3 = -1;
int previousHourUTC5 = -1;

int hr1 = 0;
int hr2 = 0;
int hr3 = 0;
int hr4 = 0;
int hr5 = 0;

int lastGPShour = -1;
int lastGPSday = -1;
bool state = false;
int lastHour = -1;
int previousHour = -1;
int previousMin = -1;

bool rMin = false;
bool rHrs = true;
bool r3Hrs = false;
bool r5Hrs = false;

char decHour[3];
char decMinute[3];

bool isTimeSynced = false;

// Variables Maximas y minimas
float _MaxHum = 0;
float _MinHum = 9999;
float _MaxTemp = 0;
float _MinTemp = 9999;
float _MaxPress = 0;
float _MinPress = 999999999;
float _MaxWindSpeed = 0;
float _MinWindSpeed = 9999;
float _MaxRain = 0;
float _MinRain = 9999;
float _MaxBatt = 0;
float _MinBatt = 9999;

byte maxHumHour[2];
byte maxHumMin[2];
byte minHumHour[2];
byte minHumMin[2];

byte maxTempHour[2];
byte maxTempMin[2];
byte minTempHour[2];
byte minTempMin[2];

byte maxPressHour[2];
byte maxPressMin[2];
byte minPressHour[2];
byte minPressMin[2];

byte maxWindSpeedHour[2];
byte maxWindSpeedMin[2];
byte minWindSpeedHour[2];
byte minWindSpeedMin[2];

byte maxRainHour[2];
byte maxRainMin[2];
byte minRainHour[2];
byte minRainMin[2];

byte maxBattHour[2];
byte maxBattMin[2];
byte minBattHour[2];
byte minBattMin[2];

byte request[255];

// Variables Globales
long lastSecond;    // tiempo de la ultima lectura
float humidity = 0; // humedad %
float tempf = 0;
float tempC = 0;
float baromin = 30.03; // presion barometrica en inHg
float pressure = 0;

float wind_dir = 0;   // [degrees (Cardinal)]
float wind_speed = 0; // [kph]
float rain = 0;       // [mm]

bool rain_sense = false;

float batt_lvl = 11.8; //[analog value from 0 to 1023]
float light_lvl = 455; //[analog value from 0 to 1023]

MPL3115A2 myPressure;                                     // Create an instance of the pressure sensor
SI7021 myHumidity;                                        // Create an instance of the humidity sensor
SFEWeatherMeterKit myweatherMeterKit(WDIR, WSPEED, RAIN); // Create an instance of the weather meter kit
TinyGPSPlus gps;                                          // Create an instance of the gps

typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} UINT16_t;

typedef union
{
    uint32_t value;
    uint8_t bytes[4];
} UINT32_t;

UINT16_t hum, temp, wDir, wSpeed, battery, maxhum, minhum, maxtemp, mintemp, maxwind, minwind, maxrain, minrain;
UINT32_t press, maxpress, minpress, rCount, lat, lon;

byte sense[20];
byte senseClock[89];
byte frameprofile[14];

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600);   // Serial2 is used for LoRa
    Serial3.begin(115200); // Serial2 is used for GPS

    pinMode(RAIN_SENSOR_PIN, INPUT); // Rain sensor

    pinMode(STAT1, OUTPUT); // Status LED Blue
    pinMode(STAT2, OUTPUT); // Status LED Green

    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP);   // input from wind meters rain gauge sensor

    pinMode(REFERENCE_3V3, INPUT);
    pinMode(LIGHT, INPUT);

    Wire.begin();                    // Join I2C bus
    myPressure.begin();              // Initialize the pressure sensor
    myPressure.setModeBarometer();   // Set to Barometer Mode
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags();   // Enable all three pressure and temp event flags
    myPressure.setModeAltimeter();

    myHumidity.begin();                         // Initialize humidity sensor
    myweatherMeterKit.setADCResolutionBits(10); // Set number of bits of ADC resolution

    myweatherMeterKit.begin(); // Initialize the weather meter kit

    lastSecond = millis();

    Serial.println("Weather Shield online!");
}

void loop()
{
    int rainValue = analogRead(RAIN_SENSOR_PIN);
    Serial.println(rainValue);

    getGps();      // Get GPS data
    calcWeather(); // Get weather data
    // maxandmin();   // Get max and min values
    printWeather();

    if (rainValue < 1020)
    {
        rain_sense = true;
        Serial.println("Lluvia detectada");
    }
    else
    {
        rain_sense = false;
        rain = 0;
        Serial.println("No hay lluvia");
    }

    if (isTimeSynced == true)
    {
        if (hourUTC != lastHour)
        {
            lastHour = hourUTC;
            state = true;
        }
    }

    if (hourUTC == 7)
    {
        if (state == true)
        {
            _MaxHum = 0;
            _MinHum = 9999;
            _MaxTemp = 0;
            _MinTemp = 9999;
            _MaxPress = 0;
            _MinPress = 999999999;
            _MaxWindSpeed = 0;
            _MinWindSpeed = 9999;
            _MaxRain = 0;
            _MinRain = 9999;
            _MaxBatt = 0;
            _MinBatt = 9999;
        }

        state == false;
    }

    if (isTimeSynced == true && rMin == true)
    {
        if (minuteUTC != previousMin)
        {
            frameWithClock();
            previousMin = minuteUTC;
        }
    }
    if (isTimeSynced == true && rHrs == true)
    {
        if (hourUTC != previousHour)
        {
            frameWithClock();
            previousHour = hourUTC;
        }
    }

    while (Serial2.available() > 0) // Read LoRa
    {
        for (int i = 0; i < 32; i++)
        {
            request[i] = Serial2.read();
            Serial.print(request[i], HEX);
            Serial.print(" ");
        }

        Serial.println();

        if (request[0] == 0x4D)
        {
            monitorProfile();
            Serial.println("[PROFILE FRAME SEND]");
        }

        if (request[0] == 0x73)
        {
            if (isTimeSynced == false)
            {
                frameNotClock();
                Serial.println("[NOT CLOCK FRAME SEND]");
            }

            else if (isTimeSynced == true)
            {
                frameWithClock();
                Serial.println("[CLOCK FRAME SEND]");
            }
        }
        if (request[0] == 0x49)
        {
            Serial.println("[MINUTE SENSE ACTIVATED]");
            rHrs = false;
            r3Hrs = false;
            r5Hrs = false;
            rMin = true;
        }
        if (request[0] == 0x01)
        {
            Serial.println("[HOUR SENSE ACTIVATED]");
            rMin = false;
            rHrs = true;
            r3Hrs = false;
            r5Hrs = false;
        }
        if (request[0] == 0x03)
        {
            hr1 = (int)request[1];
            hr2 = (int)request[2];
            hr3 = (int)request[3];

            Serial.println("[3 HOURS SENSE ACTIVATED]");
            Serial.print("Hora 1: ");
            Serial.print(hr1);
            Serial.print(" Hora 2: ");
            Serial.print(hr2);
            Serial.print(" Hora 3: ");
            Serial.println(hr3);

            rMin = false;
            rHrs = false;
            r3Hrs = true;
            r5Hrs = false;
        }
    }
}

void frameWithClock()
{
    if (isTimeSynced == true)
    {
        if (hum.value < _MinHum)
        {
            _MinHum = hum.value;
            minhum.value = _MinHum; // Min Hum

            minHumHour[0] = decHour[1];
            minHumHour[1] = decHour[0];
            minHumMin[0] = decMinute[1];
            minHumMin[1] = decMinute[0];
        }

        if (hum.value > _MaxHum)
        {
            _MaxHum = hum.value;
            maxhum.value = _MaxHum;

            maxHumHour[0] = decHour[1];
            maxHumHour[1] = decHour[0];
            maxHumMin[0] = decMinute[1];
            maxHumMin[1] = decMinute[0];
        }

        if (temp.value < _MinTemp)
        {
            _MinTemp = temp.value;
            mintemp.value = _MinTemp; // Min temp

            minTempHour[0] = decHour[1];
            minTempHour[1] = decHour[0];
            minTempMin[0] = decMinute[1];
            minTempMin[1] = decMinute[0];
        }

        if (temp.value > _MaxTemp)
        {
            _MaxTemp = temp.value;
            maxtemp.value = _MaxTemp; // Max temp

            maxTempHour[0] = decHour[1];
            maxTempHour[1] = decHour[0];
            maxTempMin[0] = decMinute[1];
            maxTempMin[1] = decMinute[0];
        }

        if (press.value / 100 < _MinPress)
        {
            _MinPress = press.value;
            minpress.value = _MinPress; // Min pressure

            minPressHour[0] = decHour[1];
            minPressHour[1] = decHour[0];
            minPressMin[0] = decMinute[1];
            minPressMin[1] = decMinute[0];
        }

        if (press.value / 100 > _MaxPress)
        {
            _MaxPress = press.value;
            maxpress.value = _MaxPress; // Max pressure

            maxPressHour[0] = decHour[1];
            maxPressHour[1] = decHour[0];
            maxPressMin[0] = decMinute[1];
            maxPressMin[1] = decMinute[0];
        }

        if (wSpeed.value < _MinWindSpeed)
        {
            _MinWindSpeed = wSpeed.value;
            minwind.value = _MinWindSpeed; // Min wind speed

            minWindSpeedHour[0] = decHour[1];
            minWindSpeedHour[1] = decHour[0];
            minWindSpeedMin[0] = decMinute[1];
            minWindSpeedMin[1] = decMinute[0];
        }

        if (wSpeed.value > _MaxWindSpeed)
        {
            _MaxWindSpeed = 144;
            maxwind.value = _MaxWindSpeed; // Max wind speed

            maxWindSpeedHour[0] = decHour[1];
            maxWindSpeedHour[1] = decHour[0];
            maxWindSpeedMin[0] = decMinute[1];
            maxWindSpeedMin[1] = decMinute[0];
        }

        if (rCount.value < _MinRain)
        {
            _MinRain = rCount.value;
            minrain.value = _MinRain; // Min rain

            minRainHour[0] = decHour[1];
            minRainHour[1] = decHour[0];
            minRainMin[0] = decMinute[1];
            minRainMin[1] = decMinute[0];
        }

        if (rCount.value > _MaxRain)
        {
            _MaxRain = rCount.value;
            maxrain.value = _MaxRain; // Max rain

            maxRainHour[0] = decHour[1];
            maxRainHour[1] = decHour[0];
            maxRainMin[0] = decMinute[1];
            maxRainMin[1] = decMinute[0];
        }

        Serial.print("[CLOCK]");

        senseClock[0] = 0x07;
        senseClock[1] = 0x31;
        senseClock[2] = 0x02;
        senseClock[3] = 0x74;

        senseClock[4] = hum.bytes[1]; // Humidity
        senseClock[5] = hum.bytes[0];
        senseClock[6] = minhum.bytes[1]; // Min humidity
        senseClock[7] = minhum.bytes[0];
        senseClock[8] = minHumHour[1];
        senseClock[9] = minHumHour[0];
        senseClock[10] = minHumMin[1];
        senseClock[11] = minHumMin[0];
        senseClock[12] = maxhum.bytes[1]; // Max humidity
        senseClock[13] = maxhum.bytes[0];
        senseClock[14] = maxHumHour[1];
        senseClock[15] = maxHumHour[0];
        senseClock[16] = maxHumMin[1];
        senseClock[17] = maxHumMin[0];

        senseClock[18] = temp.bytes[1]; // Temperature
        senseClock[19] = temp.bytes[0];
        senseClock[20] = mintemp.bytes[1]; // Min temp
        senseClock[21] = mintemp.bytes[0];
        senseClock[22] = minTempHour[1];
        senseClock[23] = minTempHour[0];
        senseClock[24] = minTempMin[1];
        senseClock[25] = minTempMin[0];
        senseClock[26] = maxtemp.bytes[1]; // Max temp
        senseClock[27] = maxtemp.bytes[0];
        senseClock[28] = maxTempHour[1];
        senseClock[29] = maxTempHour[0];
        senseClock[30] = maxTempMin[1];
        senseClock[31] = maxTempMin[0];

        senseClock[32] = press.bytes[3]; // Pressure
        senseClock[33] = press.bytes[2];
        senseClock[34] = press.bytes[1];
        senseClock[35] = press.bytes[0];
        senseClock[36] = minpress.bytes[3]; // Min pressure
        senseClock[37] = minpress.bytes[2];
        senseClock[38] = minpress.bytes[1];
        senseClock[39] = minpress.bytes[0];
        senseClock[40] = minPressHour[1];
        senseClock[41] = minPressHour[0];
        senseClock[42] = minPressMin[1];
        senseClock[43] = minPressMin[0];
        senseClock[44] = maxpress.bytes[3]; // Max pressure
        senseClock[45] = maxpress.bytes[2];
        senseClock[46] = maxpress.bytes[1];
        senseClock[47] = maxpress.bytes[0];
        senseClock[48] = maxPressHour[1];
        senseClock[49] = maxPressHour[0];
        senseClock[50] = maxPressMin[1];
        senseClock[51] = maxPressMin[0];

        senseClock[52] = wDir.bytes[1]; // Wind direction
        senseClock[53] = wDir.bytes[0];

        senseClock[54] = wSpeed.bytes[1]; // Wind speed
        senseClock[55] = wSpeed.bytes[0];
        senseClock[56] = minwind.bytes[1]; // Min wind speed
        senseClock[57] = minwind.bytes[0];
        senseClock[58] = minWindSpeedHour[1];
        senseClock[59] = minWindSpeedHour[0];
        senseClock[60] = minWindSpeedMin[1];
        senseClock[61] = minWindSpeedMin[0];
        senseClock[62] = maxwind.bytes[1]; // Max wind speed
        senseClock[63] = maxwind.bytes[0];
        senseClock[64] = maxWindSpeedHour[1];
        senseClock[65] = maxWindSpeedHour[0];
        senseClock[66] = maxWindSpeedMin[1];
        senseClock[67] = maxWindSpeedMin[0];

        senseClock[68] = rain_sense;      // BOOL RAIN SENSOR
        senseClock[69] = rCount.bytes[3]; // Rain
        senseClock[70] = rCount.bytes[2];
        senseClock[71] = rCount.bytes[1];
        senseClock[72] = rCount.bytes[0];  // Corrected line
        senseClock[73] = minrain.bytes[3]; // Min rain
        senseClock[74] = minrain.bytes[2];
        senseClock[75] = minrain.bytes[1];
        senseClock[76] = minrain.bytes[0];
        senseClock[77] = minRainHour[1];
        senseClock[78] = minRainHour[0];
        senseClock[79] = minRainMin[1];
        senseClock[80] = minRainMin[0];
        senseClock[81] = maxrain.bytes[3]; // Max rain
        senseClock[82] = maxrain.bytes[2];
        senseClock[83] = maxrain.bytes[1];
        senseClock[84] = maxrain.bytes[0];
        senseClock[85] = maxRainHour[1];
        senseClock[86] = maxRainHour[0];
        senseClock[87] = maxRainMin[1];
        senseClock[88] = maxRainMin[0];

        for (int i = 0; i < 89; i++)
        {
            Serial2.write(senseClock[i]); // Send frame to LoRa
            Serial.print(senseClock[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        Serial.print("Min WS Hour: ");
        Serial.print(minWindSpeedHour[1], HEX);
        Serial.print(minWindSpeedHour[0], HEX);
        Serial.print(" ");
        Serial.print(minWindSpeedMin[1], HEX);
        Serial.print(minWindSpeedMin[0], HEX);
        Serial.println();
        Serial.print("Max WS Hour: ");
        Serial.print(maxWindSpeedHour[1], HEX);
        Serial.print(maxWindSpeedHour[0], HEX);
        Serial.print(" ");
        Serial.print(maxWindSpeedMin[1], HEX);
        Serial.print(maxWindSpeedMin[0], HEX);
        Serial.println();
    }
}

void frameNotClock()
{
    Serial.print("[NOT CLOCK]");

    sense[0] = 0x07;
    sense[1] = 0x31;
    sense[2] = 0x02;
    sense[3] = 0x73;
    sense[4] = hum.bytes[1];
    sense[5] = hum.bytes[0];
    sense[6] = temp.bytes[1];
    sense[7] = temp.bytes[0];
    sense[8] = press.bytes[3];
    sense[9] = press.bytes[2];
    sense[10] = press.bytes[1];
    sense[11] = press.bytes[0];
    sense[12] = wDir.bytes[1];
    sense[13] = wDir.bytes[0];
    sense[14] = wSpeed.bytes[1];
    sense[15] = wSpeed.bytes[0];
    sense[16] = 0x00; // BOOL RAIN SENSOR
    sense[16] = rCount.bytes[3];
    sense[17] = rCount.bytes[2];
    sense[18] = rCount.bytes[1];
    sense[19] = rCount.bytes[0];

    for (int i = 0; i < sizeof(sense); i++)
    {
        Serial2.write(sense[i]); // Send frame to LoRa
        Serial.print(sense[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void calcWeather()
{
    // Calc GPS
    longNeg = ((longitude)) * (-1);
    millionResult[0] = ((longNeg) * (10000000));
    millionResult[1] = ((gps.location.lat()) * (100000000));

    lat.value = millionResult[0];
    lon.value = millionResult[1];

    // Calc temp/humidity from Si7021 sensor
    humidity = myHumidity.getRH();
    hum.value = humidity * 100;
    tempf = myHumidity.readTempF();
    tempC = (tempf - 32) * 5 / 9; // Convert to Celsius
    temp.value = tempC * 100;

    // Weather Meter Kit
    // Calc Wind
    wind_dir = myweatherMeterKit.getWindDirection();
    wDir.value = wind_dir * 100;

    wind_speed = myweatherMeterKit.getWindSpeed();
    wSpeed.value = wind_speed * 100;
    // Calc Rain
    rain = myweatherMeterKit.getTotalRainfall();
    rCount.value = rain * 100;

    // Calc pressure from MPL3115A2
    pressure = myPressure.readPressure();
    press.value = pressure * 100;

    // Calc battery level
    batt_lvl = get_battery_level();
    battery.value = batt_lvl * 100;
}

void monitorProfile()
{
    if (isTimeSynced)
    {
        Serial.print("[PROFILE MONITOR]");

        frameprofile[0] = 0x07;
        frameprofile[1] = 0x31;
        frameprofile[2] = 0x02;
        frameprofile[3] = 0x4D;
        frameprofile[4] = lat.bytes[3];
        frameprofile[5] = lat.bytes[2];
        frameprofile[6] = lat.bytes[1];
        frameprofile[7] = lat.bytes[0];
        frameprofile[8] = lon.bytes[3];
        frameprofile[9] = lon.bytes[2];
        frameprofile[10] = lon.bytes[1];
        frameprofile[11] = lon.bytes[0];
        frameprofile[12] = battery.bytes[1];
        frameprofile[13] = battery.bytes[0];

        for (int i = 0; i < sizeof(frameprofile); i++)
        {
            Serial2.write(frameprofile[i]); // Send frame to LoRa
            Serial.print(frameprofile[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void printWeather()
{
    Serial.println();
    Serial.print("Humidity: ");
    Serial.print(humidity, 1);
    Serial.print(" %RH, Temp: ");
    Serial.print(tempC, 1);
    Serial.print(" C, Pressure: ");
    Serial.print(pressure, 2);
    Serial.print(" Pa, Wind Direction: ");
    Serial.print(wind_dir, 1);
    Serial.print(" deg, Wind Speed: ");
    Serial.print(wind_speed, 1);
    Serial.print(" kph, Total Rain= ");
    Serial.print(rain, 1);
    Serial.print(" mm, Batt: ");
    Serial.print(batt_lvl, 2);
    Serial.print(" V");
    Serial.println();
    Serial.print("Max Hum: ");
    Serial.print(_MaxHum / 100, 1);
    Serial.print(" %RH, Min Hum: ");
    Serial.print(_MinHum / 100, 1);
    Serial.print(" %RH, Max Temp: ");
    Serial.print(_MaxTemp / 100, 1);
    Serial.print(" C, Min Temp: ");
    Serial.print(_MinTemp / 100, 1);
    Serial.print(" C, Max Press: ");
    Serial.print(_MaxPress / 100, 2);
    Serial.print(" Pa, Min Press: ");
    Serial.print(_MinPress / 100, 2);
    Serial.print(" Pa, Max Wind Speed: ");
    Serial.print(_MaxWindSpeed / 100, 1);
    Serial.print(" kph, Min Wind Speed: ");
    Serial.print(_MinWindSpeed / 100, 1);
    Serial.print(" kph, Max Rain: ");
    Serial.print(_MaxRain / 100, 1);
    Serial.print(" mm, Min Rain: ");
    Serial.print(_MinRain / 100, 1);
    Serial.print(" mm");
    Serial.println();

    Serial.print("Max WS Hour: ");
    Serial.print(maxWindSpeedHour[1], HEX);
    Serial.print(maxWindSpeedHour[0], HEX);
    Serial.print(" Max SWS Min: ");
    Serial.print(maxWindSpeedMin[1], HEX);
    Serial.print(maxWindSpeedMin[0], HEX);
    Serial.println();
}

void getGps(void) // Get data clock for GPS
{

    gps.encode(Serial3.read());

    while (Serial3.available() > 0)

        if (gps.encode(Serial3.read()))

            if (gps.date.isValid() == 1)
            {
                currentGPSyear = gps.date.year();
                currentGPSday = gps.date.day();

                if (currentGPSyear != 2080)
                {
                    isTimeSynced = true;
                    digitalWrite(STAT1, HIGH);
                }
                else
                {
                    isTimeSynced = false;
                    digitalWrite(STAT1, LOW);
                }
            }

    if (gps.time.isValid() == 1)
    {
        hourUTC = gps.time.hour() + 5;
        minuteUTC = gps.time.minute();
        secondsUTC = gps.time.second();
        Serial.print("[RTC]: ");
        Serial.print(hourUTC);
        Serial.print(":");
        Serial.print(minuteUTC);
        Serial.print(":");
        Serial.println(secondsUTC);
    }

    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print("Latitud: ");
    Serial.print(latitude, 6);
    Serial.print(" Longitud: ");
    Serial.println(longitude, 6);

    if (gps.location.isValid() == 1)
    {
        digitalWrite(STAT2, HIGH);
    }
    else
    {
        digitalWrite(STAT2, LOW);
    }

    sprintf(decHour, "%02d", hourUTC);
    sprintf(decMinute, "%02d", minuteUTC);

    smartdelay(1000);
}

static void smartdelay(unsigned long ms) // Smartdelay for GPS
{
    unsigned long start = millis();
    do
    {
        while (Serial3.available())
            gps.encode(Serial3.read());
    } while (millis() - start < ms);
}

/*
void maxandmin()
{
    if (hum.value < _MinHum)
    {
        _MinHum = hum.value;
        minhum.value = _MinHum; // Min Hum

        minHumHour[0] = decHour[1];
        minHumHour[1] = decHour[0];
        minTempMin[0] = decMinute[1];
        minTempMin[1] = decMinute[0];
    }

    if (hum.value > _MaxHum)
    {
        _MaxHum = hum.value;
        maxhum.value = _MaxHum;

        maxHumHour[0] = decHour[1];
        maxHumHour[1] = decHour[0];
        maxHumMin[0] = decMinute[1];
        maxHumMin[1] = decMinute[0];
    }

    if (temp.value < _MinTemp)
    {
        _MinTemp = temp.value;
        mintemp.value = _MinTemp; // Min temp

        minTempHour[0] = decHour[1];
        minTempHour[1] = decHour[0];
        minTempMin[0] = decMinute[1];
        minTempMin[1] = decMinute[0];
    }

    if (temp.value > _MaxTemp)
    {
        _MaxTemp = temp.value;
        maxtemp.value = _MaxTemp; // Max temp

        maxTempHour[0] = decHour[1];
        maxTempHour[1] = decHour[0];
        maxTempMin[0] = decMinute[1];
        maxTempMin[1] = decMinute[0];
    }

    if (press.value / 100 < _MinPress)
    {
        _MinPress = press.value;
        minpress.value = _MinPress; // Min pressure

        minPressHour[0] = decHour[1];
        minPressHour[1] = decHour[0];
        minPressMin[0] = decMinute[1];
        minPressMin[1] = decMinute[0];
    }

    if (press.value / 100 > _MaxPress)
    {
        _MaxPress = press.value;
        maxpress.value = _MaxPress; // Max pressure

        maxPressHour[0] = decHour[1];
        maxPressHour[1] = decHour[0];
        maxPressMin[0] = decMinute[1];
        maxPressMin[1] = decMinute[0];
    }

    if (wSpeed.value < _MinWindSpeed)
    {
        _MinWindSpeed = wSpeed.value;
        minwind.value = _MinWindSpeed; // Min wind speed

        minWindSpeedHour[0] = decHour[1];
        minWindSpeedHour[1] = decHour[0];
        minWindSpeedMin[0] = decMinute[1];
        minWindSpeedMin[1] = decMinute[0];
    }

    if (wSpeed.value > _MaxWindSpeed)
    {
        _MaxWindSpeed = wSpeed.value;
        maxwind.value = _MaxWindSpeed; // Max wind speed

        maxWindSpeedHour[0] = decHour[1];
        maxWindSpeedHour[1] = decHour[0];
        maxWindSpeedMin[0] = decMinute[1];
        maxWindSpeedMin[1] = decMinute[0];
    }

    if (rCount.value < _MinRain)
    {
        _MinRain = rCount.value;
        minrain.value = _MinRain; // Min rain

        minRainHour[0] = decHour[1];
        minRainHour[1] = decHour[0];
        minRainMin[0] = decMinute[1];
        minRainMin[1] = decMinute[0];
    }

    if (rCount.value > _MaxRain)
    {
        _MaxRain = rCount.value;
        maxrain.value = _MaxRain; // Max rain

        maxRainHour[0] = decHour[1];
        maxRainHour[1] = decHour[0];
        maxRainMin[0] = decMinute[1];
        maxRainMin[1] = decMinute[0];
    }
}*/

float get_battery_level()
{

    float operatingVoltage = analogRead(REFERENCE_3V3);

    float rawVoltage = analogRead(BATT);

    operatingVoltage = 3.30 / operatingVoltage; // The reference voltage is 3.3V

    rawVoltage = operatingVoltage * rawVoltage; // Convert the 0 to 1023 int to actual voltage on BATT pin

    rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

    return (rawVoltage);
}
