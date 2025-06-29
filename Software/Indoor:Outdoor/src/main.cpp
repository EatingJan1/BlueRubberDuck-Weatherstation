#include "pinout_brd.h"
#include <Matter.h>
#include <Wire.h>


#include <WiFi.h>
const char *ssid = ssid_d;
const char *password = password_d;


#if !defined(co2_messure) or (co2_messure == true)
  #include "LIB/Matter/matter_endpoints/MatterAirQualitySensor.h"
  #include <MHZ19.h>
  #include <SoftwareSerial.h>

  MHZ19 mhz19;
  SoftwareSerial CO2Serial(RX_CO2, TX_CO2);

  MatterAirQualitySensor CO2;
#endif

#if !defined(wind_messure) or (wind_messure == true)
  #include "LIB/Anemometer/MCAnemometer.h"

  Anemometer anemometer = Anemometer(Trigger_1, Echo_1, Trigger_2, Echo_2, Trigger_3, Echo_3, Trigger_4, Echo_4, distance_wind);
#endif

#if !defined(Rain_messure) or (Rain_messure == true)
  #include "LIB/Matter/matter_endpoints/MatterFlowSensor.h"

  MatterFlowSensor RainFlow;

#endif

#if (!defined(Rain_detect) or (Rain_detect == true)) and (defined(Reed1_Pin) or defined(Reed2_Pin))
  #include "LIB/Matter/matter_endpoints/MatterRainSensorSensor.h"

  MatterRainSensor Rain;
#endif

#if (!defined(Rain_detect) or (Rain_detect == true) or (!defined(Rain_messure) or (Rain_messure == true))) and (defined(Reed1_Pin) or defined(Reed2_Pin))
  #include "LIB/Hyetometer/mwippe.h"
  #include "LIB/average_per_time/average_per_time.h"
  #if defined(Reed1_Pin) and defined(Reed2_Pin)
    mwippe rain_reed = mwippe(Reed1_Pin, Reed2_Pin, rainunit);
  #elif defined(Reed1_Pin)
    mwippe rain_reed = mwippe(Reed1_Pin, rainunit);
  #elif defined(Reed2_Pin)
    mwippe rain_reed = mwippe(Reed2_Pin, rainunit);
  #endif
  AveragePerTime perHour;
#endif

#if defined(BME_ADDR) or !(BME_temp == false and BME_hum == false and BME_pres == false)
  #include <bme68xLibrary.h>
  Bme68x bme;

  #if ((!defined(DS_temp) or DS_temp == false) and BME_temp == true) or ((DS_temp == true) and (!defined(BME_temp) or BME_temp == true))
    MatterTemperatureSensor Temp;
  #endif

  #if !(BME_hum == false)
    MatterPressureSensor Pres;
  #endif

  #if !(BME_pres == false)
    MatterHumiditySensor Hum;
  #endif
#endif

#if defined(AL_ADDR) or AL_light == true
  #include <SparkFun_VEML6030_Ambient_Light_Sensor.h>
  #include "LIB/Matter/matter_endpoints/MatterLightSensor.h"
  SparkFun_Ambient_Light light_I2C(AL_ADDR);

  float gain = .125;
  int integrationTime = 800;
  int powMode = 2; // range [1; 4] 

  MatterLightSensor Light;
#endif


#if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
  #include "LIB/LEDAnimation/LEDAnimation.h"
  Adafruit_NeoPixel strip(NUM_PIXELS, WS2812_Pin, NEO_GRB + NEO_KHZ800);
  LEDAnimation Led = LEDAnimation(strip, NUM_PIXELS); 
  MatterOnOffPlugin OnOffPlugin;

  void setaqilight(uint8_t tv)
  {
    if (1 == tv)
    {
      Led.setColor(strip.Color(0, 0, 255));
      Led.setAnimation(AnimationType::Static);
    }
    else if (2 == tv)
    {
      Led.setColor(strip.Color(0, 255, 0));
      Led.setAnimation(AnimationType::Static);
    }
    else if (3 == tv)
    {
      Led.setColor(strip.Color(213, 78, 0));
      Led.setAnimation(AnimationType::Static);
    }
    else if (4 == tv)
    {
      Led.setColor(strip.Color(255, 0, 0));
      Led.setAnimation(AnimationType::Static);
    }
    else if (5 == tv)
    {
      Led.setAnimation(AnimationType::CriticalWarning);
    }
    else
    {
        //Led.setAnimation(AnimationType::None);
    }
  }

  bool setPluginOnOff(bool state) {
  Serial.printf("User Callback :: New Plugin State = %s\r\n", state ? "ON" : "OFF");
  if (state) {

    #if defined(co2_messure) and (co2_messure == true)
    setaqilight(CO2.getAirQuality());
    #endif
  } else {
    Led.setAnimation(AnimationType::None);
  }

  //matterPref.putBool(onOffPrefKey, state);
  return true;
  }
#endif




uint32_t button_time_stamp = 0;                
bool button_state = false;                     
uint32_t time_diff = 0;
const uint32_t decommissioningTimeout = 20000;
const uint32_t calibratetime = 5000;


void setup() {
  Serial.begin(115200);
  
  #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
    Led.setColor(strip.Color(0,0,255));
    Led.setAnimation(AnimationType::StartUp);
  #endif

  delay(1000);

  pinMode(Button_Pin, INPUT);

  WiFi.begin(ssid, password);

  
  // Wait for WiFi connection
  uint32_t counter = 0;
  while (WiFi.status() != WL_CONNECTED) {

    Led.setColor(strip.Color(90, 70, 0));
    Led.setAnimation(AnimationType::Connection);
    // WiFi searching feedback
    Serial.print(".");
    delay(500);
    // adds a new line every 30 seconds
    counter++;
    if (!(counter % 60)) {
      Serial.println();
    }
  }

  Serial.println();


  #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
    Led.setAnimation(AnimationType::None);
    OnOffPlugin.begin(true);
    OnOffPlugin.onChange(setPluginOnOff);
  #endif

  #if defined(co2_messure) and (co2_messure == true)
    CO2.begin();
  #endif

  #if defined(BME_ADDR) and !(BME_temp == false and BME_hum == false and BME_pres == false)
    #if ((!defined(DS_temp) or DS_temp == false) and BME_temp == true) or ((DS_temp == true) and (!defined(BME_temp) or BME_temp == true))
      Temp.begin();
    #endif

    #if !(BME_hum == false)
      Pres.begin();
    #endif

    #if !(BME_pres == false)
      Hum.begin();
    #endif
  #endif
  
  
  #if defined(AL_ADDR) and AL_light == true
    Light.begin();
  #endif
  
  #if !defined(Rain_messure) or (Rain_messure == true)
    RainFlow.begin();
  #endif

  #if (!defined(Rain_detect) or (Rain_detect == true)) and (defined(Reed1_Pin) or defined(Reed2_Pin))
    Rain.begin();
  #endif

  Matter.begin();


  if (!Matter.isDeviceCommissioned()) {
    Serial.println("");
    Serial.print(wsname);
    Serial.println(" is not commissioned yet.");
    Serial.println("Initiate the device discovery in your Matter environment.");
    Serial.println("Commission it to your Matter hub with the Matter Label");
    Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
    
    #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
      Led.setColor(strip.Color(0,0,255));   
      Led.setAnimation(AnimationType::Rotating);
    #endif

    uint32_t timeCount = 0;


    while (!Matter.isDeviceCommissioned()) {
      delay(100);
      if ((timeCount++ % 100) == 0) {
        Serial.print(wsname);
        Serial.println(" not commissioned yet. Waiting for commissioning.");
      }
    }
    
    Serial.print(wsname);
    Serial.println(" is commissioned and connected to Wi-Fi. Ready for use.");
    #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
      Led.setAnimation(AnimationType::None);
    #endif
  }
  if(Matter.isDeviceCommissioned())
  {
    #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
      OnOffPlugin.updateAccessory();  // configure the Plugin based on initial state
    #endif
  }

  #if defined(co2_messure) and (co2_messure == true)
    CO2Serial.begin(9600);
    mhz19.begin(CO2Serial);
    mhz19.autoCalibration(true);
  #endif

  #if defined(SDI) and defined(CLK)
    Wire.begin(SDI, CLK, 100000);
  #endif

  #if defined(BME_ADDR) and !(BME_temp == false and BME_hum == false and BME_pres == false)
    bme.begin(BME_ADDR, Wire);
    
    if(bme.checkStatus())
    {
      if (bme.checkStatus() == BME68X_ERROR)
      {
        Serial.println("Sensor error:" + bme.statusString());
      }
      else if (bme.checkStatus() == BME68X_WARNING)
      {
        Serial.println("Sensor Warning:" + bme.statusString());
      }
    }
    
    bme.setTPH();
    bme.setHeaterProf(300, 100);
  #endif

  #if defined(AL_ADDR) and AL_light == true
    if(light_I2C.begin())
      Serial.println("Ready to sense some light!"); 
    else
      Serial.println("Could not communicate with the sensor!");
    
      light_I2C.setGain(gain);
      light_I2C.setIntegTime(integrationTime);

      light_I2C.setPowSavMode(powMode);
      light_I2C.enablePowSave();
  #endif
}

void loop() {
  static uint32_t timeCounter = 0;

  #if Rain_messure or Rain_detect
    if(rain_reed.runcheckerwipp())
    {
      perHour.add(1);
    }
  #endif

  if (!(timeCounter++ % 10)) { 
    //TODO: For long term use Change Time %300(5min) or %600(10min) or % 900(15min)
    #if defined(BME_ADDR) and !(BME_temp == false and BME_hum == false and BME_pres == false)
      bme68xData data;

      Serial.printf("Current Temperature is %.02f °C\r\n", Temp.getTemperature());
      Serial.printf("Current Humidity is %.02f %\r\n", Hum.getHumidity());
      Serial.printf("Current Pressure is %.02f hPa\r\n", Pres.getPressure());

      bme.setOpMode(BME68X_FORCED_MODE);
      delay(500);
      if (bme.fetchData())
      {
        bme.getData(data);
        Temp.setTemperature(data.temperature);
        Hum.setHumidity(data.humidity);
        Pres.setPressure(data.pressure);  
      }
    #endif

    #if defined(AL_ADDR) and AL_light == true
      Serial.printf("Current Light is %d Lux\r\n", Light.getlight());
      Serial.printf("Sensor Light is %d Lux\r\n", light_I2C.readLight());

      Serial.print("LIGHT: ");
      Serial.println(light_I2C.readLight());

      Light.setlight(light_I2C.readLight());
    #endif
    
    #if defined(co2_messure) and (co2_messure == true)
    
      Serial.printf("Current C02 is %.02f ppm\r\n", CO2.getCO2());
      Serial.printf("Current C02 is %d AQI\r\n", CO2.getAirQuality());

      CO2.setCO2(mhz19.getCO2());

      setaqilight(CO2.getAirQuality());
    #endif
    
    #if (!defined(Rain_detect) or (Rain_detect == true) or (!defined(Rain_messure) or (Rain_messure == true))) and (defined(Reed1_Pin) or defined(Reed2_Pin))
      float m3 = perHour.getSum(3600000UL)/1000;
    #endif

    #if !defined(Rain_detect) or (Rain_detect == true) and (defined(Reed1_Pin) or defined(Reed2_Pin))
      if(m3>0)//TODO: In next Version, check rain from last 15 minuts
      {
        Rain.setRain(1);
      }
      else
      {
        Rain.setRain(0);
      }
    #endif

    #if !defined(Rain_messure) or (Rain_messure == true) and (defined(Reed1_Pin) or defined(Reed2_Pin))
      RainFlow.setFlow(m3);
    #endif

    #if !defined(wind_messure) or (wind_messure == true)
      anemometer.readstate();
      log_i("Angle: %f", anemometer.getangle());
      log_i("Speed: %f", anemometer.getspeed());
      log_i("Guest: %f", anemometer.getgustswind());
    #endif
  }

  if (digitalRead(Button_Pin) == LOW and !button_state) {
    button_time_stamp = millis();
    button_state = true;
  }

  if (digitalRead(Button_Pin) == HIGH and button_state) {
    time_diff = millis() - button_time_stamp;
    #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
      OnOffPlugin.toggle();
    #endif
    button_state = false;
  }

  


  if (time_diff > calibratetime and decommissioningTimeout > time_diff)
  {
    #if defined(co2_messure) and (co2_messure == true)
      Serial.println("Co2 calibrate Mode");
      mhz19.calibrate();
    #endif

    #if !defined(wind_messure) or (wind_messure == true)
      Serial.println("Anemometer calibrate Mode");
      anemometer.calibrate();
    #endif

    #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
      Led.setColor(strip.Color(255,0,0));
      Led.setAnimation(AnimationType::Connection);
    #endif

    button_time_stamp = millis();
    time_diff = 0;
    delay(1000);

    #if defined(NUM_PIXELS) and defined(WS2812_Pin) and !(ledmode == 0)
      Led.setAnimation(AnimationType::None);
    #endif
  }
  
  if (time_diff > decommissioningTimeout)
  {
    Serial.printf("Decommissioning %c. It shall be commissioned again.", wsname); 
    Matter.decommission();
    button_time_stamp = millis();
    time_diff = 0;
  }

  delay(500);
}