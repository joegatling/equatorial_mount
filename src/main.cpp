#include <Arduino.h>
#include <FastAccelStepper.h>
#include <SimpleButton.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <HijelHID_BLEKeyboard.h>
#include <ESPmDNS.h>

#define PREFERENCES_NAMESPACE "jg"
#define PREFERENCES_SOUTHERN_HEMISPHERE_KEY "south"
#define PREFERENCES_WIFI_KEY "wifi"
#define PREFERENCES_SSID_KEY "ssid"
#define PREFERENCES_PASSWORD_KEY "pass"
#define PREFERENCES_TRIGGER_INTERVAL_KEY "trig"
#define PREFERENCES_TRACKING_TRIM_PPM_KEY "trim"

enum class Hemisphere {
  Northern,
  Southern
};

enum class Mode
{
  None,
  TrackingNorthern,
  TrackingSouthern,
  SlewingClockwise,
  SlewingCounterClockwise,
  Calibration
};

enum class CalibrationStep
{
  Starting,
  Slew,
  Settle,
  Capture
};

constexpr long HEMISPHERE_DEMO_TIME_MS = 500;
constexpr uint32_t HEMISPHERE_DEMO_SPEED_HZ = 50000;

constexpr uint8_t DIRECTION_PIN = 0;
constexpr uint8_t STEP_PIN = 1;
constexpr float MOTOR_STEPS_PER_REVOLUTION = 200.0; // 1.8 degree step angle
constexpr int MICROSTEPPING = 64;
constexpr float SIDEREAL_DAY_SECONDS = 86164.0905f;

constexpr uint32_t ACCELERATION = 10000;
constexpr uint32_t SLEWING_SPEED_HZ = 20000; // Steps per second for slewing

constexpr uint8_t PIN_ADVANCE_BUTTON = GPIO_NUM_2;
constexpr uint8_t PIN_REVERSE_BUTTON = GPIO_NUM_3;
constexpr uint8_t PIN_HEMISPHERE_BUTTON = GPIO_NUM_10;
constexpr uint8_t PIN_LED = GPIO_NUM_8;

constexpr char SETUP_AP_SSID[] = "Ministar-OTA";
static const char* MDNS_NAME = "ministar";
constexpr char SETUP_AP_PASSWORD[] = "space123";
constexpr char PAGE_STYLE[] =
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<style>"
  "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;margin:0;padding:24px;background:#f6f7fb;color:#1f2937;}"
  "main{max-width:520px;margin:0 auto;background:#fff;border:1px solid #e5e7eb;border-radius:12px;padding:20px;box-shadow:0 8px 24px rgba(17,24,39,.08);}"
  "h1{font-size:1.4rem;margin:0 0 16px;}"
  "h2{font-size:1rem;margin:18px 0 10px;color:#374151;}"
  "label{display:block;font-size:.92rem;font-weight:600;margin:8px 0 4px;}"
  ".radio-option{display:inline-flex;align-items:center;margin:6px 14px 6px 0;font-weight:500;}"
  "input[type='text'],input[type='password'],input[type='number'],input:not([type]){width:100%;box-sizing:border-box;padding:10px 12px;border:1px solid #d1d5db;border-radius:8px;background:#fff;}"
  "input[type='radio']{margin:0 6px 0 0;}"
  "button,input[type='submit']{margin-top:10px;padding:10px 14px;border:0;border-radius:8px;background:#2563eb;color:#fff;font-weight:600;cursor:pointer;}"
  "button:disabled,input[type='submit']:disabled{background:#9ca3af;cursor:not-allowed;}"
  "p{margin:8px 0 0;color:#4b5563;}"
  "</style>";

constexpr double GEAR_RATIO = 114 + (6.0f / 17.0f); // Negative for reverse direction, 114.3529411765 : 1 
constexpr double MICROSTEPS_PER_DAY = MOTOR_STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO;
constexpr uint32_t EARTH_ROTATION_MILLIHZ_PER_STEP = static_cast<uint32_t>(((MICROSTEPS_PER_DAY / SIDEREAL_DAY_SECONDS) * 1000.0) + 0.5); // Millihertz per step for sidereal tracking (1 revolution per 23h56m4s)
constexpr int32_t TRACKING_TRIM_PPM_MIN = -5000;
constexpr int32_t TRACKING_TRIM_PPM_MAX = 5000;

constexpr unsigned long CALIBRATION_DOUBLE_PRESS_TIME_MS = 1000;

// Calibration Timings
constexpr uint32_t CALIBRATION_SLEW_ACCELERATION = 20000;
constexpr uint32_t CALIBRATION_SLEW_SPEED_HZ = 20000;
constexpr float CALIBRATION_SLEW_STEPS = MOTOR_STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO / (360);

constexpr unsigned long CALIBRATION_SETTLE_TIME_MS = 500;
constexpr unsigned long CALIBRATION_CAPTURE_TIME_MS = 500;

constexpr unsigned long CALIBRATION_START_TIME_MS = 2000;


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

Preferences preferences;

SimpleButton slewClockwiseButton(PIN_ADVANCE_BUTTON);
SimpleButton slewCounterClockwiseButton(PIN_REVERSE_BUTTON);
SimpleButton hemisphereButton(PIN_HEMISPHERE_BUTTON);

Mode currentMode = Mode::None;
Hemisphere currentHemisphere = Hemisphere::Northern;
bool isWifiOn = false;

HijelHID_BLEKeyboard keyboard("Ministar Remote", "Joe Gatling");
WebServer webServer(80);
DNSServer dnsServer;

unsigned long currentModeStartTime = 0;
bool trackingDemoMode = false;
int setupDemoIndex = 0;
bool setupAccessPointActive = false;

bool isProvisioning = false;

unsigned long calibrationTimer = 0;
CalibrationStep currentCalibrationStep = CalibrationStep::Slew;
float calibrationMoveError = 0;

bool triggerModeActive = false;
unsigned long lastTriggerTime = 0;
unsigned long triggerInterval = 2000;
int32_t trackingTrimPpm = 0;
uint32_t trackingSpeedMilliHz = EARTH_ROTATION_MILLIHZ_PER_STEP;

uint32_t calculateTrackingSpeedMilliHz(int32_t trimPpm)
{
  int32_t clampedTrimPpm = constrain(trimPpm, TRACKING_TRIM_PPM_MIN, TRACKING_TRIM_PPM_MAX);
  double trimScale = 1.0 + (static_cast<double>(clampedTrimPpm) / 1000000.0);
  double adjustedMilliHz = static_cast<double>(EARTH_ROTATION_MILLIHZ_PER_STEP) * trimScale;

  if (adjustedMilliHz < 1.0)
  {
    return 1;
  }

  return static_cast<uint32_t>(lround(adjustedMilliHz));
}

void setMode(Mode newMode)
{
  if (currentMode != newMode)
  {
    auto previousMode = currentMode;
    currentMode = newMode;
    currentModeStartTime = millis();

    if(previousMode == Mode::Calibration)
    {
      keyboard.end();  
    }

    if(stepper == nullptr)
    {
      return;
    }

    switch (currentMode)
    {
      case Mode::None:
        stepper->stopMove();
        break;
      case Mode::TrackingNorthern:
        if(previousMode == Mode::TrackingSouthern)
        {
          stepper->setSpeedInHz(HEMISPHERE_DEMO_SPEED_HZ);
          stepper->runForward();
          trackingDemoMode = true;
        }
        else
        {
          stepper->setSpeedInMilliHz(trackingSpeedMilliHz);
          stepper->runForward();
        }
        break;
      case Mode::TrackingSouthern:
        if(previousMode == Mode::TrackingNorthern)
        {          
          stepper->setSpeedInHz(HEMISPHERE_DEMO_SPEED_HZ);
          stepper->runBackward();
          trackingDemoMode = true;
        }
        else
        {
          stepper->setSpeedInMilliHz(trackingSpeedMilliHz);
          stepper->runBackward();
        }
        break;
      case Mode::SlewingClockwise:
        stepper->setSpeedInHz(SLEWING_SPEED_HZ);
        stepper->runBackward();
        break;
      case Mode::SlewingCounterClockwise:
        stepper->setSpeedInHz(SLEWING_SPEED_HZ);
        stepper->runForward();
        break;
      case Mode::Calibration:
        stepper->stopMove();
        calibrationTimer = millis();
        currentCalibrationStep = CalibrationStep::Starting;

        keyboard.setLogLevel(HIDLogLevel::Off);
        keyboard.begin();  

        break;
    }
  }
}

bool connectToWiFi()
{  
    preferences.begin(PREFERENCES_NAMESPACE, true);

    String ssid = preferences.getString(PREFERENCES_SSID_KEY, "");
    String pass = preferences.getString(PREFERENCES_PASSWORD_KEY, "");
    
    preferences.end();

    if(ssid.length() == 0)
    {
        Serial.println("No stored WiFi credentials.");
        return false;
    }

    Serial.printf("Connecting to %s\n", ssid.c_str());

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());

    const unsigned long timeoutMs = 8000;
    unsigned long start = millis();

    while(WiFi.status() != WL_CONNECTED)
    {
        delay(250);
        Serial.print(".");

        if(millis() - start > timeoutMs)
        {
            Serial.println("\nWiFi connection failed.");
            return false;
        }
    }

    Serial.println();
    Serial.println("WiFi connected.");
    Serial.println(WiFi.localIP());

    if(MDNS.begin(MDNS_NAME))
    {
        Serial.println("mDNS started.");
    }

    return true;
}


void startProvisioningAP()
{
    isProvisioning = true;

    WiFi.mode(WIFI_AP);

    IPAddress ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(ip, gateway, subnet);
    WiFi.softAP(SETUP_AP_SSID, SETUP_AP_PASSWORD);

    Serial.println("Provisioning AP started.");
    Serial.println(WiFi.softAPIP());
}

void setupButtons()
{
  slewClockwiseButton.SetLongPressDuration(500);
  slewCounterClockwiseButton.SetLongPressDuration(500);
  hemisphereButton.SetLongPressDuration(2000);

  slewClockwiseButton.SetBeginHoldCallback([]() {
    setMode(Mode::SlewingClockwise);
  });

  slewClockwiseButton.SetEndHoldCallback([]() {
    if(currentMode == Mode::SlewingClockwise) 
    {
      if(currentHemisphere == Hemisphere::Northern) 
      {
        setMode(Mode::TrackingNorthern);
      } 
      else 
      {
        setMode(Mode::TrackingSouthern);
      }
    }
  });

  slewCounterClockwiseButton.SetBeginHoldCallback([]() {
    setMode(Mode::SlewingCounterClockwise);
  });

  slewCounterClockwiseButton.SetEndHoldCallback([]() {
    if(currentMode == Mode::SlewingCounterClockwise) 
    {
      if(currentHemisphere == Hemisphere::Northern) 
      {
        
        setMode(Mode::TrackingNorthern);
      } 
      else 
      {
        setMode(Mode::TrackingSouthern);
      }
    }
  });

  slewCounterClockwiseButton.SetEndPressCallback([]() {
    if(currentMode != Mode::Calibration)
    {
      triggerModeActive = !triggerModeActive;

      if(triggerModeActive)
      {
        keyboard.begin();
      }
      else
      {
        keyboard.end();
      }
    }
  });

  hemisphereButton.SetEndPressCallback([]() {

    if(currentMode != Mode::Calibration)
    {
      currentHemisphere = (currentHemisphere == Hemisphere::Northern) ? Hemisphere::Southern : Hemisphere::Northern;
  
      preferences.begin(PREFERENCES_NAMESPACE, false);
      preferences.putBool(PREFERENCES_SOUTHERN_HEMISPHERE_KEY, currentHemisphere == Hemisphere::Southern);
      preferences.end();
    }

    if(millis() - currentModeStartTime < CALIBRATION_DOUBLE_PRESS_TIME_MS)
    {
      setMode(Mode::Calibration);
    }
    else
    {
      if(currentHemisphere == Hemisphere::Northern) 
      {
        setMode(Mode::TrackingNorthern);
      } 
      else 
      {
        setMode(Mode::TrackingSouthern);
      }    
    }
  });

  hemisphereButton.SetBeginHoldCallback([]() 
  {
    isWifiOn = !isWifiOn;

    preferences.begin(PREFERENCES_NAMESPACE, false);
    preferences.putBool(PREFERENCES_WIFI_KEY, isWifiOn);
    preferences.end();

  });
}

void handleHTTP()
{
  webServer.sendHeader("Connection", "close");
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html; charset=UTF-8", "");
}

void setupWebServer()
{
    webServer.on("/", HTTP_GET, []()
    {
        String html;

        preferences.begin(PREFERENCES_NAMESPACE, true);
        html += "<html><head>";
        html += PAGE_STYLE;
        html += "</head><body><main>";
        html += "<h1>Ministar Setup</h1>";
        html += "<h2>WiFi</h2>";
        html += "<form action='/save_wifi' method='POST'>";
        html += "<label for='ssid'>SSID</label>";
        html += "<input name='s' id='ssid' value='" + preferences.getString(PREFERENCES_SSID_KEY, "") + "'><br>";
        html += "<label for='password'>Password</label>";
        html += "<input name='p' id='password' type='password'><br><br>";
        html += "<input id='wifiSaveButton' type='submit' value='Save' disabled>";
        html += "</form>";
        html += "<script>"
          "const passwordInput=document.getElementById('password');"
          "const wifiSaveButton=document.getElementById('wifiSaveButton');"
          "const updateWifiSaveState=()=>{wifiSaveButton.disabled=passwordInput.value.trim().length===0;};"
          "passwordInput.addEventListener('input',updateWifiSaveState);"
          "updateWifiSaveState();"
          "</script>";
        preferences.end();

        html += "<h2>General Setup</h2>";
        html += "<form action='/save_config' method='POST'>";
        html += "<label class='radio-option'><input name='h' type='radio' id='northern' value='northern' " + String(currentHemisphere == Hemisphere::Northern ? "checked" : "") + ">Northern Hemisphere</label><br>";
        html += "<label class='radio-option'><input name='h' type='radio' id='southern' value='southern' " + String(currentHemisphere == Hemisphere::Southern ? "checked" : "") + ">Southern Hemisphere</label><br>";
        html += "<label for='triggerInterval'>Shutter Trigger Interval</label>";
        html += "<input name='t' id='triggerInterval' type='number' min='100' max='30000' step='1' value='" + String(triggerInterval) + "'><br><br>";
        html += "<label for='trackingTrim'>RA Tracking Trim (ppm)</label>";
        html += "<input name='r' id='trackingTrim' type='number' min='" + String(TRACKING_TRIM_PPM_MIN) + "' max='" + String(TRACKING_TRIM_PPM_MAX) + "' step='1' value='" + String(trackingTrimPpm) + "'><p>Use small values to fine-tune RA speed: positive = faster, negative = slower.</p><br>";
        html += "<p>Current RA tracking speed: " + String(trackingSpeedMilliHz) + " mHz (" + String(trackingSpeedMilliHz / 1000.0, 3) + " steps/s), trim " + String(trackingTrimPpm) + " ppm.</p>";
        html += "<input type='submit' value='Save'>";
        html += "</form>";

        html += "</main></body></html>";

        webServer.send(200, "text/html", html);
    });

    webServer.on("/save_wifi", HTTP_POST, []()
    {
        String ssid = webServer.arg("s");
        String pass = webServer.arg("p");

        preferences.begin(PREFERENCES_NAMESPACE, false);
        preferences.putString(PREFERENCES_SSID_KEY, ssid);
        preferences.putString(PREFERENCES_PASSWORD_KEY, pass);
        preferences.end();

        String html = "<html><head>";
        html += "<meta http-equiv='refresh' content='10;url=/'>";
        html += PAGE_STYLE;
        html += "</head><body><main>";
        html += "<h2>Saved. Rebooting...</h2>";
        html += "<p>Your WiFi settings were stored.</p>";
        html += "<p>You will be redirected to the config page in 10 seconds.</p>";
        html += "</main></body></html>";

        webServer.send(200, "text/html", html);

        delay(1000);

        ESP.restart();
    });

    webServer.on("/save_config", HTTP_POST, []()
    {
        String hemisphere = webServer.arg("h");
        bool isSouthern = hemisphere == "southern";
        String triggerIntervalStr = webServer.arg("t");
        triggerInterval = MAX(100, MIN(30000, triggerIntervalStr.toInt()));
        String trackingTrimStr = webServer.arg("r");
        trackingTrimPpm = constrain(trackingTrimStr.toInt(), TRACKING_TRIM_PPM_MIN, TRACKING_TRIM_PPM_MAX);
        trackingSpeedMilliHz = calculateTrackingSpeedMilliHz(trackingTrimPpm);

        preferences.begin(PREFERENCES_NAMESPACE, false);
        preferences.putBool(PREFERENCES_SOUTHERN_HEMISPHERE_KEY, isSouthern);
        preferences.putULong(PREFERENCES_TRIGGER_INTERVAL_KEY, triggerInterval);
        preferences.putInt(PREFERENCES_TRACKING_TRIM_PPM_KEY, trackingTrimPpm);
        preferences.end();

        String html = "<html><head>";
        html += "<meta http-equiv='refresh' content='10;url=/'>";
        html += PAGE_STYLE;
        html += "</head><body><main>";
        html += "<h2>Saved. Rebooting...</h2>";
        html += "<p>Your configuration was stored.</p>";
        html += "<p>You will be redirected to the config page in 10 seconds.</p>";
        html += "</main></body></html>";

        webServer.send(200, "text/html", html);

        delay(1000);

        ESP.restart();
    });    

    
    // Helps macOS quickly realize there is no internet.
    webServer.on("/hotspot-detect.html", HTTP_GET, []()
    {
        webServer.send(204);
    });

    webServer.onNotFound(handleHTTP);    

    webServer.begin();

  // // Handle all requests and respond to prevent macOS from hanging
  // webServer.onNotFound(handleHTTP);
  // webServer.begin();
}

void setupOTA() 
{
    ArduinoOTA.setHostname(MDNS_NAME);
    ArduinoOTA.setPassword(SETUP_AP_PASSWORD);
    
    ArduinoOTA.onStart([]()
    {
        Serial.println("OTA Start");
    });

    ArduinoOTA.onEnd([]()
    {
        Serial.println("\nOTA End");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
    {
        Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error)
    {
        Serial.printf("OTA Error[%u]\n", error);
    });
    
    ArduinoOTA.begin();
}

void startSetupAccessPoint()
{
  if (setupAccessPointActive)
  {
    return;
  }

  if(!connectToWiFi())
  {
    startProvisioningAP();
  }
  
  setupWebServer();
  setupOTA();
  
  setupAccessPointActive = true;
}

void endAccessPoint()
{
  if (!setupAccessPointActive)
  {
    return;
  }

  dnsServer.stop();
  webServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  ArduinoOTA.end();

  setupAccessPointActive = false;
}

void updateLed()
{
  // switch (currentMode)
  // {
  //   case Mode::None:
  //     digitalWrite(PIN_LED, LOW);
  //     break;
  //   case Mode::TrackingNorthern:
  //     digitalWrite(PIN_LED, millis() / 500 % 2);
  //     break;
  //   case Mode::TrackingSouthern:
  //     digitalWrite(PIN_LED, millis() / 1000 % 2);
  //     break;
  //   case Mode::SlewingClockwise:
  //     digitalWrite(PIN_LED, millis() / 50 % 2);
  //     break;
  //   case Mode::SlewingCounterClockwise:
  //     digitalWrite(PIN_LED, millis() / 200 % 2);
  //     break;
  // }

  if(isWifiOn)
  {
    if(WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(PIN_LED, millis() / 500 % 2);
    }
    else
    {
      digitalWrite(PIN_LED, LOW);
    }
  }
  else
  {
    digitalWrite(PIN_LED, HIGH);
  }
}

void updateHemisphereDemo()
{
  if(!trackingDemoMode)
  {
    return;
  }

  if(stepper == nullptr)
  {
    return;
  }

  if(millis() - currentModeStartTime > HEMISPHERE_DEMO_TIME_MS)
  {
    trackingDemoMode = false;
    
    stepper->setSpeedInMilliHz(trackingSpeedMilliHz);

    if(currentMode == Mode::TrackingNorthern)
    {
      stepper->runForward();
    } 
    else if(currentMode == Mode::TrackingSouthern)
    {
      stepper->runBackward();
    }
  }

}

void updateConnection()
{
  if(isWifiOn == false)
  {
    endAccessPoint();
    return;
  }

  startSetupAccessPoint();
  dnsServer.processNextRequest();
  webServer.handleClient();
  ArduinoOTA.handle();
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  preferences.begin(PREFERENCES_NAMESPACE, true);

  bool isSouthernHemisphere = preferences.getBool(PREFERENCES_SOUTHERN_HEMISPHERE_KEY, false);
  currentHemisphere = isSouthernHemisphere ? Hemisphere::Southern : Hemisphere::Northern;
  isWifiOn = preferences.getBool(PREFERENCES_WIFI_KEY, false);
  triggerInterval = preferences.getULong(PREFERENCES_TRIGGER_INTERVAL_KEY, triggerInterval);
  trackingTrimPpm = constrain(preferences.getInt(PREFERENCES_TRACKING_TRIM_PPM_KEY, 0), TRACKING_TRIM_PPM_MIN, TRACKING_TRIM_PPM_MAX);
  trackingSpeedMilliHz = calculateTrackingSpeedMilliHz(trackingTrimPpm);
  
  preferences.end();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper != nullptr) 
  {
      stepper->setDirectionPin(DIRECTION_PIN);        
      stepper->setAcceleration(ACCELERATION);     // steps/s²
  }

  setupButtons();
  setMode(isSouthernHemisphere ? Mode::TrackingSouthern : Mode::TrackingNorthern);

  currentModeStartTime = millis();
  trackingDemoMode = true;
}

void updateCalibration()
{
  if(currentMode != Mode::Calibration)
  {
    return;
  }

  if(stepper == nullptr)
  {
    return;
  }

  if(currentCalibrationStep == CalibrationStep::Starting)
  {
    if(millis() - calibrationTimer > CALIBRATION_START_TIME_MS)
    {
      stepper->setAcceleration(CALIBRATION_SLEW_ACCELERATION);
      stepper->setSpeedInHz(CALIBRATION_SLEW_SPEED_HZ);
      stepper->setCurrentPosition(0);

      float targetPosition = CALIBRATION_SLEW_STEPS;
      uint32_t targetPositionInSteps = round(targetPosition);
      calibrationMoveError = targetPosition - targetPositionInSteps;

      stepper->moveTo(targetPositionInSteps);
      
      currentCalibrationStep = CalibrationStep::Slew;
      calibrationTimer = millis();
    }
  }
  else if(currentCalibrationStep == CalibrationStep::Slew)
  {
    if(stepper->isRunning() == false)
    {
      currentCalibrationStep = CalibrationStep::Settle;
      calibrationTimer = millis();
    }
  }
  else if(currentCalibrationStep == CalibrationStep::Settle)
  {
    if(millis() - calibrationTimer > CALIBRATION_SETTLE_TIME_MS)
    {
      if(keyboard.isConnected())
      {
          keyboard.tap(MEDIA_VOLUME_UP);
      }

      currentCalibrationStep = CalibrationStep::Capture;
      calibrationTimer = millis();
    }
  }
  else if(currentCalibrationStep == CalibrationStep::Capture)
  {
    if(millis() - calibrationTimer > CALIBRATION_CAPTURE_TIME_MS)
    {
      stepper->setAcceleration(CALIBRATION_SLEW_ACCELERATION);
      stepper->setSpeedInHz(CALIBRATION_SLEW_SPEED_HZ);
      
      float targetPosition = stepper->getCurrentPosition() + CALIBRATION_SLEW_STEPS - calibrationMoveError;
      uint32_t targetPositionInSteps = round(targetPosition);
      calibrationMoveError = targetPosition - targetPositionInSteps;

      stepper->moveTo(targetPositionInSteps);
      
      currentCalibrationStep = CalibrationStep::Slew;
      calibrationTimer = millis();
    }
  }
}

void updateTriggerMode()
{
  if(triggerModeActive == false)
  {
    return;
  }

  if(currentMode == Mode::TrackingNorthern || currentMode == Mode::TrackingSouthern)
  {
    if(keyboard.isConnected())
    {
      if(millis() - lastTriggerTime > triggerInterval)
      {
        keyboard.tap(MEDIA_VOLUME_UP);
        lastTriggerTime = millis();
      }
    }
  }
}

unsigned long lastTestTime = 0;

void loop()
{
  slewClockwiseButton.Update();
  slewCounterClockwiseButton.Update();
  hemisphereButton.Update();

  updateLed();
  updateHemisphereDemo();
  updateConnection();
  updateCalibration();
  
  updateTriggerMode();
}