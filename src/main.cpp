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
constexpr float SIDEREAL_DAY_SECONDS = 86164.091f;

constexpr uint32_t ACCELERATION = 30000;
constexpr uint32_t SLEWING_SPEED_HZ = 20000; // Steps per second for slewing

constexpr uint8_t PIN_ADVANCE_BUTTON = GPIO_NUM_2;
constexpr uint8_t PIN_REVERSE_BUTTON = GPIO_NUM_3;
constexpr uint8_t PIN_HEMISPHERE_BUTTON = GPIO_NUM_10;
constexpr uint8_t PIN_LED = GPIO_NUM_8;

constexpr char SETUP_AP_SSID[] = "Ministar-OTA";
static const char* MDNS_NAME = "ministar";
constexpr char SETUP_AP_PASSWORD[] = "space123";

constexpr float GEAR_RATIO = 114 + (6.0f / 17.0f); // Negative for reverse direction, 114.3529411765 : 1 
constexpr uint32_t MICROSTEPS_PER_DAY = MOTOR_STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO;
constexpr uint32_t EARTH_ROTATION_MILLIHZ_PER_STEP = MICROSTEPS_PER_DAY / SIDEREAL_DAY_SECONDS * 1000; // Millihertz per step for sidereal tracking (1 revolution per 23h56m4s)

constexpr unsigned long CALIBRATION_DOUBLE_PRESS_TIME_MS = 1000;

// Calibration Timings
constexpr uint32_t CALIBRATION_SLEW_ACCELERATION = 20000;
constexpr uint32_t CALIBRATION_SLEW_SPEED_HZ = 20000;
constexpr float CALIBRATION_SLEW_STEPS = MOTOR_STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO / 100.0f;

constexpr unsigned long CALIBRATION_SETTLE_TIME_MS = 1000;
constexpr unsigned long CALIBRATION_CAPTURE_TIME_MS = 3000;

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

void setMode(Mode newMode)
{
  if (currentMode != newMode)
  {
    auto previousMode = currentMode;
    currentMode = newMode;
    currentModeStartTime = millis();

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
          stepper->setSpeedInMilliHz(EARTH_ROTATION_MILLIHZ_PER_STEP);
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
          stepper->setSpeedInMilliHz(EARTH_ROTATION_MILLIHZ_PER_STEP);
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
    Serial.printf("Password: %s\n", pass.c_str());

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
  slewClockwiseButton.SetBeginPressCallback([]() {
    setMode(Mode::SlewingClockwise);
  });

  slewClockwiseButton.SetEndPressCallback([]() {
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

  slewCounterClockwiseButton.SetBeginPressCallback([]() {
    setMode(Mode::SlewingCounterClockwise);
  });

  slewCounterClockwiseButton.SetEndPressCallback([]() {
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

  hemisphereButton.SetLongPressDuration(2000);
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

        html += "<html><body>";
        html += "<h2>Ministar WiFi Setup</h2>";
        html += "<form action='/save' method='POST'>";
        html += "SSID:<br>";
        html += "<input name='s'><br><br>";
        html += "Password:<br>";
        html += "<input name='p' type='password'><br><br>";
        html += "<input type='submit' value='Save'>";
        html += "</form>";
        html += "</body></html>";

        webServer.send(200, "text/html", html);
    });

    webServer.on("/save", HTTP_POST, []()
    {
        String ssid = webServer.arg("s");
        String pass = webServer.arg("p");

        preferences.begin(PREFERENCES_NAMESPACE, false);
        preferences.putString(PREFERENCES_SSID_KEY, ssid);
        preferences.putString(PREFERENCES_PASSWORD_KEY, pass);
        preferences.end();

        webServer.send(200, "text/html",
            "<html><body>"
            "<h2>Saved. Rebooting...</h2>"
            "</body></html>");

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
    
    stepper->setSpeedInMilliHz(EARTH_ROTATION_MILLIHZ_PER_STEP);

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
  
  preferences.end();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  keyboard.setLogLevel(HIDLogLevel::Normal);
  keyboard.begin();  

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
  

}