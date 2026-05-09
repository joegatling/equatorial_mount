#include <Arduino.h>
#include <FastAccelStepper.h>
#include <SimpleButton.h>
#include <Preferences.h>
#include <WiFiSettings.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>

#define PREFERENCES_NAMESPACE "jg"
#define PREFERENCES_SOUTHERN_HEMISPHERE_KEY "south"

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
  Setup
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

constexpr float GEAR_RATIO = 114 + (6.0f / 17.0f); // Negative for reverse direction, 114.3529411765 : 1 
constexpr uint32_t MICROSTEPS_PER_DAY = MOTOR_STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO;
constexpr uint32_t EARTH_ROTATION_MILLIHZ_PER_STEP = MICROSTEPS_PER_DAY / SIDEREAL_DAY_SECONDS * 1000; // Millihertz per step for sidereal tracking (1 revolution per 23h56m4s)

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

Preferences preferences;

SimpleButton slewClockwiseButton(PIN_ADVANCE_BUTTON);
SimpleButton slewCounterClockwiseButton(PIN_REVERSE_BUTTON);
SimpleButton hemisphereButton(PIN_HEMISPHERE_BUTTON);

Mode currentMode = Mode::None;
Hemisphere currentHemisphere = Hemisphere::Northern;

unsigned long currentModeStartTime = 0;
bool trackingDemoMode = false;
int setupDemoIndex = 0;

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
      case Mode::Setup:
        stepper->stopMove();
        setupDemoIndex = -1;
        break;
    }
  }
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

  hemisphereButton.SetBeginPressCallback([]() {
    currentHemisphere = (currentHemisphere == Hemisphere::Northern) ? Hemisphere::Southern : Hemisphere::Northern;

    preferences.begin(PREFERENCES_NAMESPACE, false);
    preferences.putBool(PREFERENCES_SOUTHERN_HEMISPHERE_KEY, currentHemisphere == Hemisphere::Southern);
    preferences.end();

      if(currentHemisphere == Hemisphere::Northern) 
      {
        setMode(Mode::TrackingNorthern);
      } 
      else 
      {
        setMode(Mode::TrackingSouthern);
      }    
  });

  hemisphereButton.SetLongPressDuration(2000);
  hemisphereButton.SetBeginHoldCallback([]() 
  {
    if(stepper != nullptr)
    {
      stepper->stopMove();
    }

    setMode(Mode::Setup);
    digitalWrite(PIN_LED, HIGH);
  });
}

void setupOTA() 
{
    ArduinoOTA.setHostname(WiFiSettings.hostname.c_str());
    ArduinoOTA.setPassword("space");
    ArduinoOTA.begin();
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

void updateSetupMode()
{
  if(currentMode != Mode::Setup)
  {
    return;
  }

  WiFiSettings.hostname = "Ministar";
  WiFiSettings.secure = false;
  WiFiSettings.portal();  

  // unsigned long index = (millis() - currentModeStartTime) / 500;

  // if(index != setupDemoIndex)
  // {
  //   setupDemoIndex = index;

  //   if(setupDemoIndex == 0)
  //   {
  //     stepper->setAcceleration(40000);
  //     stepper->setSpeedInHz(HEMISPHERE_DEMO_SPEED_HZ);
  //     stepper->runForward();
  //   }
  //   else if(setupDemoIndex == 1)
  //   {
  //     stepper->setAcceleration(40000);
  //     stepper->setSpeedInHz(HEMISPHERE_DEMO_SPEED_HZ);
  //     stepper->runBackward();
  //   }
  //   else if(setupDemoIndex == 2)
  //   {
  //     stepper->stopMove();
  //   }
  //   else if(setupDemoIndex == 3)
  //   {
  //     WiFiSettings.hostname = "Ministar";
  //     WiFiSettings.secure = false;
  //     WiFiSettings.portal();  
  //   }
  // }
}

void setup()
{
  Serial.begin(115200);
  SPIFFS.begin(true);   
  delay(500);

  preferences.begin(PREFERENCES_NAMESPACE, true);
  bool isSouthernHemisphere = preferences.getBool(PREFERENCES_SOUTHERN_HEMISPHERE_KEY, false);
  currentHemisphere = isSouthernHemisphere ? Hemisphere::Southern : Hemisphere::Northern;
  preferences.end();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Set callbacks to start OTA when the portal is active
  WiFiSettings.onPortal = []() 
  {
      setupOTA();
  };

  WiFiSettings.onPortalWaitLoop = []() 
  {
      ArduinoOTA.handle();
  };  

  // Initialize the engine (starts hardware timers)
  engine.init();

  // Connect stepper to step pin
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

void loop()
{
  slewClockwiseButton.Update();
  slewCounterClockwiseButton.Update();
  hemisphereButton.Update();

  updateLed();
  updateHemisphereDemo();
  updateSetupMode();
}