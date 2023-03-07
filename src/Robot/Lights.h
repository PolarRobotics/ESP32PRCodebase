// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);

#include <FastLED.h>
#define LED_PIN 22
#define NUM_LEDS 39
#define TIME_BETWEEN_TOGGLES 25

class Lights {
private:
    unsigned long lastToggleTime;
    uint8_t currState; // LEDState currState;
    CRGBArray<NUM_LEDS> leds;
    uint8_t iteration;
    bool m_isOffense;
    // int i, updateCount;
public:
    // MUHAMMED ENUM PRAISE BE UPON HIM
    enum LEDState {
        PAIRING,     // Yellow
        PAIRED,      // green then fade out
        OFFENSE,     // blue and green
        DEFENSE,     // green
        TACKLED,     // turn red when tackled
    };
    Lights();
    void setupLEDS();
    void setLEDStatus(LEDState status);
    // void setLEDColor(uint8_t r, uint8_t g, )
    void updateLEDS();
    //   void runLoop(int count);
    void togglePosition();
    int returnStatus();
    void pairState(bool state);
};

// Function Definitions

Lights::Lights() {
    currState = PAIRING;
    m_isOffense = false;
}

void Lights::setupLEDS() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    // FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // Power Failsafe
    // Clears LEDs when code is updated
    FastLED.clear();

    updateLEDS();
    FastLED.setBrightness(110);
}

// To set LED status
void Lights::setLEDStatus(LEDState status) {
    currState = status;
    updateLEDS();
}


// To change LED color
void Lights::updateLEDS() {
    switch (currState) {
    case PAIRING: {
        leds = CRGB::DarkOrange;
        break;
    }
    case PAIRED: {
        leds = CRGB::Blue;
        break;
    }
    case OFFENSE: {
        for(int i = 0; i < 39; i ++){
        if(i % 2 == 0){leds[i] = CRGB::Blue;}
        else{leds[i] = CRGB::Green;}
        }
        break;
    }
    case DEFENSE: {
        leds = CRGB::Green;
        break;
    }
    case TACKLED: {
        leds = CRGB::Red;
        break;
    }
    default: {
        leds = CRGB::Black;
        break;
    }
    }
    FastLED.show();
}

void Lights::togglePosition() {
    // debounce makes sure you cant hold down the button, 
    // i think the ps5 library already does this we probably should check
    if (millis() - lastToggleTime >= TIME_BETWEEN_TOGGLES) {
        if (m_isOffense) {
            setLEDStatus(DEFENSE);
        }
        else {

            setLEDStatus(OFFENSE);
        }
        m_isOffense = !m_isOffense;
        lastToggleTime = millis();
    }
}

int Lights::returnStatus(){
    int status = 0;
    status = currState;
    return status;
}