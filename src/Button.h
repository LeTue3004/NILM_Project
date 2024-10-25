#ifndef Button_h
#define Button_h

#include <Arduino.h>

class Button
{
private:
    __uint8_t buttonPin;
    bool buttonState = false;

public:
    Button(__uint8_t pin);
    __uint8_t getButtonPin();
    bool getButtonState();
};

#endif