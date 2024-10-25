#include "Button.h"

Button::Button(__uint8_t pin)
{
    buttonPin = pin;
    pinMode(buttonPin, INPUT);
}

__uint8_t Button::getButtonPin()
{
    return buttonPin;
}

bool Button::getButtonState()
{
    buttonState = digitalRead(getButtonPin());

    return buttonState;
}