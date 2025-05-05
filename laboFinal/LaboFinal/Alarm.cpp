#include <Arduino.h>

#include "Alarm.h"

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float& distance)

  : _rPin(rPin), _gPin(gPin), _bPin(bPin), _buzzerPin(buzzerPin), _distance(distance) {


  pinMode(_rPin, OUTPUT);
  pinMode(_gPin, OUTPUT);
  pinMode(_bPin, OUTPUT);
  pinMode(_buzzerPin, OUTPUT);
}

void Alarm::update() {
  _currentTime = millis();

  switch (_state) {
    case OFF:
      _offState();
      break;
    case WATCHING:
      _watchState();
      break;
    case ON:
      _onState();
      break;
    case TESTING:
      _testingState();
      break;
  }
}

void Alarm::setColourA(int r, int g, int b) {

  _colA[0] = r;
  _colA[1] = g;
  _colA[2] = b;
}

void Alarm::setColourB(int r, int g, int b) {

  _colB[0] = r;
  _colB[1] = g;
  _colB[2] = b;
}

void Alarm::setDistance(float d) {

  _distanceTrigger = d;
}

float Alarm::getDistance() {

  return _distanceTrigger;
}

void Alarm::setTimeout(unsigned long ms) {

  _timeoutDelay = ms;
}

void Alarm::turnOff() {

  _turnOff();
}

void Alarm::turnOn() {

  _state = WATCHING;
}

void Alarm::test() {

  _state = TESTING;
}

AlarmState Alarm::getState() const {

  return _state;
}

void Alarm::_setRGB(int r, int g, int b) {

  analogWrite(_rPin, r);
  analogWrite(_gPin, g);
  analogWrite(_bPin, b);
}

void Alarm::_turnOff() {

  noTone(_buzzerPin);
  _setRGB(0, 0, 0);
  _state = OFF;
}

void Alarm::_offState() {

  if (_turnOnFlag) {
    turnOn();
  } else {
    _turnOff();
  }
}

void Alarm::_watchState() {

  if (_distance < _distanceTrigger) {
    _state = ON;
  }


  noTone(_buzzerPin);
  _setRGB(0, 0, 0);
}


void Alarm::_onState() {
  bool transition = _distance >= _distanceTrigger;


  if (transition) {
    if (_currentTime - _lastDetectedTime >= _timeoutDelay) {
      _state = WATCHING;
    }
  } else {
    _lastDetectedTime = _currentTime;
  }


  if (_currentTime - _lastUpdate >= _variationRate) {
    _lastUpdate = _currentTime;
    _currentColor = !_currentColor;
  }
  if (_currentColor) {
    _setRGB(_colB[0], _colB[1], _colB[2]);
  } else {
    _setRGB(_colA[0], _colA[1], _colA[2]);
  }
  tone(_buzzerPin, 1);
}


void Alarm::_testingState() {


  if (_currentTime - _testStartTime >= _timeoutDelay) {
    _testStartTime = _currentTime;
    _turnOff();
  }

  if (_currentTime - _lastUpdate >= _variationRate) {
    _lastUpdate = _currentTime;
    _currentColor = !_currentColor;
  }
  if (_currentColor) {
    _setRGB(_colB[0], _colB[1], _colB[2]);
  } else {
    _setRGB(_colA[0], _colA[1], _colA[2]);
  }
}
