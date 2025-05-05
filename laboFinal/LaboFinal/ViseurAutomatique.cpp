#include <Arduino.h>

#include "ViseurAutomatique.h"

#define MOTOR_INTERFACE_TYPE 4


ViseurAutomatique::ViseurAutomatique(int p1, int p2, int p3, int p4, float& distance)

  : _stepper(MOTOR_INTERFACE_TYPE, p1, p3, p2, p4), _distance(distance) {

  _stepper.setMaxSpeed(500);
  _stepper.setSpeed(200);
  _stepper.setAcceleration(200);
}

void ViseurAutomatique::update() {

  _currentTime = millis();

  switch (_etat) {
    case INACTIF:
      _inactifState(_currentTime);
      break;
    case SUIVI:
      _suiviState(_currentTime);
      break;
    case REPOS:
      _reposState(_currentTime);
      break;
  }

  _stepper.run();
}
void ViseurAutomatique::setAngleMin(float angle) {

  _angleMin = angle;
}

void ViseurAutomatique::setAngleMax(float angle) {

  _angleMax = angle;
}

void ViseurAutomatique::setPasParTour(int steps) {

  _stepsPerRev = steps;
}

void ViseurAutomatique::setDistanceMinSuivi(float distanceMin) {

  _distanceMinSuivi = distanceMin;
}

void ViseurAutomatique::setDistanceMaxSuivi(float distanceMax) {

  _distanceMaxSuivi = distanceMax;
}
float ViseurAutomatique::getAngle() const {

  float ratio = (_distance - _distanceMinSuivi) / (_distanceMaxSuivi - _distanceMinSuivi);
  return _angleMin + ratio * (_angleMax - _angleMin);
}

float ViseurAutomatique::getDistanceMinSuivi() {

  return _distanceMinSuivi;
}

float ViseurAutomatique::getDistanceMaxSuivi() {

  return _distanceMaxSuivi;
}

void ViseurAutomatique::activer() {

  _etat = REPOS;
}

float ViseurAutomatique::getMinStep() {

  return _angleMin / 360.0 * _stepsPerRev;
}

float ViseurAutomatique::getMaxStep() {

  return _angleMax / 360.0 * _stepsPerRev;
}

void ViseurAutomatique::desactiver() {

  _etat = INACTIF;
}

const char* ViseurAutomatique::getEtatTexte() const {

  if (_etat == INACTIF) return "INACTIF";
  else if (_etat == SUIVI) return "SUIVI";
  else if (_etat == REPOS) return "REPOS";
}

void ViseurAutomatique::_inactifState(unsigned long cT) {

  _stepper.disableOutputs();
}
void ViseurAutomatique::_suiviState(unsigned long cT) {


  if (_distance < _distanceMinSuivi || _distance > _distanceMaxSuivi) {
    _etat = REPOS;
  }

  int stepsToMove = _angleEnSteps(getAngle());


  if (abs(_stepper.distanceToGo()) == 0.1) {
    _stepper.disableOutputs();
  }
  _stepper.moveTo(stepsToMove);
}

void ViseurAutomatique::_reposState(unsigned long cT) {

  if (_distance < _distanceMinSuivi) {
    float minPosition = map(_distanceMinSuivi, _distanceMinSuivi, _distanceMaxSuivi, getMinStep(), getMaxStep());

    _stepper.moveTo(minPosition);

    if (_stepper.distanceToGo() == 0) {
      _stepper.disableOutputs();
    }
  } else if (_distance > _distanceMaxSuivi) {
    float maxPosition = map(_distanceMaxSuivi, _distanceMinSuivi, _distanceMaxSuivi, getMinStep(), getMaxStep());

    _stepper.moveTo(maxPosition);

    if (_stepper.distanceToGo() == 0) {
      _stepper.disableOutputs();
    }
  } else {
    _etat = SUIVI;
  }
}

long ViseurAutomatique::_angleEnSteps(float angle) const {

  return map(angle, _distanceMinSuivi, _distanceMaxSuivi, getMinStep(), getMaxStep());
}
