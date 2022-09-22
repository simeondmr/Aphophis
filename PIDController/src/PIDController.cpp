#include "../include/PIDController.h"

PIDController::PIDController(double Kp, double Ki, double Kd):
Kp(Kp),
Ki(Ki),
Kd(Kd)
{
	this->resetI();
}

PIDController& PIDController::resetI()
{
	this->iError = 0;
	this->lastError = 0;
	return *this;
}

double PIDController::getKp() { return this->Kp; }
double PIDController::getKi() { return this->Ki; }
double PIDController::getKd() { return this->Kd; }
PIDController& PIDController::setKp(double Kp) { this->Kp = Kp; return *this; }
PIDController& PIDController::setKi(double Ki) { this->Ki = Ki; return *this; }
PIDController& PIDController::setKd(double Kd) { this->Kd = Kd; return *this; }

double PIDController::control(double target, double value, double dt)
{
	double error = target - value;
	this->iError += error * dt;
	double dError = (error - this->lastError) / dt;
	this->lastError = error;
	return this->Kp * error + this->Ki * this->iError + this->Kd * dError;
}