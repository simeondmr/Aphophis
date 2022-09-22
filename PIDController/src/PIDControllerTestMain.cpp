#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include "../include/PIDController.h"

/// @brief Size of the time step
const double dt = 0.01;

/// @brief Friction coefficient, modeled with drag equation: dx/dt^2 = - friction * (dx/dt)^2
const double friction = 0.01;
/// @brief Constant (negative) acceleration coefficient
const double gravity = 10;

/// @brief Minimum allowed control value
const double ddpControlMin = -100;
/// @brief Maximum allowed control value
const double ddpControlMax = 100;

/// @brief Maximum allowed time to test PID convergence
const double tMax = 10;
/// @brief target for PID convergence
const double target = 100;
/// @brief Maximum allowed error for PID convergence
const double eMax = 0.1;
/// @brief Maximum allowed speed error PID convergence
const double deMax = 0.1;

/// @brief Minimum Kp for PID convergence search
const double minKp = 0;
/// @brief Minimum Ki for PID convergence search
const double minKi = 0;
/// @brief Minimum Kd for PID convergence search
const double minKd = 0;
/// @brief Maximum Kp for PID convergence search
const double maxKp = 5;
/// @brief Maximum Ki for PID convergence search
const double maxKi = 5;
/// @brief Maximum Kd for PID convergence search
const double maxKd = 5;
/// @brief Step size for Kp for PID convergence search
const double stepKp = 0.1;
/// @brief Step size for Ki for PID convergence search
const double stepKi = 0.1;
/// @brief Step size for Kd for PID convergence search
const double stepKd = 0.1;

/// @brief The output file with the progress of the system state
/// @details The columns are: time, position, speed, acceleration, and acceleration respectively due to PID, friction and gravity
std::ofstream outputFile("PIDControllerTestOutput.csv");

/// @brief Calculates a time step for the system state and the PID, with optional output to outputFile
/// @details Uses Euler integration
/// @todo Runge-Kutta integration
/// @todo Generic system response
void step(double& p, double& dp, PIDController& pid, bool out = false);

/// @brief Calculates the time for the PID with the given parameters to converge
/// @return The time for the first accepted convergence, or else -1
double timeToTarget(double Kp, double Ki, double Kd);

/// @brief Searches for the fastest converging PID, then creates a .csv file to graph the process.
/// Outputs on stdout the parameters for the best PID found
/// @todo Non-exhaustive search (gradient descent?)
int main()
{
	double KpMin = 0;
	double KiMin = 0;
	double KdMin = 0;
	double tMin = std::numeric_limits<double>::infinity();
	for(double Kp = minKp ; Kp <= maxKp ; Kp += stepKp)
	for(double Ki = minKi ; Ki <= maxKi ; Ki += stepKi)
	for(double Kd = minKd ; Kd <= maxKd ; Kd += stepKd)
	{
		double t = timeToTarget(Kp, Ki, Kd);
		if(t >= 0 && t < tMin)
		{
			KpMin = Kp;
			KiMin = Ki;
			KdMin = Kd;
			tMin = t;
		}
	}
	std::cout << "KpMin, KiMin, KdMin, tMin" << std::endl;
	std::cout << KpMin << ", " << KiMin << ", " << KdMin << ", " << tMin << std::endl;
	
	if(!outputFile.is_open()) return 1;
	outputFile << "t, p, dp, ddp, ddpControl, ddpFriction, ddpGravity" << std::endl;
	double p = 0;
	double dp = 0;
	PIDController pid = PIDController(KpMin, KiMin, KdMin);
	for(double t = 0 ; t < tMax ; t += dt)
	{	
		outputFile << t << ", ";
		step(p, dp, pid, true);
	}
	return 0;
}

void step(double& p, double& dp, PIDController& pid, bool out)
{
	double ddpControl = pid.control(target, p, dt);
	ddpControl = std::clamp(ddpControl, ddpControlMin, ddpControlMax);
	double ddpFriction = -friction * dp * std::abs(dp);
	double ddpGravity = -gravity;
	double ddp = ddpControl + ddpFriction + ddpGravity;
	p += dp * dt;
	dp += ddp * dt;
	if(out) outputFile << p << ", " << dp << ", " << ddp << ", " << ddpControl << ", " << ddpFriction << ", " << ddpGravity << std::endl;
}

double timeToTarget(double Kp, double Ki, double Kd)
{
	PIDController pid = PIDController(Kp, Ki, Kd);
	double p = 0;
	double dp = 0;
	double tTot = -1;
	for(double t = 0 ; t < tMax ; t += dt)
	{
		step(p, dp, pid);
		if(tTot == -1 && std::abs(p - target) <= eMax && std::abs(dp) <= deMax) tTot = t;
	}
	if(std::abs(p - target) <= eMax && std::abs(dp) <= deMax) return tTot;
	return -1;
}