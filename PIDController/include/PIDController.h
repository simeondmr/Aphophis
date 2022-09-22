/// @brief A basic PID controller
/// @todo Limited control magnitude
/// @todo Limited integral magnitude
/// @todo Tuning methods
/// @todo Derivative based on value instead of error
/// @todo Higher-order finite differences
class PIDController
{
	private:
		/// @brief Current integral of the error
		double iError;
		/// @brief Last error to calculate derivative term
		double lastError;

		/// @brief Coefficient for proportional term
		double Kp;
		/// @brief Coefficient for integral term
		double Ki;
		/// @brief Coefficient for derivative term
		double Kd;

	public:
		/// @brief Creates a new PID controller with the specified parameters
		/// @param Kp Coefficient for proportional term
		/// @param Ki Coefficient for integral term
		/// @param Kd Coefficient for derivative term
		PIDController(double Kp = 0, double Ki = 0, double Kd = 0);
		/// @brief Resets the integral to 0
		/// @return A reference to this for method chaining
		PIDController& resetI();

		double getKp();
		double getKi();
		double getKd();
		/// @return A reference to this for method chaining
		PIDController& setKp(double Kp = 0);
		/// @return A reference to this for method chaining
		PIDController& setKi(double Ki = 0);
		/// @return A reference to this for method chaining
		PIDController& setKd(double Kd = 0);

		/// @brief Calculates the control given the system state
		/// @param target The target to aim for
		/// @param value The current value of the controlled variable
		/// @param dt The size of the time step
		/// @return The control to apply to the system
		double control(double target, double value, double dt = 1);
};