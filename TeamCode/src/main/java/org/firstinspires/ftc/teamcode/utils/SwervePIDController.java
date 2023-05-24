package org.firstinspires.ftc.teamcode.utils;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwervePIDController {


	double Kp, Ki, Kd, integralSum, lastError, error;
	ElapsedTime timer;

	public SwervePIDController() {
		this(0, 0,0);
	}

	public SwervePIDController( double p, double i, double d) {
		setPID( p, i, d );
		timer = new ElapsedTime();
	}

	public void setPID( double p, double i, double d) {
		Kp = p;
		Ki = i;
		Kd = d;
	}

	public double update( double targetAngle,  double currentAngle ) {
		error = findShortestAngularTravel( targetAngle, currentAngle );

		integralSum += error * timer.seconds();

		double derivative = (error - lastError) / timer.seconds();
		lastError = error;

		timer.reset();

		return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
	}

	public double getError () {
		return error;
	}

	public static double findShortestAngularTravel(double targetAngle, double currentAngle) {
		return ((((targetAngle - currentAngle + Math.PI) % TWO_PI) + TWO_PI) % TWO_PI) - Math.PI;
	}

}
