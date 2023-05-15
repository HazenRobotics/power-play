package org.firstinspires.ftc.teamcode.utils;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwervePDController {


	double Kp, Kd, lastError, error;
	ElapsedTime timer;

	public SwervePDController () {
		this(0,0);
	}

	public SwervePDController (double p, double d) {
		setPD( p,d );
		timer = new ElapsedTime();
	}

	public void setPD (double p, double d) {
		Kp = p;
		Kd = d;
	}

	public double update( double targetAngle,  double currentAngle ) {
		error = findShortestAngularTravel( targetAngle, currentAngle );
		double derivative = (error - lastError) / timer.seconds();
		lastError = error;

		timer.reset();

		return (Kp * error) + (Kd * derivative);
	}

	public double getError () {
		return error;
	}

	public static double findShortestAngularTravel (double joyAngle, double motAngle ) {
		double temp = Math.abs( motAngle - joyAngle ) % Math.PI;
		double distance = temp > Math.PI ? TWO_PI - temp : temp;

		int sign = (joyAngle - motAngle >= 0 && joyAngle - motAngle <= Math.PI) || (joyAngle - motAngle <= -Math.PI && joyAngle - motAngle >= -TWO_PI) ? 1 : -1;

		return distance * sign;
	}


}
