package org.firstinspires.ftc.teamcode.utils;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AlternativeSwervePIDController {


	double Kp, Ki, Kd, Kv, Ks, integral, error, integralMaxMin;
	ElapsedTime timer;

	public AlternativeSwervePIDController() {
		this(0, 0, 0, 0, 0);
	}

	public AlternativeSwervePIDController( double p, double i, double d, double v, double s ) {
		setValues( p, i, d, v, s );
		timer = new ElapsedTime();
	}

	public void setValues( double p, double i, double d, double v, double s ) {
		Kp = p;
		Ki = i;
		Kd = d;
		Kv = v;
		Ks = s;
		integralMaxMin = 0.15 / Kv;
	}

	public double update( double targetAngle,  double currentAngle, double currentVel ) {

		error = findShortestAngularTravel( targetAngle, currentAngle );

		double targetVel = Kp * error;
		double velError = targetVel - currentVel;
		integral += Ki * velError * timer.seconds();
		integral = Range.clip( integral, -integralMaxMin, integralMaxMin );
		double outputVel = Kd * velError + integral;

		timer.reset();

		return Kv * outputVel + Ks * Math.signum(outputVel);
	}

	public double getError () {
		return error;
	}

	public static double findShortestAngularTravel(double targetAngle, double currentAngle) {
		return ((((targetAngle - currentAngle + Math.PI) % TWO_PI) + TWO_PI) % TWO_PI) - Math.PI;
	}

}
