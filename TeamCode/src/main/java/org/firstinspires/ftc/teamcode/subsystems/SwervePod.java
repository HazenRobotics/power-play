package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SwervePod {

	double TICKS_PER_REV;
	DcMotorEx driveMotor, rotateMotor;

	public SwervePod() {

	}

	public double convertTicksToAngle(double ticks) {
		double angle = (ticks % TICKS_PER_REV) * (360.0 / TICKS_PER_REV);
		return angle < 0 ? angle + 360 : angle;
	}

	public double findShortestAngularTravel (double motAngle, double joyAngle ) {
		double temp = Math.abs( joyAngle - motAngle ) % 360;
		double distance = temp > 180 ? 360 - temp : temp;

		int sign = (motAngle - joyAngle >= 0 && motAngle - joyAngle <= 180) || (motAngle - joyAngle <= -180 && motAngle - joyAngle >= -360) ? 1 : -1;

		return distance * sign;
	}

}
