package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalLinkage {

	Servo servo;

	double minRotationLimit;
	double maxRotationLimit;

	double zeroLength;
	double maxLength;

	double bar1Length;
	double bar2Length;

	public HorizontalLinkage( HardwareMap hw ) {
		this( hw, "linkage", 0, 1, 0, 1, 1, 1  );
	}

	public HorizontalLinkage( HardwareMap hw, String servoName, double retraction, double extension, double zeroLength, double maxLength, double length1, double length2 ) {
		servo = hw.servo.get( servoName );

		minRotationLimit = retraction;
		maxRotationLimit = extension;

		this.zeroLength = zeroLength;
		this.maxLength = maxLength;

		bar1Length = length1;
		bar2Length = length2;
	}

	public void moveToExtensionAngle( double angle ) {
		if (angle < minRotationLimit )
			angle = minRotationLimit;
		if (angle > maxRotationLimit )
			angle = maxRotationLimit;

		servo.setPosition( angle / 180 );
	}

	public void moveToExtensionDistance( double distance ) {
		servo.setPosition( convertDistanceToAngle( distance ) );
	}


	/**
	 * converts the desired distance to travel by the slides to the servo angle
	 * @param distance the distance in inches
	 */
	public double convertDistanceToAngle( double distance ) {
		double extensionDistance = Math.min(maxLength - zeroLength, distance - zeroLength);

		return Math.toDegrees( Math.acos( ( Math.pow( bar1Length, 2 ) + Math.pow( extensionDistance, 2 ) - Math.pow( bar2Length, 2 ) ) / (2 * bar1Length * distance) ) );
	}

}