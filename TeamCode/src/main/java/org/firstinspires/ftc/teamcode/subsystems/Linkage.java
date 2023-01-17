package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Linkage {

	Servo servo;

	public double minAngleLimit;
	public double maxAngleLimit;

	public double minRotationLimit;
	public double maxRotationLimit;

	public double retractionLength;
	public double extensionLength;

	public double bar1Length;
	public double bar2Length;

	public Linkage( HardwareMap hw ) {
		this( hw, "linkage", false, 0,
				1, 0, 0.3,
				0, 1, 1, 1  );
	}

	public Linkage( HardwareMap hw, String servoName, boolean reversed, double retractionAngleLimit, double extensionAngleLimit, double retractionServoLimit, double extensionServoLimit, double zeroLength, double maxLength, double length1, double length2 ) {
		servo = hw.servo.get( servoName );

		if( reversed )
			servo.setDirection( Servo.Direction.REVERSE );

		minAngleLimit = retractionAngleLimit;
		maxAngleLimit = extensionAngleLimit;

		retractionLength = zeroLength;
		extensionLength = maxLength;

		bar1Length = length1;
		bar2Length = length2;
	}

	public void moveToExtensionAngle( double angle ) {
		if (angle < minAngleLimit )
			angle = minAngleLimit;
		if (angle > maxAngleLimit )
			angle = maxAngleLimit;

		setPosition( angle / 180 );
	}

	public void moveToExtensionDistance( double distance ) {
		setPosition( convertDistanceToAngle( distance ) );
	}

	public void setPosition( double position ) {
		position = Math.min(position, maxRotationLimit);

		servo.setPosition( position );
	}

	/**
	 * converts the desired distance to travel by the slides to the servo angle
	 * @param distance the distance in inches
	 */
	public double convertDistanceToAngle( double distance ) {
		double extensionDistance = Math.min( extensionLength , distance ) - retractionLength;

		return Math.toDegrees( Math.acos( ( Math.pow( bar1Length, 2 ) + Math.pow( extensionDistance, 2 ) - Math.pow( bar2Length, 2 ) ) / (2 * bar1Length * distance) ) );
	}

}