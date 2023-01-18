package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Linkage {

	Servo servo;

	public double retractionAngleLimit;
	public double extensionAngleLimit;

	public double retractionServoLimit;
	public double extensionServoLimit;

	public double retractionLength;
	public double extensionLength;

	public double bar1Length;
	public double bar2Length;

	public Linkage( HardwareMap hw ) {
		this( hw, "linkage", false, 0,
				1, 0, 0.3,
				0, 1, 1, 1  );
	}

	public Linkage( HardwareMap hw, String servoName, boolean reversed, double retractionAngleLimit,
					double extensionAngleLimit, double retractionServoLimit, double extensionServoLimit,
					double retractionLength, double extensionLength, double bar1Length, double bar2Length ) {
		servo = hw.servo.get( servoName );

		if( reversed )
			servo.setDirection( Servo.Direction.REVERSE );

		this.retractionAngleLimit = retractionAngleLimit;
		this.extensionAngleLimit = extensionAngleLimit;

		this.retractionServoLimit = retractionServoLimit;
		this.extensionServoLimit = extensionServoLimit;

		this.retractionLength = retractionLength;
		this.extensionLength = extensionLength;

		this.bar1Length = bar1Length;
		this.bar2Length = bar2Length;
	}

	public void moveToExtensionAngle( double angle ) {
		if (angle < retractionAngleLimit )
			angle = retractionAngleLimit;
		if (angle > extensionAngleLimit )
			angle = extensionAngleLimit;

		setPosition( angle / 180 );
	}

	public void moveToExtensionDistance( double distance ) {
		setPosition( convertDistanceToAngle( distance ) );
	}

	public void setPosition( double position ) {
		position = Math.min(position, extensionServoLimit );

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