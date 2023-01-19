package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.Drive;

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
		this( hw, "linkage", true,
				90, 7, 0,
				0.36, 0, 14, 7, 8.25 );
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

	public double getServoPos( ) {
		return servo.getPosition( );
	}

	public double getAngle( ) {
		return Drive.normalize( getServoPos( ), retractionServoLimit, extensionServoLimit, retractionAngleLimit, extensionAngleLimit );
	}

	public double getExtensionDistance( ) {
		return Drive.normalize( convertAngleToDistance( getAngle( ) ),
				convertAngleToDistance( retractionAngleLimit ), convertAngleToDistance( extensionAngleLimit ),
				retractionLength, extensionLength );
	}

	public void moveToExtensionDistance( double distance ) {
		setPosition( convertAngleToServoPos( convertDistanceToAngle( distance ) ) );
	}

	public void setPosition( double position ) {
		position = Math.min( position, extensionServoLimit );
		position = Math.max( position, retractionServoLimit );

		servo.setPosition( position );
	}

	public double convertAngleToServoPos( double angle ) {
		return Drive.normalize( angle, retractionAngleLimit, extensionAngleLimit, retractionServoLimit, extensionServoLimit );
	}

	/**
	 * converts the desired distance to travel by the slides to the servo angle
	 *
	 * @param distance the distance in inches
	 */
	public double convertDistanceToAngle( double distance ) {

		distance = Drive.normalize( distance, 0, 14, 4.4, 15.2 );

		if (distance == retractionLength)
			return 90;

		return Math.toDegrees( Math.acos( (square( bar2Length ) - square( bar1Length ) - square( distance )) / (-2 * bar1Length * distance) ) );
	}

	public double convertAngleToDistance( double angle ) {
		angle = Math.toRadians( angle );

//		return Math.sqrt( square( bar1Length ) + square( bar2Length ) - (2 * bar1Length * bar2Length * Math.cos( angle )) );
		return (bar1Length * Math.cos( angle )) + Math.sqrt( square( bar1Length * Math.cos( angle ) ) + (square( bar2Length ) - square( bar1Length )) );
	}

	public double square( double x ) {
		return Math.pow( x, 2 );
	}

}