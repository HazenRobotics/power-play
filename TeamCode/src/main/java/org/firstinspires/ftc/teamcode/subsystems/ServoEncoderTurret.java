package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class ServoEncoderTurret {

	CRServo servo;
	DcMotorEx encoder;

	AngleUnit unit;

	double pulsesPerRevolution;
	double gearRatio;

	int turretPosition;

	public enum MovementState {
		REST, MOVING;
	}

	MovementState movementState = MovementState.REST;

	double leftLimit = Double.NEGATIVE_INFINITY;
	double rightLimit = Double.POSITIVE_INFINITY;

	public ServoEncoderTurret( HardwareMap hw ) {
		this( hw, "turret", "encoder", false, false, AngleUnit.RADIANS, 1, 1 );
	}

	public ServoEncoderTurret( HardwareMap hw, String servoName, String encoderName, boolean reverseServo, boolean reverseEncoder, AngleUnit angleUnit, double ppr, double gearRatio ) {
		unit = angleUnit;

		servo = hw.crservo.get( servoName );
		encoder = hw.get( DcMotorEx.class, encoderName );

		servo.setDirection( reverseServo ? Direction.REVERSE : Direction.FORWARD );
		encoder.setDirection( reverseEncoder ? Direction.REVERSE : Direction.FORWARD );
		encoder.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		pulsesPerRevolution = ppr;
		this.gearRatio = gearRatio;

//		resetTurret( );
	}

	public void setPower( double power ) {
		servo.setPower( power );
	}

	public void setLimit( double left, double right ) {
		double multiplyer = unit.equals( AngleUnit.DEGREES ) ? Math.PI / 180 : 1;
		leftLimit = left * multiplyer;
		rightLimit = right * multiplyer;
	}

	public void setLiveRotationPower( Vector2d move ) {

		double tolerance = 0.0;

		double moveX = move.getX( );
		double moveY = move.getY( );

		double target = move.angle( ); // target angle between
		double power = Math.sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) );

		double currentHeading = getHeading( );

		Robot.writeToDefaultFile( tolerance + ", " + move + ", " + target + ", " + currentHeading, true, false );

		if( target < leftLimit || target > rightLimit )
			power *= -1;

		if( target > currentHeading + tolerance )
			servo.setPower( -power );
		else if( target < currentHeading - tolerance )
			servo.setPower( power );
	}

	public void setLivePower( double power ) {
		double pos = getHeading( AngleUnit.RADIANS );

		Robot.writeToDefaultFile( "pos: " + pos, true, false );
		Robot.writeToDefaultFile( "limits: " + leftLimit + ", " + rightLimit, true, false );
		Robot.writeToDefaultFile( "power: " + power, true, false );
//		if( (power > 0  && pos < rightLimit) || (power < 0 && pos > leftLimit) ) {
		servo.setPower( power );

//		} else
//			servo.setPower( 0 );
	}

	/**
	 * @param power    power to turn at
	 * @param rotation the rotation to set the turret to in the predetermined AngleUnit
	 */
	public void setRotation( double power, double rotation ) {
		if( unit.equals( AngleUnit.DEGREES ) )
			rotation *= Math.PI / 180;
		Robot.writeToDefaultFile( "rotation: " + rotation, true, false );
		Robot.writeToDefaultFile( "limits: " + leftLimit + ", " + rightLimit, true, false );
		if( rotation > rightLimit || rotation < leftLimit )
			return;
//		double finalRotation = Drive.convertDistTicks( rotation, 2 * Math.PI, pulsesPerRevolution, gearRatio );
		double finalRotation = Math.round( pulsesPerRevolution * (rotation / (2 * Math.PI)) / gearRatio );
		double finalPower = Math.abs( power )  /* *(getPosition( ) > finalRotation ? 1 : -1)*/;
		Robot.writeToDefaultFile( "rotation stuff: " + "Drive.convertDistTicks( " + rotation + ", " + 2 * Math.PI + ", " + pulsesPerRevolution + ", " + gearRatio + " )", true, false );
		Robot.writeToDefaultFile( "finalRotation: " + finalRotation, true, false );
		Robot.writeToDefaultFile( "finalPower: " + finalPower, true, false );
		servo.setPower( -power );
		new Thread( ( ) -> {
			if( finalPower > 0 )
				while( getPosition( ) < finalRotation )
					Robot.waitTime( 50 );
			else if( finalPower < 0 ) // rotating left to a position less than current
				while( getPosition( ) > finalRotation )
					Robot.waitTime( 50 );

			servo.setPower( 0 );
		} ).start( );
	}

	private double getHeading( AngleUnit angleUnit ) {
		double heading = Drive.convertTicksDist( getPosition( ), 2 * Math.PI, pulsesPerRevolution, gearRatio ); // in radians
		return angleUnit == AngleUnit.DEGREES ? Math.toDegrees( heading ) : heading;
	}

	public double getHeading( ) {
		return getHeading( unit );
	}

	public int getPosition( ) {
		return /*turretPosition +*/ encoder.getCurrentPosition( );
	}

	/**
	 * stops and resets the physical motor and its encoder and sets turretHeading to 0
	 */
	public void resetTurret( ) {
		servo.setPower( 0 );
		encoder.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		turretPosition = 0;
	}

	public MovementState getState( ) {
		return servo.getPower( ) > 0 ? MovementState.MOVING : MovementState.REST;
	}

	/**
	 * adds the current motor position to liftPosition then stops and resets the encoder
	 */
	public void stopAndReset( ) {

		Log.d( "LOGGER", "lift position: " + encoder.getCurrentPosition( ) );
		turretPosition = getPosition( );
		encoder.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		// stop and reset encoder sets the encoder position to zero
	}

	/*
	0, -1.6  :: 2.5
	1.9, .2  :: 3.2
	3.5, 1.7  :: 3.2
	4.9, 2.9  ::
	 */
}
