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
		this( hw, "turret", "encoder", false, AngleUnit.RADIANS, 1, 1 );
	}

	public ServoEncoderTurret( HardwareMap hw, String servoName, String encoderName, boolean reverseServo, AngleUnit angleUnit, double ppr, double gearRatio ) {
		unit = angleUnit;

		servo = hw.crservo.get( servoName );
		encoder = hw.get( DcMotorEx.class, encoderName );

		servo.setDirection( reverseServo ? Direction.REVERSE : Direction.FORWARD );

		pulsesPerRevolution = ppr;
		this.gearRatio = gearRatio;

//		resetTurret( );
	}

	public void setPower( double power ) {
		servo.setPower( power );
	}

	public void setLimit( double left, double right ) {
		double multiplyer = 1;
		if( unit.equals( AngleUnit.DEGREES ) )
			multiplyer = Math.PI / 180;
		leftLimit = left * multiplyer;
		rightLimit = right * multiplyer;
	}

	/**
	 * @param power    power to turn at
	 * @param position the rotation of the turret in the predetermined AngleUnit
	 */
	public void setRotationPower( double power, double position ) {
		setRotationPower( power, position, unit );
	}

	public void setLiveRotationPower( Vector2d move ) {
		
		double tolerance = 0.0;

		double moveX = move.getX( );
		double moveY = move.getY( );

		double target = move.angle( ); // target angle between
		double power = Math.sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) );

		double currentHeading = getTurretHeading( );

		Robot.writeToDefaultFile( tolerance + ", " + move + ", " + target + ", " + currentHeading, true, false );

		if( target < leftLimit || target > rightLimit )
			power *= -1;

		if( target > currentHeading + tolerance )
			servo.setPower( -power );
		else if( target < currentHeading - tolerance )
			servo.setPower( power );
	}

	public void setRotationPower( double power, double position, AngleUnit angleUnit ) {
		if( angleUnit.equals( AngleUnit.DEGREES ) )
			position *= Math.PI / 180;
		int distance = Drive.convertDistTicks( position % (2 * Math.PI), 2 * Math.PI, pulsesPerRevolution, gearRatio );
		servo.setPower( power );
		new Thread( ( ) -> {
			while( getState( ) == MovementState.MOVING )
				Robot.waitTime( 50 );

			servo.setPower( 0 );
		} ).start( );
	}

	public double getTurretHeading( ) {
		return Drive.convertTicksDist( getPosition( ), 2 * Math.PI, pulsesPerRevolution, gearRatio );
	}

	public int getPosition( ) {
		return turretPosition + encoder.getCurrentPosition( );
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


}
