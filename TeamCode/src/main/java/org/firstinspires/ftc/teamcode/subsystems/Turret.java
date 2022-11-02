package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;

public class Turret {

	DcMotorEx motor;

	AngleUnit unit;

	double pulsesPerRevolution;
	double gearRatio;

	double turretHeading;

	double leftLimit = Double.NEGATIVE_INFINITY;
	double rightLimit = Double.POSITIVE_INFINITY;

	public Turret( HardwareMap hw ) {
		this( hw, "turret", false, AngleUnit.RADIANS, 1, 1 );
	}

	public Turret( HardwareMap hw, String motorName, boolean reverseMotor, AngleUnit angleUnit, double ppr, double gearRatio ) {
		unit = angleUnit;

		motor = hw.get( DcMotorEx.class, motorName );

		motor.setDirection( reverseMotor ? Direction.REVERSE : Direction.FORWARD );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		pulsesPerRevolution = ppr;
		this.gearRatio = gearRatio;

		resetTurret( );
	}

	public void setLimit( double left, double right ) {
		double multiplyer = 1;
		if( unit.equals( AngleUnit.DEGREES ) )
			multiplyer = 180 / Math.PI;
		leftLimit = left * multiplyer;
		rightLimit = right * multiplyer;
	}

	public void setRotationVelocity( double velocity, double position ) {
		motor.setTargetPosition( Drive.convertDistTicks( position, 2 * Math.PI, pulsesPerRevolution, gearRatio ) );
		motor.setVelocity( velocity, unit );
	}

	/**
	 * @param power    power to turn at
	 * @param position the rotation of the turret in the predetermined AngleUnit
	 */
	public void setRotationPower( double power, double position ) {
		setRotationPower( power, position, unit );
	}

	public void setLiveRotationPower( Vector2d move ) {

		double tolerance = 0.1;

		double moveX = move.getX( );
		double moveY = move.getY( );

		double target = move.angle( ); // target angle between
		double power = Math.sqrt( moveX * moveX + moveY * moveY - (moveX * moveX * moveY * moveY) );

		double currentHeading = Drive.convertTicksDist( motor.getCurrentPosition( ), 2 * Math.PI, pulsesPerRevolution, gearRatio );

		if( target < leftLimit || target > rightLimit )
			power *= -1;

		if( target > currentHeading + tolerance )
			motor.setPower( -power );
		else if( target < currentHeading - tolerance )
			motor.setPower( power );
	}

	public void setRotationPower( double power, double position, AngleUnit angleUnit ) {
		if( angleUnit.equals( AngleUnit.DEGREES ) )
			position *= 180 / Math.PI;
		motor.setTargetPosition( (int) Drive.convertDistTicks( position % (2 * Math.PI), 2 * Math.PI, pulsesPerRevolution, gearRatio ) );
		motor.setPower( power );
	}


	public void resetTurret( ) {
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		turretHeading = 0;
	}


}
