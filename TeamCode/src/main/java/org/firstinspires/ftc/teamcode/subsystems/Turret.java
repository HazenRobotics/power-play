package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;

public class Turret {

	final double PULSES_PER_REVOLUTION = 1; //TODO: get PPR of turret motor
	final double GEAR_RATIO = 1; //TODO: get gear ratio of turret

	DcMotorEx motor;
	AngleUnit unit;

	double turretHeading;

	public Turret( HardwareMap hw ) {
		this( hw, "turret", false, AngleUnit.RADIANS );
	}

	public Turret( HardwareMap hw, String motorName, boolean reverseMotor, AngleUnit angleUnit ) {
		unit = angleUnit;

		motor = hw.get( DcMotorEx.class, motorName );

		motor.setDirection( reverseMotor ? Direction.REVERSE : Direction.FORWARD );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		resetTurret( );
	}

	public void setRotationVelocity( double velocity, double position ) {
		motor.setTargetPosition( (int) Drive.convertDistTicks( position, 2 * Math.PI, PULSES_PER_REVOLUTION, GEAR_RATIO ) );
		motor.setVelocity( velocity, unit );
	}

	/**
	 * @param power    power to turn at
	 * @param position the rotation of the turret out of 360 (in degrees)
	 */
	public void setRotationPower( double power, double position ) {
		setRotationPower( power, position, AngleUnit.DEGREES );
	}


	public void setRotationPower( double power, double position, AngleUnit angleUnit ) {
		if( angleUnit.equals( AngleUnit.DEGREES ) )
			position *= 180 / Math.PI;
		motor.setTargetPosition( (int) Drive.convertDistTicks( position % (2 * Math.PI), 2 * Math.PI, PULSES_PER_REVOLUTION, GEAR_RATIO ) );
		motor.setPower( power );
	}

	public void resetTurret( ) {
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		turretHeading = 0;
	}


}
