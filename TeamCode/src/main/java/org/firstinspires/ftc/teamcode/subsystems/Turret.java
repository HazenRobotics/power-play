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
		this( hw, "turret", false, AngleUnit.RADIANS);
	}

	public Turret( HardwareMap hw, String motorName, boolean reverseMotor, AngleUnit angleUnit ) {
		unit = angleUnit;

		motor = hw.get( DcMotorEx.class, motorName );

		motor.setDirection( reverseMotor ? Direction.REVERSE : Direction.FORWARD );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
	}

	public void setPositionVelocity( double velocity, int position ) {
		motor.setTargetPosition( (int) Drive.convertTicksDist( position, 2 * Math.PI, PULSES_PER_REVOLUTION, GEAR_RATIO ) );
		motor.setVelocity( velocity, unit );
	}

	public void setPositionPower( double power, int position ) {
		motor.setTargetPosition( (int) Drive.convertTicksDist( position, 2 * Math.PI, PULSES_PER_REVOLUTION, GEAR_RATIO ) );
		motor.setPower( power );
	}

	public void resetTurret( ) {
		motor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		turretHeading = 0;
	}


}
