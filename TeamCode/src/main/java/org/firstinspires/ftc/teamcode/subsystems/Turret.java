package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;

public class Turret {

	DcMotorEx motor;
	AngleUnit unit;
	double ratio;
	double PPR;

	public Turret( HardwareMap hw ) {
		this( hw, "turret", false, AngleUnit.RADIANS, 1, 1 );
	}

	public Turret( HardwareMap hw, String motorName, boolean reverseMotor, AngleUnit angleUnit, double gearRatio, double motorPPR ) {
		unit = angleUnit;
		ratio = gearRatio;
		PPR = motorPPR;

		motor = hw.get( DcMotorEx.class, motorName );

		motor.setDirection( reverseMotor ? Direction.REVERSE : Direction.FORWARD );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
	}

	public void setPositionVelocity( double velocity, double position ) {
		motor.setTargetPosition( Drive.convertDistTicks( position, 2 * Math.PI, PPR, ratio ) );
		motor.setVelocity( velocity, unit );
	}

	public void setPositionPower( double power, double position ) {
		motor.setTargetPosition( Drive.convertDistTicks( position, 2 * Math.PI, PPR, ratio ) );
		motor.setPower( power );
	}
}
