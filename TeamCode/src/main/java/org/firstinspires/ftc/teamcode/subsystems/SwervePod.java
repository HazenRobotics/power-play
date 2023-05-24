package org.firstinspires.ftc.teamcode.subsystems;

import static org.apache.commons.math3.util.MathUtils.TWO_PI;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.SwervePIDController;

public class SwervePod {

	public DcMotorEx driveMotor, rotateMotor;
	double TICKS_PER_REV;
	SwervePIDController controller;

	double targetAngle = Math.PI / 2;


	public SwervePod( HardwareMap hw ) {
		this( hw, "drive", false, "rotate", true, new double[]{ 0, 0, 0 }, 537.7 );
	}

	/**
	 *
	 * @param hw
	 * @param driveM
	 * @param driveReverse
	 * @param rotateM
	 * @param rotateReverse
	 * @param PID
	 * @param TPR
	 */
	public SwervePod( HardwareMap hw, String driveM, boolean driveReverse,
					  String rotateM, boolean rotateReverse,
					  double[] PID, double TPR ) {
		driveMotor = hw.get( DcMotorEx.class, driveM );
		driveMotor.setDirection( driveReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
		rotateMotor = hw.get( DcMotorEx.class, rotateM );
		rotateMotor.setDirection( rotateReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );

		controller = new SwervePIDController( PID[0], PID[1], PID[2] );
		TICKS_PER_REV = TPR;
	}

	public void setPID( double p, double i, double d ) {
		controller.setPID( p, i, d );
	}

	public void updateRotatePD( ) {
		rotateMotor.setPower( -controller.update( targetAngle, getPodAngle( ) ) );
	}

	public double getError( ) {
		return controller.getError( );
	}

	public void setWheelPower( double power ) {
		driveMotor.setPower( power );
	}

	public void setWheelVelocity( double velocity ) {
		driveMotor.setVelocity( velocity );
	}

	public void reverseDriveMotor( ) {
		driveMotor.setDirection( driveMotor.getDirection( ) == DcMotorSimple.Direction.FORWARD ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD );
	}

	public boolean getDriveMotorReversed( ) {
		return driveMotor.getDirection( ) == DcMotorSimple.Direction.REVERSE;
	}

	public void setPodAngleTarget( double target ) {
		targetAngle = target;
	}

	public double getPodAngleTarget( ) {
		 return targetAngle;
	}

	public double getPodAngle( ) {
		return convertTicksToAngle( rotateMotor.getCurrentPosition( ) );
	}

	public void resetPodAngle( ) {
		rotateMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rotateMotor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
	}

	public double convertTicksToAngle( double ticks ) {
		double angle = (ticks % TICKS_PER_REV) * (TWO_PI / TICKS_PER_REV);
		return angle < 0 ? angle + TWO_PI : angle;
	}

	public double convertAngleToTicks( double angle ) {
		return (angle / 360) * TICKS_PER_REV;
	}

}
