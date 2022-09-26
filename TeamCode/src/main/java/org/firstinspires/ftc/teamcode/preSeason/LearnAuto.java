package org.firstinspires.ftc.teamcode.preSeason;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Learn Auto", group = "Test")
public class LearnAuto extends LinearOpMode {

	DcMotorEx frontLeft, backLeft, frontRight, backRight;
	final double PULSES_PER_REVOLUTION = 537.7;
	final double GEAR_RATIO = 1;
	final double wheelSize = 3.875 * Math.PI;//in bad imperial inches


	@Override
	public void runOpMode( ) throws InterruptedException {
		frontLeft = hardwareMap.get( DcMotorEx.class, "frontLeft" );
		backLeft = hardwareMap.get( DcMotorEx.class, "backLeft" );
		frontRight = hardwareMap.get( DcMotorEx.class, "frontRight" );
		backRight = hardwareMap.get( DcMotorEx.class, "backRight" );
		frontRight.setDirection( DcMotorSimple.Direction.REVERSE );
		backRight.setDirection( DcMotorSimple.Direction.REVERSE );



		int distanceToMove = Math.round( convertDistTicks( 24 ) );
		frontLeft.setTargetPosition( distanceToMove );
		backRight.setTargetPosition( distanceToMove );
		frontRight.setTargetPosition( distanceToMove );
		backLeft.setTargetPosition( distanceToMove );

		frontLeft.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		backLeft.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		frontRight.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		backRight.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		waitForStart( );
		frontLeft.setPower( -0.5 );
		backLeft.setPower( -1 );
		frontRight.setPower( 0.5 );
		backRight.setPower( 1 );
		while(frontLeft.isBusy());
		frontLeft.setPower( 0 );
		backLeft.setPower( 0 );
		frontRight.setPower( 0 );
		backRight.setPower( 0 );

	}


	/**
	 * @param distanceToTravel the distance to move in inches
	 * @return totalTicks - the amount of ticks to move forward
	 */
	public int convertDistTicks( double distanceToTravel ) {
		double revolutions = distanceToTravel / wheelSize;
		int totalTicks = (int) Math.round( (revolutions * PULSES_PER_REVOLUTION) / GEAR_RATIO );

		return totalTicks;
	}

	public int convertTicksDist( double ticksToTravel ) {
		double calculations = ticksToTravel * wheelSize * GEAR_RATIO;
		int totalDistance = (int) Math.round( calculations / PULSES_PER_REVOLUTION );

		return totalDistance;
	}

}
