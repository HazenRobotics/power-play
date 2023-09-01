package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robots.MiniBot;

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

	MiniBot robot;

	final boolean right = true;

	final double CYCLE_TURRET_HEADING = -(90 + 45);
	final float STACK_HEIGHT_SCALAR = 1.5f;
	final float STACK_HEIGHT_OFFSET = 0.1f;
	int currentConeHeight = 4;
	public DcMotorEx frontLeft, frontRight, backLeft, backRight;

	@Override
	public void runOpMode( ) throws InterruptedException {
		frontLeft = hardwareMap.get( DcMotorEx.class, "frontLeft" );
		frontRight = hardwareMap.get( DcMotorEx.class, "frontRight" );
		backLeft = hardwareMap.get( DcMotorEx.class, "backLeft" );
		backRight = hardwareMap.get( DcMotorEx.class, "backRight" );

		frontLeft.setTargetPosition( 0 );
		frontRight.setTargetPosition( 0 );
		backLeft.setTargetPosition( 0 );
		backRight.setTargetPosition( 0 );

		frontLeft.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		frontRight.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION  );
		backRight.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		waitForStart();
		while( opModeIsActive( ) ) {
			telemetry.addData( "FL", frontLeft.getCurrentPosition() );
			telemetry.addData( "FR", frontRight.getCurrentPosition()  );
			telemetry.addData( "BL", backLeft.getCurrentPosition()  );
			telemetry.addData( "BR", backRight.getCurrentPosition()  );
			telemetry.update();
		}
	}
}
