package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DeadWheelTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		DcMotorEx para = hardwareMap.get( DcMotorEx.class,"para" );
		DcMotorEx perp = hardwareMap.get( DcMotorEx.class,"perp" );
		para.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		perp.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		while( opModeIsActive() ) {
			telemetry.addData( "Para: ",para.getCurrentPosition() );
			telemetry.addData( "Prep: ",perp.getCurrentPosition() );
			telemetry.addData( "Wheel circumference",0.728 );
			telemetry.update();
		}
	}
}
