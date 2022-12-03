package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class DeadWheelTest extends OpMode {
	DcMotorEx para,perp;
	@Override
	public void init( ) {
		 para = hardwareMap.get( DcMotorEx.class, "para" );
		 perp = hardwareMap.get( DcMotorEx.class, "perp" );
		 para.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		 perp.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
	}

	@Override
	public void loop( ) {
		telemetry.addData( "Para: ",para.getCurrentPosition() );
		telemetry.addData( "Prep: ",perp.getCurrentPosition() );
		telemetry.addData( "Wheel circumference",0.728 );
		telemetry.update();
	}
}
