package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(group = "test")
public class MotorRPMTest extends LinearOpMode {

	DcMotorEx motor;

	@Override
	public void runOpMode( ) throws InterruptedException {
		motor = hardwareMap.get( DcMotorEx.class, "rotate" );

		waitForStart();

		motor.setPower( 1 );

		while(!isStopRequested()) {
			telemetry.addData( "velocity in radians", motor.getVelocity( AngleUnit.RADIANS ) );
			telemetry.update();
		}
	}
}
