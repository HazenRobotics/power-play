package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@TeleOp
public class CRServoWithEncoderTest extends LinearOpMode {

	CRServo servo;
	AnalogInput servoEncoder;
	double maxVoltage = 3.3;
	public static double power = 0;

	@Override
	public void runOpMode( ) throws InterruptedException {
		servo = hardwareMap.crservo.get( "servo" );
		servoEncoder = hardwareMap.analogInput.get( "servoEncoder" );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();

		while(opModeIsActive()) {
			servo.setPower( power );

			telemetry.addData( "servo angle", (servoEncoder.getVoltage() / maxVoltage) * 360 );
			telemetry.update();
		}
	}
}
