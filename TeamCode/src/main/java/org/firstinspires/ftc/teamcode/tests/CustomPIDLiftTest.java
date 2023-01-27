package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.MiniBot;

@Config
@Autonomous(group = "test")
public class CustomPIDLiftTest extends LinearOpMode {

	MiniBot robot;

	public static int target = 0;
	public static int desiredTarget = 0;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new MiniBot( this );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		telemetry.addLine( "ready to start" );
		telemetry.update();
		waitForStart();

		while( opModeIsActive() ) {

			if(gamepad1.a) {
				desiredTarget += 10;
			}

			if (gamepad1.b) {
				target = desiredTarget;
			}

			robot.leftLift.setTargetInches( target );
			robot.rightLift.setTargetInches( target );

			robot.leftLift.updatePID( 1 );
			robot.rightLift.updatePID( 1 );

			telemetry.addData( "lift target", robot.leftLift.getTarget() );
			telemetry.addData( "lift power", robot.leftLift.getPower() );
			telemetry.update();
		}
	}
}
