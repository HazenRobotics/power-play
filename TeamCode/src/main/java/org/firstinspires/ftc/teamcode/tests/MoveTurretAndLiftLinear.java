package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Config
@TeleOp(group = "test")
public class MoveTurretAndLiftLinear extends LinearOpMode {

	GamepadEvents controller1;

	MiniBot robot;

	public static int desiredLiftPos = 0;
	public static int desiredTurretPos = 0;

	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new MiniBot( this );
		controller1 = new GamepadEvents( gamepad1 );

		telemetry.addLine("done");
		telemetry.update();

		waitForStart();

		while(opModeIsActive()) {
			if ( controller1.a.onPress() ) {
				robot.leftLift.setHeightPower( 1, desiredLiftPos, false, false );
				robot.rightLift.setHeightPower( 1, desiredLiftPos, false, false );
				robot.turret.setRotate( 0.5, desiredTurretPos, false );
				while ( (robot.leftLift.isBusy() || robot.turret.isBusy()) && opModeIsActive( ) ) {
					telemetry.addData( "lift position", robot.leftLift.getMotorPosition() );
					telemetry.addData( "lift target", robot.leftLift.getTargetPosition() );
					telemetry.update();
				}
				telemetry.addLine("finished");
				telemetry.update();
			}
			controller1.update();
		}
	}

}
