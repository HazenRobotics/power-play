package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Config
@TeleOp(group = "test")
public class MoveTurretAndLift extends OpMode {

	GamepadEvents controller1;

	MiniBot robot;

	public static int desiredLiftPos = 0;
	public static int desiredTurretPos = 0;

	@Override
	public void init( ) {
		robot = new MiniBot( this );
		controller1 = new GamepadEvents( gamepad1 );
	}

	@Override
	public void loop( ) {
		if ( controller1.a.onPress() ) {
			robot.leftLift.setHeightPower( 0.5, desiredLiftPos, false, false );
			robot.rightLift.setHeightPower( 0.5, desiredLiftPos, false, false );

			robot.turret.setRotate( 0.5, desiredTurretPos, false );
			while ( (robot.leftLift.isBusy() || robot.turret.isBusy()) && robot.opModeIsActive( ) ) {
				telemetry.addData( "lift position", robot.leftLift.getMotorPosition() );
				telemetry.addData( "lift target", robot.leftLift.getTarget() );
				telemetry.update();
			}
			telemetry.addLine("finished");
			telemetry.update();
		}
		controller1.update();
	}
}
