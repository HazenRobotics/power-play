package org.firstinspires.ftc.teamcode.tests.RRUnparalleledTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

import java.util.Vector;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Tests")
//@Disabled
public class LocalizationOffsetTest extends LinearOpMode {

	MiniBot robot;

	@Override
	public void runOpMode( ) throws InterruptedException {
		MiniBot robot = new MiniBot( this );

		robot.drive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.drive.setPoseEstimate( robot.getStartPos( true, true ) );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		boolean usingTurret = false;

		int x = 1;
		int y = 0;

		GamepadEvents gamepad = new GamepadEvents( gamepad1 );

		telemetry.addLine("done");
		telemetry.update();

		waitForStart( );

		while( !isStopRequested( ) ) {
			robot.drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y,
							-gamepad1.left_stick_x,
							-gamepad1.right_stick_x
					)
			);

			robot.drive.update( );

			Pose2d poseEstimate = robot.drive.getPoseEstimate( );
			double heading = MiniBot.getJunctionTurretHeading( poseEstimate, x, y );
			double junctionHeight = PPField.getJunction( x, y ).height();

			Vector2d junctionVector = PPField.getJunctionPose( x, y, false );
			Vector2d robotVector = new Vector2d( poseEstimate.getX(), poseEstimate.getY() );

			double distance = Math.abs( junctionVector.distTo( robotVector ) );

			boolean aboveJunction = robot.leftLift.getMotorPositionInch() > junctionHeight + 3;
			boolean withinLinkageExtension = distance < robot.linkage.extensionLength + 12;

			if( aboveJunction && withinLinkageExtension ) {
				robot.linkage.moveToExtensionDistance( distance - 13.5 );
			} else if (robot.leftLift.getMotorPositionInch() < junctionHeight - 4) {
				robot.linkage.moveToExtensionDistance( 0 );
			}

			robot.liftPower( gamepad1.right_trigger - gamepad1.left_trigger );

			if( gamepad.a.onPress() ) {
				robot.claw.toggle();
			}

			if( gamepad.y.onPress() ) {
				usingTurret = !usingTurret;
			}

			if( usingTurret ) {
				robot.turret.setTargetHeading( heading );
			} else {
				robot.turret.setTargetHeading( 0 );
			}
			robot.turret.updatePID( 0.35 );



			if( gamepad.dpad_up.onPress( ) )
				y++;
			if( gamepad.dpad_down.onPress( ) )
				y--;
			if( gamepad.dpad_right.onPress( ) )
				x++;
			if( gamepad.dpad_left.onPress( ) )
				x--;

			telemetry.addData( "robot x", poseEstimate.getX( ) );
			telemetry.addData( "robot y", poseEstimate.getY( ) );
			telemetry.addData( "robot heading", Math.toDegrees( poseEstimate.getHeading( ) ) );
			telemetry.addLine( "pole x: " + x );
			telemetry.addLine( "pole y: " + y );
			telemetry.addData( "turret target heading", heading );
			telemetry.addData( "turret actual heading", robot.turret.getTurretHeading() );
			telemetry.addData( "distance", distance );
			telemetry.addData( "junction vector", junctionVector );
			telemetry.addData( "robot vector ", robotVector );
			telemetry.addData( "above", aboveJunction );
			telemetry.addData( "withinLinkageExtension", withinLinkageExtension );
			telemetry.update( );

			gamepad.update();
		}
	}
}
