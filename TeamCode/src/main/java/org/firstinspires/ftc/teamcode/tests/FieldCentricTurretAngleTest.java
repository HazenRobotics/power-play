package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.analysis.function.Min;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveUnparalleled;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Tests")
//@Disabled
public class FieldCentricTurretAngleTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		MiniBot robot = new MiniBot( this );

		robot.drive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.drive.setPoseEstimate( new Pose2d( 0,0, Math.toRadians( 90 ) ) );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		GamepadEvents gamepad = new GamepadEvents( gamepad1 );

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
			robot.turret.setTurretPower( gamepad2.right_stick_x * 0.5 );

			Pose2d poseEstimate = robot.drive.getPoseEstimate( );

			telemetry.addData( "robot x", poseEstimate.getX( ) );
			telemetry.addData( "robot y", poseEstimate.getY( ) );
			telemetry.addData( "robot heading", Math.toDegrees( poseEstimate.getHeading( ) ) );
			telemetry.addData( "field centric turret heading", robot.turret.getFieldCentricTurretHeading( Math.toDegrees( poseEstimate.getHeading() ) ) % 360 );
			telemetry.update( );

			gamepad.update();
		}
	}
}
