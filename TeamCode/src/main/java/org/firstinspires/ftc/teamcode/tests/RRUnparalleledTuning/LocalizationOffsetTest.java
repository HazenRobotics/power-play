package org.firstinspires.ftc.teamcode.tests.RRUnparalleledTuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class LocalizationOffsetTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {
		MecanumDriveUnparalleled drive = new MecanumDriveUnparalleled( hardwareMap );

		drive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		waitForStart( );

		int x = 0;
		int y = 0;

		GamepadEvents gamepad = new GamepadEvents( gamepad1 );

		while( !isStopRequested( ) ) {
			drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y,
							-gamepad1.left_stick_x,
							-gamepad1.right_stick_x
					)
			);

			drive.update( );

			Pose2d poseEstimate = drive.getPoseEstimate( );
			double heading = MiniBot.getJunctionTurretHeading( poseEstimate, x, y );

			if( gamepad.dpad_up.onPress( ) )
				x++;
			if( gamepad.dpad_down.onPress( ) )
				x--;
			if( gamepad.dpad_right.onPress( ) )
				y++;
			if( gamepad.dpad_left.onPress( ) )
				y--;

			telemetry.addData( "robot x", poseEstimate.getX( ) );
			telemetry.addData( "robot y", poseEstimate.getY( ) );
			telemetry.addData( "robot heading", poseEstimate.getHeading( ) );
			telemetry.addLine( "pole x: " + x );
			telemetry.addLine( "pole y: " + y );
			telemetry.addData( "turret heading offset", heading );
			telemetry.update( );


		}
	}
}
