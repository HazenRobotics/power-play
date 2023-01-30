package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.subsystems.TwoAxesClaw;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(group = "")
@Disabled
public class MovingTurretAndLiftWhileDriving extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = true;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new MiniBot( this );

		robot.initSubsystems();

		telemetry.addLine("done");
		telemetry.update();

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( new Pose2d( 0, 0, 0 ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
				.forward( 30 )
				.addTemporalMarker( () -> {
					robot.setLiftTargetInches( 5 );
				} )
				.build( );

		robot.drive.followTrajectorySequenceAsync( mainTrajectory );

		waitForStart();
		robot.signalUtil.stopCamera( );

		while(opModeIsActive()) {
			robot.drive.update();
			robot.updatePIDs();
			displayTelemetry( );

		}





	}

	public void displayTelemetry() {
		telemetry.addData( "turret target heading", robot.turret.getTargetHeading() );
		telemetry.addData( "turret heading", robot.turret.getTurretHeading() );
		telemetry.addData( "lift target heading", robot.leftLift.getTargetPositionInch() );
		telemetry.addData( "lift position", robot.leftLift.getMotorPositionInch() );
		telemetry.addData( "linkage position", robot.linkage.getExtensionDistance() );
		telemetry.update();
	}
}
