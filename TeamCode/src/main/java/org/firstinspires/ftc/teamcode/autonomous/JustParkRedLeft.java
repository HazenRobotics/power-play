package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(group = "A")
public class JustParkRedLeft extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = false;

	@Override
	public void runOpMode( ) throws InterruptedException {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new MiniBot( this );

		robot.signalUtil.init( );
		robot.claw.close( );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

		while( !isStopRequested( ) && !isStarted( ) ) {
			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
			telemetry.update( );
		}

		waitForStart( );

		robot.junctionToLiftPos( PPField.Junction.GROUND );

		Vector2d conePos = robot.getSignalPos( red, right );
		Vector2d parkPos = robot.justParkInit( red, right );

		robot.signalUtil.stopCamera( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
				.lineTo( conePos )
				.lineTo( parkPos )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
