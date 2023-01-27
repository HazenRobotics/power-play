package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(group = "Park")
public class JustParkRight extends LinearOpMode {

	MiniBot robot;

	final boolean right = true;

	@Override
	public void runOpMode( ) throws InterruptedException {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new MiniBot( this );

		robot.initSubsystems( );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

		while( !isStopRequested( ) && !isStarted( ) ) {
			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
			telemetry.update( );
		}

		waitForStart( );

		robot.claw.setState( SingleServoClaw.ClawState.CLOSED );
		robot.junctionToLiftPos( PPField.Junction.GROUND );

//		Vector2d parkPos = robot.parkPosInit( right );
		Vector2d conePos = MiniBot.getSignalPos( right );

		robot.signalUtil.stopCamera( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( right ) );

//		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
//				.lineTo( conePos )
//				.lineTo( parkPos )
//				.build( );
//
//		robot.drive.followTrajectorySequence( mainTrajectory );

	}
}
