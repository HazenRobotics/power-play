package org.firstinspires.ftc.teamcode.autonomous.old;

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

@Disabled
@Autonomous(group = "Park")
public class JustParkBlueLeft extends LinearOpMode {

	MiniBot robot;

	final boolean red = false, right = false;

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

		robot.claw.setState( TiltingClaw.ClawState.CLOSED );
		robot.junctionToLiftPos( PPField.Junction.GROUND );

		Vector2d parkPos = robot.parkPosInit( red, right );
		Vector2d conePos = MiniBot.getSignalPos( red, right );

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
