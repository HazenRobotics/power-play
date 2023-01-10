package org.firstinspires.ftc.teamcode.autonomous.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoClaw;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Disabled
@Autonomous(name = "Cones (NOT THIS ONE, Right)")
public class ConesRedRight extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = true;
	final int[] quadSign = MiniBot.getQuadrantSign( red, right );

	double CYCLE_TURRET_HEADING = -135;

	@Override
	public void runOpMode( ) throws InterruptedException {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new MiniBot( this );

		MiniBot.setRedSide( true );

//		robot.initSubsystems( );
		robot.claw.setState( SingleServoClaw.ClawState.CLOSED );

		Vector2d parkPos = robot.parkPosInit( red, right );
		Vector2d conePos = MiniBot.getSignalPos( red, right );
		Pose2d medJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), quadSign[0], quadSign[1] );
		Pose2d initialHighJunction = MiniBot.getJunctionOffsetPos( MiniBot.getAngleOnSide( red, right ), MiniBot.CLAW_OFFSET, quadSign[1] == -1 ? 90 : 270, quadSign[0], 0 );
		Pose2d cycleHighJunction = new Pose2d( initialHighJunction.getX(), initialHighJunction.getY(), 0 );
		Pose2d pickUpPos = MiniBot.getJunctionOffsetPos( Math.toRadians( quadSign[0] == 1 ? 0 : 180 ), MiniBot.ROBOT_MAX_LENGTH - MiniBot.ROBOT_LENGTH / 2, new Vector2d( quadSign[0] * PPField.TILE_SIZE * 3, (red ? -1 : 1) * PPField.TILE_SIZE / 2 ) );

//		camden's because i have no clue how to utilize the offset pos function above
		Pose2d altPickupPos = new Pose2d( 2.5 * (PPField.TILE_SIZE + PPField.TILE_CONNECTOR) - 7, -PPField.TILE_SIZE / 2 - PPField.TILE_CONNECTOR + 2, 0 );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		Robot.writeToDefaultFile( "start of file", false, false );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

//		while( !isStopRequested( ) && !isStarted( ) ) {
//			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
//			telemetry.update( );
//		}

		waitForStart( );

		robot.junctionToLiftPos( PPField.Junction.GROUND );
//		robot.signalUtil.stopCamera( );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
//				.lineTo( conePos )
				.lineToLinearHeading( new Pose2d( conePos.getX( ), conePos.getY( ), quadSign[0] == 1 ? 0 : 180 ) ) // straighten out and center
				.addSpatialMarker( new Vector2d( 36, -30 ),
						( ) -> {
							robot.junctionToLiftPos( PPField.Junction.HIGH );
						} )
				.lineToLinearHeading( initialHighJunction ) // move to high junction
				.waitSeconds( 0.2 )
				.addTemporalMarker( ( ) -> {
					// move turret async
					robot.turret.setRotationPower( 0.5, -40 );
				} )
				.waitSeconds( 0.75 ) // wait
				.addTemporalMarker( ( ) -> {
					// deploy & open claw
//					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
					robot.claw.setState( SingleServoClaw.ClawState.OPEN );
					// return turret to front
				} )
				.waitSeconds( 0.25 )
				.lineToLinearHeading( altPickupPos ) // straighten out and center
				// grab new cone
				.addTemporalMarker( () -> {
					robot.turret.setRotationPower( 0.5, 0 );
					// return lift down async
					robot.lift.setHeightPower( 1, 5 );
					// stow claw
//					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
				} )
				.waitSeconds( 0.5 ) // wait
				.forward( 4.5 )
				.addTemporalMarker( ( ) -> {
					// close claw
//					robot.claw.setState( TiltingClaw.ClawState.CLOSED );
					// move lift up async
					robot.junctionToLiftPos( PPField.Junction.HIGH );
				} )
				.waitSeconds( 0.5 )
				.lineToLinearHeading( cycleHighJunction )
				.addTemporalMarker( ( ) -> {
					// move turret async
					robot.turret.setRotationPower( 0.5, CYCLE_TURRET_HEADING );
				} )
				.waitSeconds( 0.5 ) // wait
				.addTemporalMarker( ( ) -> {
					// deploy & open claw
//					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
					robot.claw.setState( SingleServoClaw.ClawState.OPEN );
					// return turret to front
				} )
				.build( );

		robot.drive.followTrajectorySequence( mainTrajectory );

		while( !isStopRequested() );

	}
}
