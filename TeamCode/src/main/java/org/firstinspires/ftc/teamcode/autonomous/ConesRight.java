package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TiltingClaw;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(name = "Cones")
public class ConesRight extends LinearOpMode {

	MiniBot robot;

	final boolean right = true;

	final double CYCLE_TURRET_HEADING = -(90 + 43);
	final float STACK_HEIGHT_SCALAR = 1.5f;
	final float STACK_HEIGHT_OFFSET = 0.2f;
	int currentConeHeight = 4;


	@Override
	public void runOpMode( ) throws InterruptedException {

		Robot.createMatchLogFile( getClass( ).getSimpleName( ) );

		robot = new MiniBot( this );

		MiniBot.setRedSide( true );

		boolean t = true, f = false;

		telemetry.addLine( );

//		robot.initSubsystems( );
		robot.claw.setState( TiltingClaw.ClawState.CLOSED );
		robot.waitSeconds( 0.25 );
		robot.claw.setState( TiltingClaw.VerticalClawState.STOWED );

		Vector2d parkPos = robot.parkPosInit( right );
		Vector2d conePos = MiniBot.getSignalPos( right );
		Pose2d medJunction = MiniBot.getJunctionOffsetPos( right ? 135 : 45, right ? 1 : -1, -1 );
		Pose2d initialHighJunction = MiniBot.getJunctionOffsetPos( right ? 135 : 45, MiniBot.CLAW_OFFSET, 90, right ? 1 : -1, 0 );
		Pose2d cycleHighJunction = new Pose2d( initialHighJunction.getX( ), initialHighJunction.getY( ), 0 );
		Pose2d pickUpPos = MiniBot.getJunctionOffsetPos( Math.toRadians( right ? 0 : 180 ), MiniBot.CLAW_OFFSET + 4, new Vector2d( (right ? 1 : -1) * (PPField.HALF_FIELD - PPField.CONE_RADIUS), -1 * PPField.CONE_STACK_OFFSET ) );

//		camden's because i have no clue how to utilize the offset pos function above
		Pose2d altPickUpPos = new Pose2d( 2.5 * (PPField.TILE_SIZE + PPField.TILE_CONNECTOR) - 7, -PPField.TILE_SIZE / 2 - PPField.TILE_CONNECTOR + 2, 0 );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( right ) );

//		Robot.writeToDefaultFile( "start of file", false, false );

		TrajectorySequence mainTrajectory = robot.getTrajectorySequenceBuilder( )
//				.lineTo( conePos )
				.lineToLinearHeading( new Pose2d( conePos.getX( ), conePos.getY( ), right ? 0 : 180 ) ) // straighten out and center
				.addSpatialMarker( new Vector2d( 36, -30 ), ( ) -> {
					robot.junctionToLiftPos( PPField.Junction.HIGH, 2 );
				} )
				.lineToLinearHeading( initialHighJunction ) // move to high junction
				.waitSeconds( 0.2 )
				.addTemporalMarker( ( ) -> {
					// move turret async
					robot.turret.setRotationPower( 0.5, -41.5 );
					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
				} )
				.waitSeconds( 0.75 ) // wait
				.addTemporalMarker( ( ) -> {
					// deploy & open claw
					robot.lift.setHeightPower( 1, PPField.Junction.HIGH.height( ) + 2, false, true ); /// double check height
					robot.claw.setState( TiltingClaw.ClawState.OPEN );
					// return turret to front
				} )
				.waitSeconds( 0.35 )

				// =================================================================================

				.lineToLinearHeading( pickUpPos ) // straighten out and center
				// grab new cone
				.addTemporalMarker( ( ) -> {
					robot.turret.setRotationPower( 0.5, 0 );
					// return lift down async
					robot.lift.setHeightPower( 1, STACK_HEIGHT_OFFSET + STACK_HEIGHT_SCALAR * currentConeHeight-- );
					// stow claw
					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
				} )
				.waitSeconds( 0.5 ) // wait
				.forward( 4.35 )
				.addTemporalMarker( ( ) -> {
					// close claw
					robot.claw.setState( TiltingClaw.ClawState.CLOSED );
					// move lift up async
					robot.junctionToLiftPos( PPField.Junction.HIGH );
				} )
				.waitSeconds( 0.5 )
				.addTemporalMarker( ( ) -> {
					// move turret async
					robot.turret.setRotationPower( 0.5, CYCLE_TURRET_HEADING );
				} )
				.lineToLinearHeading( cycleHighJunction )
				.waitSeconds( 0.5 ) // wait
				.addTemporalMarker( ( ) -> {
					// deploy & open claw
					while( robot.turret.isBusy( ) && Math.abs( robot.turret.getPower( ) ) > 0.05 );

					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
					robot.waitSeconds( 0.1 );
					robot.claw.setState( TiltingClaw.ClawState.OPEN );
					// return turret to front
				} )

				// =================================================================================

				.lineToLinearHeading( pickUpPos ) // straighten out and center
				// grab new cone
				.addTemporalMarker( ( ) -> {
					robot.turret.setRotationPower( 0.5, 0 );
					// return lift down async
					robot.lift.setHeightPower( 1, STACK_HEIGHT_OFFSET + STACK_HEIGHT_SCALAR * currentConeHeight-- );
					// stow claw
					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
				} )
				.waitSeconds( 0.5 ) // wait
				.forward( 4.35 )
				.addTemporalMarker( ( ) -> {
					// close claw
					robot.claw.setState( TiltingClaw.ClawState.CLOSED );
					// move lift up async
					robot.junctionToLiftPos( PPField.Junction.HIGH );
				} )
				.waitSeconds( 0.5 )
				.addTemporalMarker( ( ) -> {
					// move turret async
					robot.turret.setRotationPower( 0.5, CYCLE_TURRET_HEADING );
				} )
				.lineToLinearHeading( cycleHighJunction )
				.waitSeconds( 0.5 ) // wait
				.addTemporalMarker( ( ) -> {
					// deploy & open claw
					while( robot.turret.isBusy( ) && Math.abs( robot.turret.getPower( ) ) > 0.05 );

					robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
					robot.waitSeconds( 0.1 );
					robot.claw.setState( TiltingClaw.ClawState.OPEN );
					// return turret to front
				} )
				.build( );

		telemetry.addLine( "Ready!" );
		telemetry.update( );

//		while( !isStopRequested( ) && !isStarted( ) ) {
//			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
//			telemetry.update( );
//		}

		waitForStart( );

		robot.junctionToLiftPos( PPField.Junction.GROUND );
//		robot.signalUtil.stopCamera( );


		robot.drive.followTrajectorySequence( mainTrajectory );

		while( !isStopRequested( ) ) ;
	}
}
