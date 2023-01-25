package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@Autonomous(group = "aaa")
public class LinkageCycle extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = true;

	public enum AutoState {
		DRIVING,
		CYCLE,
	}

	public enum CycleState {
		PICKUP_CONE,
		DELIVER_CONE,
	}

	AutoState autoState;
	CycleState cycleState;
	int coneStackHeight = 4;
	int cycleTurretHeading = -138;
	double cycleLiftHeight = PPField.Junction.HIGH.height( ) + 3.5;
	double currentLiftHeight;
	double turretHeading;
	ElapsedTime timer = new ElapsedTime(  );

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new MiniBot( this );
		autoState = AutoState.DRIVING;

		robot.initSubsystems( );

		Vector2d parkPos = MiniBot.getSignalPos( right );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		TrajectorySequence cycleTrajectory = robot.getTrajectorySequenceBuilder( )
				.addTemporalMarker( () -> {
					robot.setLiftTargetInches( cycleLiftHeight );
					robot.turret.setTargetHeading( cycleTurretHeading );

				} )
				.splineTo( new Vector2d( 36, -48 ), Math.toRadians( 90 ) )
//				.addTemporalMarker( 1.5, ( ) -> {
//					robot.setLiftTargetInches( cycleLiftHeight );
//				} )
//				.addTemporalMarker( 1.6, ( ) -> {
//					robot.turret.setTargetHeading( cycleTurretHeading );
//					robot.linkage.moveToExtensionDistance( 14 );
//				} )
				.addTemporalMarker( 2.5, () -> {
					robot.linkage.moveToExtensionDistance( 13.5 );
				} )
				.splineTo( new Vector2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
					robot.claw.open( );
					robot.setLiftTargetInches( coneStackHeight );
					robot.turret.setTargetHeading( 0 );
					robot.linkage.moveToExtensionDistance( 11.5 );

					autoState = AutoState.CYCLE;
					cycleState = CycleState.PICKUP_CONE;
				} )
				.build( );

		TrajectorySequence parkTrajectory = robot.getTrajectorySequenceBuilder( new Pose2d( PPField.TILE_SIZE * 2, -PPField.TILE_SIZE * .5 ) )
				.lineToConstantHeading( parkPos )
				.build( );

		robot.drive.followTrajectorySequenceAsync( cycleTrajectory );

		telemetry.addLine( "done" );
		telemetry.update( );



		waitForStart( );
		robot.signalUtil.stopCamera( );

		while( opModeIsActive( ) ) {
			double currentLiftHeight = robot.leftLift.getPositionInch( );
			double turretHeading = robot.turret.getTurretHeading( );
			robot.updatePIDs( );
			displayTelemetry();

			switch( autoState ) {
				case DRIVING:
					robot.drive.update( );
					break;
				case CYCLE:
					switch( cycleState ) {
						case PICKUP_CONE:
							if( Math.abs( currentLiftHeight - coneStackHeight ) < 0.25 && (turretHeading < 2 && turretHeading > -2) ) {
								robot.claw.close( );
								robot.setLiftTargetInches( cycleLiftHeight );
								robot.turret.setTargetHeading( cycleTurretHeading );
								cycleState = CycleState.DELIVER_CONE;
							}
							break;
						case DELIVER_CONE:
							if( Math.abs( currentLiftHeight - cycleLiftHeight ) < 0.5 && (turretHeading < cycleTurretHeading + 0.5 && turretHeading > cycleTurretHeading - 0.5) ) {
								robot.claw.open( );
								robot.setLiftTargetInches( coneStackHeight );
								robot.turret.setTargetHeading( 0 );
								coneStackHeight--;

								if (cycleLiftHeight < 0) {
									robot.drive.followTrajectorySequence( parkTrajectory );
									autoState = AutoState.DRIVING;
								}
								cycleState = CycleState.PICKUP_CONE;
							}
							break;
					}
					break;
			}
		}
	}

	public void displayTelemetry() {
		telemetry.addData( "autoState", autoState );
		telemetry.addData( "cycleState", cycleState );
		telemetry.addData( "turret target heading", robot.turret.getTarget() );
		telemetry.addData( "turret heading", turretHeading );
		telemetry.addData( "lift target heading", robot.leftLift.getTarget() );
		telemetry.addData( "lift position", currentLiftHeight );
		telemetry.addData( "time", timer.nanoseconds() );
		telemetry.update();
	}
}
