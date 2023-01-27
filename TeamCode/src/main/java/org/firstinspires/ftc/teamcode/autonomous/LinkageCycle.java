package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;

import java.util.Vector;

@Autonomous(group = "aaa")
public class LinkageCycle extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = true;

	public enum AutoState {
		INITIAL_DRIVE,
		CYCLE,
		PARK
	}

	public enum InitialDriveState {
		DRIVE_FIRST_CONE,
		MOVE_LIFT_AND_TURRET,
		DROP_CONE,
		RESET,
		TO_CYCLE
	}

	public enum CycleState {
		MOVE_TO_CONE_PICKUP,
		GRAB_CONE,
		LIFT_LIFT_TO_DELIVERY,
		TURN_TURRET_TO_DELIVERY,
		MOVE_TO_CONE_DELIVERY,
		DROP_CONE
	}

	AutoState autoState;
	InitialDriveState initialDriveState;
	CycleState cycleState;
	double coneStackHeight = -0.5;
	double cycleTurretHeading;
	double cycleLiftHeight = PPField.Junction.HIGH.height( ) + 3.5;
	double currentLiftHeight;
	double turretHeading;
	ElapsedTime timer = new ElapsedTime(  );
	AprilTagDetectionPipeline.SignalPosition signalPos;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new MiniBot( this );
		autoState = AutoState.INITIAL_DRIVE;
		initialDriveState = InitialDriveState.DRIVE_FIRST_CONE;

		robot.initSubsystems( );

		TrajectorySequence leftParkTrajectory = robot.getTrajectorySequenceBuilder( new Pose2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 ) )
				.setReversed( true )
				.lineToLinearHeading( new Pose2d( robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.LEFT ).getX(),
						robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.LEFT ).getY(), Math.toRadians( 270 ) ))
				.build( );

		TrajectorySequence middleParkTrajectory = robot.getTrajectorySequenceBuilder( new Pose2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 ) )
				.lineToLinearHeading( new Pose2d( robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.MIDDLE ).getX(),
						robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.MIDDLE ).getY(), Math.toRadians( 270 ) ))
				.build( );

		TrajectorySequence rightParkTrajectory = robot.getTrajectorySequenceBuilder( new Pose2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 ) )
				.lineToLinearHeading( new Pose2d( robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.RIGHT ).getX(),
						robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.RIGHT ).getY(), Math.toRadians( 270 ) ))
				.build( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );

		TrajectorySequence firstConeTraj = robot.getTrajectorySequenceBuilder( )
				.addTemporalMarker( () -> {
					robot.setLiftTargetInches( 5 );
					robot.turret.setTargetHeading( -138 );
				} )
				.splineTo( new Vector2d( 36, -48 ), Math.toRadians( 90 ) )
				.addTemporalMarker( 1.5, ( ) -> {
					robot.setLiftTargetInches( cycleLiftHeight );
				} )
				.addTemporalMarker( 1.6, ( ) -> {
					robot.turret.setTargetHeading( cycleTurretHeading );
					robot.linkage.moveToExtensionDistance( 14 );
				} )
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
					cycleState = CycleState.MOVE_TO_CONE_PICKUP;
				} )
				.build( );

//		TrajectorySequence firstCone = robot.getTrajectorySequenceBuilder()
//				.lineToLinearHeading( new Pose2d (36, -48, Math.toRadians( 0 ) ))
//				.lineToLinearHeading( new Pose2d( PPField.TILE_SIZE * 1.5, -PPField.TILE_SIZE * .65 , Math.toRadians( 90 ) ))
//				.build();
//
//		TrajectorySequence toCycle = robot.getTrajectorySequenceBuilder()
//				.setReversed( true )
//				.lineToLinearHeading(new Pose2d(  PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 , Math.toRadians( 0 )) )
//				.addTemporalMarker( () -> {
//					autoState = AutoState.PARK;
//				} )
//				.build();

		robot.drive.followTrajectorySequenceAsync( firstConeTraj );

		telemetry.addLine( "done" );
		telemetry.update( );

		while( !opModeIsActive() ) {
			signalPos = robot.signalUtil.getSignalPosition();
			telemetry.addData( "position", signalPos );
			telemetry.update( );
		}


		waitForStart( );
		robot.signalUtil.stopCamera( );

		while( opModeIsActive( ) ) {
			currentLiftHeight = (robot.leftLift.getMotorPositionInch( ) + robot.rightLift.getMotorPositionInch()) / 2;
			turretHeading = robot.turret.getTurretHeading( );
//			robot.drive.update();
			robot.updatePIDs( 0.5, 0.1 );

			displayTelemetry();

			switch( autoState ) {
//				case INITIAL_DRIVE:
//					switch(initialDriveState) {
//						case DRIVE_FIRST_CONE:
//							robot.drive.followTrajectorySequence( firstCone );
//							robot.setLiftTargetInches( PPField.Junction.HIGH.height( ) );
//							robot.turret.setTargetHeading( robot.getJunctionTurretHeading( 1, 0 ) );
//							robot.linkage.moveToExtensionDistance( 11 );
//							initialDriveState = InitialDriveState.MOVE_LIFT_AND_TURRET;
//							break;
//						case MOVE_LIFT_AND_TURRET:
//							if( currentLiftHeight - PPField.Junction.HIGH.height( ) < 0.25 && (turretHeading < robot.turret.getTargetHeading( ) + 2 && turretHeading > robot.turret.getTargetHeading( ) - 2) ) {
//								timer.reset( );
//								robot.claw.open( );
//								initialDriveState = InitialDriveState.DROP_CONE;
//							}
//							break;
//						case DROP_CONE:
//							if( timer.milliseconds( ) > 500 ) {
//								initialDriveState = InitialDriveState.RESET;
//							}
//							break;
//						case RESET:
//							robot.setLiftTargetInches( 3 );
//							robot.turret.setTargetHeading( robot.getJunctionTurretHeading( 1, 0 ) );
//							robot.linkage.moveToExtensionDistance( 3 );
//							initialDriveState = InitialDriveState.TO_CYCLE;
//							break;
//						case TO_CYCLE:
//							if( currentLiftHeight - PPField.Junction.HIGH.height( ) < 0.25 && (turretHeading < robot.turret.getTargetHeading( ) + 2 && turretHeading > robot.turret.getTargetHeading( ) - 2) ) {
//								robot.drive.followTrajectorySequence( toCycle );
//								autoState = AutoState.PARK;
//							}
//							break;
//					}
//					break;
				case CYCLE:
					switch( cycleState ) {
						case MOVE_TO_CONE_PICKUP:
							if (coneStackHeight < 0) {
								autoState = AutoState.PARK;
								break;
							}
							if( Math.abs( currentLiftHeight - coneStackHeight ) < 0.25 && (turretHeading < 3 && turretHeading > -3) ) {
								timer.reset();
								robot.claw.close( );
								cycleState = CycleState.GRAB_CONE;
							}
							break;
						case GRAB_CONE:
							if (timer.nanoseconds() > .5 * 1000000000) {
								robot.setLiftTargetInches( cycleLiftHeight );
								cycleState = CycleState.LIFT_LIFT_TO_DELIVERY;
							}
							break;
						case LIFT_LIFT_TO_DELIVERY:
							if( Math.abs( currentLiftHeight - 10 ) < 0.5) {
								cycleTurretHeading = robot.getJunctionTurretHeading( (red && right) || (!red && !right) ? 1 : -1, 0 );
								robot.turret.setTargetHeading( cycleTurretHeading );
								robot.linkage.moveToExtensionDistance( 11.5 );
							}
							break;
						case TURN_TURRET_TO_DELIVERY:
							if( Math.abs( currentLiftHeight - cycleLiftHeight ) < 0.5 && (turretHeading < cycleTurretHeading + 0.5 && turretHeading > cycleTurretHeading - 0.5) ) {
								timer.reset();
								robot.claw.open( );
								coneStackHeight--;
								if (coneStackHeight < 0) {
									autoState = AutoState.PARK;
									break;
								}

								cycleState = CycleState.DROP_CONE;
							}
							break;
						case DROP_CONE:
							if(timer.nanoseconds() > 0.5 * 1000000000) {
								robot.setLiftTargetInches( coneStackHeight );
								robot.turret.setTargetHeading( 0 );
								robot.linkage.moveToExtensionDistance( 11.5 );
								cycleState = CycleState.MOVE_TO_CONE_PICKUP;
							}
							break;
					}
					break;
				case PARK:
					if (signalPos == AprilTagDetectionPipeline.SignalPosition.LEFT)
						robot.drive.followTrajectorySequence( leftParkTrajectory );
					else if (signalPos == AprilTagDetectionPipeline.SignalPosition.RIGHT)
						robot.drive.followTrajectorySequence( rightParkTrajectory );
					else
						robot.drive.followTrajectorySequence( middleParkTrajectory );
					this.requestOpModeStop();
					break;
			}
		}
	}

	public void displayTelemetry() {
		telemetry.addData( "autoState", autoState );
		telemetry.addData( "cycleState", cycleState );
		telemetry.addData( "turret target heading", robot.turret.getTargetHeading() );
		telemetry.addData( "turret heading", turretHeading );
		telemetry.addData( "lift target heading", robot.leftLift.getTargetPositionInch() );
		telemetry.addData( "lift position", currentLiftHeight );
		telemetry.addData( "cone stack height", coneStackHeight );
		telemetry.addData( "determined turret cycle angle", cycleTurretHeading );
		telemetry.addData( "time", timer.nanoseconds() );
		telemetry.update();
	}
}
