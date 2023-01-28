package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.LinkageCycle.CycleState.EXTEND_LINKAGE_AGAIN;
import static org.firstinspires.ftc.teamcode.autonomous.LinkageCycle.CycleState.GRAB_CONE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;

@Autonomous(name = "Linkage Cycle", group = "aaa")
public class LinkageCycle extends LinearOpMode {

	MiniBot robot;

	final boolean red = true, right = true;

	public enum AutoState {
		INITIAL_DRIVE,
		CYCLE,
		PARK,
		UPDATE
	}

	public enum CycleState {
		MOVE_TO_CONE_PICKUP,
		EXTEND_LINKAGE,
		GRAB_CONE,
		LIFT_LIFT_TO_DELIVERY,
		TURN_TURRET_TO_DELIVERY,
		EXTEND_LINKAGE_AGAIN,
		LIFT_LIFT_A_BIT,
		DROP_CONE
	}

	public enum ParkState {
		RESET,
		PARK,
		HALT
	}

	AutoState autoState;
	CycleState cycleState;
	ParkState parkState = ParkState.RESET;
	double coneStackHeight = 5;
	double cycleTurretHeading;
	double cycleLiftHeight = PPField.Junction.HIGH.height( ) + 1.5;
	double currentLiftHeight;
	double turretHeading;
	ElapsedTime timer = new ElapsedTime( );
	AprilTagDetectionPipeline.SignalPosition signalPos;

	@Override
	public void runOpMode( ) throws InterruptedException {

		robot = new MiniBot( this );
		autoState = AutoState.INITIAL_DRIVE;
		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		robot.initSubsystems( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( robot.getStartPos( red, right ) );


//		TrajectorySequence firstConeTraj = robot.getTrajectorySequenceBuilder( )
//				.addTemporalMarker( () -> {
//					robot.setLiftTargetInches( 5 );
//					robot.turret.setTargetHeading( -138 );
//				} )
//				.splineTo( new Vector2d( 36, -48 ), Math.toRadians( 90 ) )
//				.addTemporalMarker( 1.5, ( ) -> {
//					robot.setLiftTargetInches( cycleLiftHeight );
//				} )
//				.addTemporalMarker( 1.6, ( ) -> {
//					robot.turret.setTargetHeading( cycleTurretHeading );
//					robot.linkage.moveToExtensionDistance( 14 );
//				} )
//				.addTemporalMarker( 2.5, () -> {
//					robot.linkage.moveToExtensionDistance( 13.5 );
//				} )
//				.splineTo( new Vector2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65 ), Math.toRadians( 0 ) )
//				.addTemporalMarker( ( ) -> {
//					robot.claw.open( );
//					robot.setLiftTargetInches( coneStackHeight );
//					robot.turret.setTargetHeading( 0 );
//					robot.linkage.moveToExtensionDistance( 11.5 );
//
//					autoState = AutoState.CYCLE;
//					cycleState = CycleState.MOVE_TO_CONE_PICKUP;
//				} )
//				.build( );

		TrajectorySequence toCycle = robot.getTrajectorySequenceBuilder( )
//				.lineToLinearHeading( new Pose2d( 36, -48, Math.toRadians( 0 ) ) )
//				.lineToLinearHeading( new Pose2d( PPField.TILE_SIZE * 1.5, -PPField.TILE_SIZE * .65, Math.toRadians( 0 ) ) )

//				.setTangent( Math.toRadians( 50 ) )
//				.lineToSplineHeading( new Pose2d( PPField.TILE_SIZE * 1.5, -PPField.TILE_SIZE * .65, Math.toRadians( 0 ) ) )

				.setTangent( Math.toRadians( 60 ) )
				.splineToLinearHeading( new Pose2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65, Math.toRadians( 0 ) ), Math.toRadians( 0 ) )

				.addTemporalMarker( 2.2, ( ) -> {
					robot.turret.setTargetHeading( -135 );
				} )
//				.setReversed( true )
//				.lineToLinearHeading( new Pose2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65, Math.toRadians( 0 ) ) )
				.addTemporalMarker( ( ) -> {
					timer.reset( );
					robot.setLiftTargetInches( cycleLiftHeight );
					autoState = AutoState.CYCLE;
					cycleState = CycleState.LIFT_LIFT_TO_DELIVERY;
				} )
				.build( );

		// beginning delay, push pole



		Pose2d parkStartPos = new Pose2d( PPField.TILE_SIZE * 2 + 1, -PPField.TILE_SIZE * .65, Math.toRadians( 0 ) );
		TrajectorySequence leftParkTrajectory = robot.getTrajectorySequenceBuilder( parkStartPos )
//				.setReversed( true )
				.lineToLinearHeading( new Pose2d( robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.LEFT ).getX( ),
						robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.LEFT ).getY( ), Math.toRadians( 270 ) ) )
				.build( );

		TrajectorySequence middleParkTrajectory = robot.getTrajectorySequenceBuilder( parkStartPos )
				.lineToLinearHeading( new Pose2d( robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.MIDDLE ).getX( ),
						robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.MIDDLE ).getY( ), Math.toRadians( 270 ) ) )
				.build( );

		TrajectorySequence rightParkTrajectory = robot.getTrajectorySequenceBuilder( parkStartPos )
				.lineToLinearHeading( new Pose2d( robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.RIGHT ).getX( ),
						robot.parkPosInit( right, AprilTagDetectionPipeline.SignalPosition.RIGHT ).getY( ), Math.toRadians( 270 ) ) )
				.build( );


		robot.drive.followTrajectorySequenceAsync( toCycle );

		telemetry.addLine( "done" );
		telemetry.update( );

		while( !opModeIsActive( ) && opModeInInit( ) ) {
			signalPos = robot.signalUtil.getSignalPosition( );
			telemetry.addData( "position", signalPos );
			telemetry.update( );
		}

		waitForStart( );
		robot.signalUtil.stopCamera( );
		robot.claw.close( );
		robot.waitSeconds( 0.45 ); // .35
		robot.setLiftTargetInches( 15 );
		timer.reset( );
		this.resetRuntime( );


		while( opModeIsActive( ) ) {
			currentLiftHeight = (robot.leftLift.getMotorPositionInch( ) + robot.rightLift.getMotorPositionInch( )) / 2;
			turretHeading = robot.turret.getTurretHeading( );
			robot.drive.update( );
			robot.updatePIDs( 1, 0.45 ); // 0.6

			if( this.getRuntime( ) > 26 ) {
				autoState = AutoState.PARK;
			}

			displayTelemetry( );

			switch( autoState ) {
				case INITIAL_DRIVE:
					break;
				case CYCLE:
					switch( cycleState ) {
						case EXTEND_LINKAGE:
							if( Math.abs( currentLiftHeight - coneStackHeight ) < 0.25 && Math.abs( turretHeading ) < 1 ) {
								timer.reset( );
								robot.linkage.moveToExtensionDistance( 14 );
								cycleState = CycleState.MOVE_TO_CONE_PICKUP;
							}
							break;
						case MOVE_TO_CONE_PICKUP:
							if( timer.milliseconds( ) > 300 ) {
								timer.reset( );
								robot.claw.close( );
								robot.linkage.moveToExtensionDistance( 10 );
								cycleState = GRAB_CONE;
							}
							break;
						case GRAB_CONE:
							if( timer.nanoseconds( ) > .5 * 1000000000 ) {
								robot.setLiftTargetInches( cycleLiftHeight );
								cycleState = CycleState.LIFT_LIFT_TO_DELIVERY;
							}
							break;
						case LIFT_LIFT_TO_DELIVERY:
							if( currentLiftHeight > 10 ) {
								cycleTurretHeading = robot.getJunctionTurretHeading( (red && right) || (!red && !right) ? 1 : -1, 0 );
								robot.turret.setTargetHeading( cycleTurretHeading + 8 );
								robot.linkage.moveToExtensionDistance( 6 );
								cycleState = CycleState.TURN_TURRET_TO_DELIVERY;
							}
							break;
						case TURN_TURRET_TO_DELIVERY:
							if( Math.abs( currentLiftHeight - cycleLiftHeight ) < 1 && (turretHeading < robot.turret.getTargetHeading( ) + 1 && turretHeading > robot.turret.getTargetHeading( ) - 1) ) {
								timer.reset( );
								robot.linkage.moveToExtensionDistance( 14 );
								cycleState = EXTEND_LINKAGE_AGAIN;
							}
							break;
						case EXTEND_LINKAGE_AGAIN:
							if( timer.milliseconds( ) > 250 ) {
								timer.reset( );
								robot.claw.open( );
								coneStackHeight -= 0.6; // 0.8
								if( coneStackHeight < 0 ) {
									autoState = AutoState.PARK;
									break;
								}

								cycleState = CycleState.DROP_CONE;
							}
							break;
						case LIFT_LIFT_A_BIT:
							if( timer.milliseconds( ) > 250 ) {
								timer.reset( );
								robot.setLiftTargetInches( robot.leftLift.getTarget( ) + 3 );
							}
						case DROP_CONE:
							if( timer.milliseconds( ) > 500 ) {
								robot.setLiftTargetInches( coneStackHeight );
								robot.linkage.moveToExtensionDistance( 0 );
								robot.turret.setTargetHeading( 0 );
								cycleState = CycleState.EXTEND_LINKAGE;
							}
							break;
					}
					break;
				case PARK:
					switch( parkState ) {
						case RESET:
							robot.turret.setTargetHeading( 0 );
							robot.setLiftTargetInches( 1 );
							robot.linkage.moveToExtensionDistance( 0 );
							parkState = ParkState.PARK;
							break;
						case PARK:
							if(robot.turret.getTurretHeading() > -25 ) {
								robot.turret.motor.setMotorDisable();
								robot.leftLift.motor.setMotorDisable();
								robot.rightLift.motor.setMotorDisable();

//								robot.drive.setPoseEstimate( parkStartPos );
								if( signalPos == AprilTagDetectionPipeline.SignalPosition.LEFT )
									robot.drive.followTrajectorySequence( leftParkTrajectory );
								else if( signalPos == AprilTagDetectionPipeline.SignalPosition.RIGHT )
									robot.drive.followTrajectorySequence( rightParkTrajectory );
								else
									robot.drive.followTrajectorySequence( middleParkTrajectory );
								parkState = ParkState.HALT;
							}
							break;
						case HALT:
							break;
					}
				break;
			}
		}
	}

	public void displayTelemetry( ) {
		telemetry.addData( "autoState", autoState );
		telemetry.addData( "cycleState", cycleState );
		telemetry.addData( "turret target heading", robot.turret.getTargetHeading( ) );
		telemetry.addData( "turret heading", turretHeading );
		telemetry.addData( "left lift target", robot.leftLift.getTargetPositionInch( ) );
		telemetry.addData( "left lift position", robot.leftLift.getMotorPositionInch( ) );
		telemetry.addData( "right lift target", robot.rightLift.getTargetPositionInch( ) );
		telemetry.addData( "right lift position", robot.rightLift.getMotorPositionInch( ) );
		telemetry.addData( "linkage position", robot.linkage.getExtensionDistance( ) );
		telemetry.addData( "L motor power", robot.leftLift.getPower( ) );
		telemetry.addData( "R motor power", robot.rightLift.getPower( ) );
		telemetry.addData( "cone stack height", coneStackHeight );
		telemetry.addData( "determined turret cycle angle", cycleTurretHeading );
		telemetry.addData( "time", timer.nanoseconds( ) );
		telemetry.update( );
	}
}
