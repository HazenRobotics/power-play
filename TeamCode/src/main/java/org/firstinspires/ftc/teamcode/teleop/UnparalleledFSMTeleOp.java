package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.util.Vector;

@TeleOp(name = "🎮 SM Mini TeleOP", group = "ANewTeleOp")
//@Disabled
public class UnparalleledFSMTeleOp extends LinearOpMode {

	MiniBot robot;
	GamepadEvents controller1;
	GamepadEvents controller2;

	double linkagePos = 0;
	ElapsedTime timer = new ElapsedTime( );
	double loopTime = 0;
	int junctionHeightIndex = 0;
	double turretTarget;
	double linkageDeliveryPosition = 11;
	double turretDeliveryPosition = -139;
	double tilt;
	double heading;
	boolean fieldCentric = false;

	public enum Speeds {

		DRIVE( 0.5, 1 ),
		STRAFE( 0.5, 1 ),
		ROTATE( 0.5, 1 );

		Speeds( double min, double max ) {
			this.min = min;
			this.max = max;
		}

		private final double min;
		private final double max;

		public double min( ) {
			return min;
		}

		public double max( ) {
			return max;
		}

		public double speed( Gamepad gamepad ) {
			if( this == ROTATE )
				return gamepad.right_stick_button ? max( ) : min( );
			return gamepad.left_stick_button ? max( ) : min( );
		}
	}

	public enum TeleOpStates {
		DRIVER_CONTROLLED,
		EXTEND_LINKAGE,
		GRAB_CONE,
		RETRACT_LINKAGE,
		EXTEND_LINKAGE_AGAIN,
		DROP_CONE,
		RESET
	}

	public enum PowerControl {
		USING_PID,
		POWER_BASED
	}

	TeleOpStates opModeState = TeleOpStates.DRIVER_CONTROLLED;
	PowerControl liftState = PowerControl.POWER_BASED;
	PowerControl turretState = PowerControl.POWER_BASED;


	@Override
	public void runOpMode( ) throws InterruptedException {
		controller1 = new GamepadEvents( gamepad1 );
		controller2 = new GamepadEvents( gamepad2 );

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );


		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		robot = new MiniBot( this );
		robot.claw.init( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( MiniBot.endAutoPos == null ? new Pose2d( 0, 0, 0 ) : MiniBot.endAutoPos );

		telemetry.addData( "Mode", "waiting for start??" );
		timer.reset( );
		telemetry.update( );
		controller1.update( );
		controller2.update( );

		waitForStart( );

		while( opModeIsActive( ) ) {
			if ( gamepad1.ps ) {
				robot.leftLift.resetLift();
				robot.rightLift.resetLift();
			}
			if( controller1.y.onPress( ) && opModeState != TeleOpStates.DRIVER_CONTROLLED ) {
				opModeState = TeleOpStates.DRIVER_CONTROLLED;
				liftState = PowerControl.POWER_BASED;
				turretState = PowerControl.POWER_BASED;
				linkagePos = robot.linkage.getExtensionDistance( );
			}

			tilt = robot.drive.imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES ).secondAngle;

			switch( opModeState ) {
				case DRIVER_CONTROLLED:
					subsystemControls( );
//					if( controller1.x.onPress( ) ) {
//						timer.reset( );
//						linkagePos = 3;
//						liftState = PowerControl.USING_PID;
//						turretState = PowerControl.USING_PID;
//						robot.	leftLift.setTargetInches( 0 );
//						robot.rightLift.setTargetInches( 0 );
//						robot.turret.setTarget( 0 );
//						opModeState = TeleOpStates.RESET;
//					}
					break;
				case RESET:
					if( robot.leftLift.getMotorPositionInch( ) < 1.5 && (robot.turret.getTurretHeading( ) < 3 && robot.turret.getTurretHeading( ) > -2) ) {
						robot.claw.open( );
						linkagePos = 14;
						timer.reset( );
						opModeState = TeleOpStates.EXTEND_LINKAGE;
					}
					break;
				case EXTEND_LINKAGE:
					if( controller1.a.onPress( ) ) {
						robot.claw.close( );
						timer.reset( );
						opModeState = TeleOpStates.GRAB_CONE;
					}
					break;
				case GRAB_CONE:
					if( timer.nanoseconds( ) > 0.35 * 1000000000 ) {
						linkagePos = 2;
						timer.reset( );
						opModeState = TeleOpStates.RETRACT_LINKAGE;
					}
					break;
				case RETRACT_LINKAGE:
					if( timer.nanoseconds( ) > 0.15 * 1000000000 ) {
						robot.leftLift.setTargetInches( robot.liftHeights[3] + 3.5 );
						robot.rightLift.setTargetInches( robot.liftHeights[3] + 3.5 );
						robot.turret.setTargetHeading( turretDeliveryPosition );
						timer.reset( );

						opModeState = TeleOpStates.EXTEND_LINKAGE_AGAIN;
					}
					break;
				case EXTEND_LINKAGE_AGAIN:
					if( timer.nanoseconds( ) > 0.2 * 1000000000 ) {
						linkagePos = linkageDeliveryPosition;
						opModeState = TeleOpStates.DROP_CONE;
					}
					break;
				case DROP_CONE:
					linkageDeliveryPosition += (gamepad1.right_bumper ? 0.2 : 0) - (gamepad1.left_bumper ? 0.2 : 0);
					linkageDeliveryPosition = Math.min( robot.linkage.extensionLength, Math.max( linkageDeliveryPosition, robot.linkage.retractionLength ) );
					linkagePos = linkageDeliveryPosition;

					turretDeliveryPosition += (controller1.dpad_right.onPress( ) ? 2 : 0) - (controller1.dpad_left.onPress( ) ? 2 : 0);
					robot.turret.setTargetHeading( turretDeliveryPosition );

					if( controller1.a.onPress( ) ) {
						robot.claw.open( );
						opModeState = TeleOpStates.DRIVER_CONTROLLED;
					}
					break;
				default:
					opModeState = TeleOpStates.DRIVER_CONTROLLED;
			}

			if( liftState == PowerControl.USING_PID ) {
				robot.leftLift.updatePID( 1 );
				robot.rightLift.updatePID( 1 );
			}
			if( turretState == PowerControl.USING_PID ) {
				robot.turret.updatePID( 0.5 );
			}
//among us

			robot.angler.pointCameraToPosition( robot.linkage.getExtensionDistance( ) + MiniBot.CLAW_CAMERA_OFFSET, robot.leftLift.getMotorPositionInch( ) + 0.001 );
			robot.linkage.moveToExtensionDistance( linkagePos );
			displayTelemetry( );
			driveRobot( );
			updateControllers( );
		}
	}

	public void driveRobot( ) {

		if( Math.abs( tilt + 90 ) > 10 ) {
			robot.drive.setWeightedDrivePower(
					new Pose2d(
							Math.signum( tilt + 90 ) * Drive.normalize( Math.abs( tilt + 90 ), 0, 70, 0, 0.8 ),
							0,
							0
					)
			);
		} else {
			robot.drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y * Speeds.DRIVE.speed( gamepad1 ),
							-gamepad1.left_stick_x * Speeds.STRAFE.speed( gamepad1 ),
							-gamepad1.right_stick_x * Speeds.ROTATE.speed( gamepad1 )
					)
			);
		}
	}

	public void updateLinkagePos( ) {
		linkagePos += (gamepad2.right_bumper ? 0.2 : 0) - (gamepad2.left_bumper ? 0.2 : 0);
		linkagePos += (controller1.left_bumper.onPress( ) ? -robot.linkage.extensionLength / 2 : (controller1.right_bumper.onPress( ) ? robot.linkage.extensionLength / 2 : 0));
		linkagePos = Math.min( robot.linkage.extensionLength, Math.max( linkagePos, robot.linkage.retractionLength ) );
		robot.linkage.moveToExtensionDistance( linkagePos );
	}

	public void subsystemControls( ) {
		if( liftState != PowerControl.USING_PID ) {
			robot.liftPower( ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75) - ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.25) /*+ power*/ );
		} else if( gamepad1.right_trigger + gamepad1.left_trigger + gamepad2.right_trigger + gamepad2.left_trigger > 0.05 ) {
			liftState = PowerControl.POWER_BASED;
		}

//		if( controller2.x.onPress( ) ) {
//			robot.turret.resetTurret( );
//		}

//		if( turretState != PowerControl.USING_PID ) {
//			robot.turret.setTurretPower( controller2.right_stick_x * 0.5 );
//		} else if( gamepad2.right_stick_x > 0.1 ) {
//			turretState = PowerControl.POWER_BASED;
//		}

		robot.turret.setTurretPower( (gamepad1.dpad_right ? 0.15 : 0) - (gamepad1.dpad_left ? 0.15 : 0) );

		if( controller1.a.onPress( ) || controller2.a.onPress( ) )
			robot.claw.toggle( );

//		if( controller1.dpad_up.onPress( ) || controller1.dpad_down.onPress( ) || controller2.dpad_up.onPress( ) || controller2.dpad_down.onPress( ) ) {
//			dpadToLiftPos( );
//		}

//		if( controller2.dpad_left.onPress( ) || controller2.dpad_right.onPress( ) || controller2.y.onPress( ) ) {
//			dpadToTurretPos( );
//		}

		if (controller1.y.onPress())
			fieldCentric = !fieldCentric;

		updateLinkagePos( );
	}

	public void updateControllers( ) {
		controller1.update( );
		controller2.update( );
	}

	public void displayTelemetry( ) {
//		telemetry.addData( "angles:", imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES ) );
//		telemetry.addLine( );
//		telemetry.addData( "ly", -gamepad1.left_stick_y );
//		telemetry.addData( "lx", gamepad1.left_stick_x );
//		telemetry.addData( "rx", gamepad1.right_stick_x );
//		telemetry.addLine( );
//		telemetry.addData( "frontLeft pos", gamepad1.touchpad_finger_1_x );
//		telemetry.addData( "backLeft pos", gamepad1.touchpad_finger_1_y );
//		telemetry.addData( "frontRight pos", gamepad1.touchpad_finger_2_x );
//		telemetry.addData( "backRight pos", gamepad1.touchpad_finger_2_y );
//		telemetry.addLine( );
//		telemetry.addData( "touchpad 1x", gamepad1.touchpad_finger_1_x );
//		telemetry.addData( "touchpad 1y", gamepad1.touchpad_finger_1_y );
//		telemetry.addData( "touchpad 2x", gamepad1.touchpad_finger_2_x );
//		telemetry.addData( "touchpad 2y", gamepad1.touchpad_finger_2_y );
//		telemetry.addLine( );
//		telemetry.addData( "heading x", imu.getAngularVelocity().xRotationRate );
//		telemetry.addData( "heading y*", imu.getAngularVelocity( ).yRotationRate );
//		telemetry.addData( "heading z", imu.getAngularVelocity( ).zRotationRate );
//		telemetry.addLine( );

////		telemetry.addData( "power shift", power );
//		telemetry.addData( "currentL", robot.leftLift.getCurrent( CurrentUnit.AMPS ) );
//		telemetry.addData( "currentR", robot.rightLift.getCurrent( CurrentUnit.AMPS ) );
//		telemetry.addData( "powerL", robot.leftLift.getPower( ) );
//		telemetry.addData( "powerR", robot.rightLift.getPower( ) );
////		telemetry.addData( "velocity", robot.lift.getVelocity( ) );
		telemetry.addData( "pos (ticks) L", robot.leftLift.getMotorPosition( ) );
		telemetry.addData( "pos (ticks) R", robot.rightLift.getMotorPosition( ) );

//		telemetry.addLine( );

//		telemetry.addData( "gyro", robot.gyro.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES ) );
//		telemetry.addData( "isOverJunction", robot.isOverJunction( ) );
//		telemetry.addData( "pos", robot.drive.getPoseEstimate( ) );
//	telemetry.addData( "powerOveridden", powerOverridden );
		telemetry.addData( "turret heading", robot.turret.getTurretHeading( ) );
//		telemetry.addData( "left limit", robot.turret.getLeftLimit( ) );
//		telemetry.addData( "right limit", robot.turret.getRightLimit( ) );
//		telemetry.addData( "too far left:", robot.turret.getTurretHeading( ) < robot.turret.getLeftLimit( ) );
//		telemetry.addData( "too far right:", robot.turret.getTurretHeading( ) > robot.turret.getRightLimit( ) );

		telemetry.addData( "linkagePosition", linkagePos );
//		telemetry.addData( "current state", opModeState );
//		telemetry.addData( "lift target", robot.leftLift.getTargetPosition() );
//		telemetry.addData( "turret target", robot.turret.getTargetPosition() );

//		telemetry.addData( "linkage distance from camera", robot.linkage.getExtensionDistance( ) + MiniBot.CLAW_CAMERA_OFFSET );
//		telemetry.addData( "lift distance from servo", robot.leftLift.getMotorPositionInch( ) );
//		telemetry.addData( "intended angle", Math.toDegrees( Math.atan( robot.linkage.getExtensionDistance( ) + MiniBot.CLAW_CAMERA_OFFSET / (robot.leftLift.getMotorPositionInch( ) + 0.001) ) ) );
//		telemetry.addData( "actual angle", robot.angler.servoToAngle( robot.angler.servo.getPosition( ) ) );
//		telemetry.addData( "servo pos", robot.angler.servo.getPosition( ) );

		telemetry.addData( "tilt angle", tilt );
		telemetry.addData("turret power", robot.turret.getPower());

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;
//		telemetry.update( );

//		telemetry.addLine( "Docs:\nDrive:\nNormal mecanum drive\nSubSystems:\nA = Claw\nTriggers = Lift Up and Down" );

		telemetry.addData( "poseX", robot.drive.getPoseEstimate().getX());
		telemetry.addData( "poseY", robot.drive.getPoseEstimate().getY());
		telemetry.addData( "heading", robot.drive.getPoseEstimate().getHeading());

		telemetry.addData( "gyro data", robot.gyro.getAngularOrientation());


		telemetry.update( );
	}

	public void dpadToLiftPos( ) {

		liftState = PowerControl.USING_PID;

		if( (controller1.dpad_up.onPress( ) || controller2.dpad_up.onPress( ))  && junctionHeightIndex < robot.liftHeights.length - 1 ) {
			junctionHeightIndex++;
		} else if( (controller1.dpad_down.onPress( ) || controller2.dpad_down.onPress( )) && junctionHeightIndex > 0 ) {
			junctionHeightIndex--;
		}

		robot.setLiftTargetInches( robot.liftHeights[junctionHeightIndex] + (junctionHeightIndex != 0 ? 1.75 : 0) );
	}

	public void dpadToTurretPos( ) {
		turretState = PowerControl.USING_PID;

		turretTarget = robot.turret.getTurretHeading( );

		double roundedTarget = Math.round( turretTarget / 45 ) * 45;

		if( controller2.dpad_left.onPress( ) ) {
			turretTarget = roundedTarget - 45;
		} else if( controller2.dpad_right.onPress( ) ) {
			turretTarget = roundedTarget + 45;
		}


		robot.turret.setTargetHeading( turretTarget );
	}
}
