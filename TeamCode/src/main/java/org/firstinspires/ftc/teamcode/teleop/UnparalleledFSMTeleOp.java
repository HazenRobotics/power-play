package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@TeleOp(name = "🎮 SM Mini TeleOP", group = "ANewTeleOp")
//@Disabled
public class UnparalleledFSMTeleOp extends LinearOpMode {

	MiniBot robot;
	GamepadEvents controller1;
	GamepadEvents controller2;
	boolean movingLift = false;
	boolean movingTurret = false;
	double maxCurrent = 5;
	double robotTiltAngle = 0;
	double linkagePos = 0;
	ElapsedTime timer = new ElapsedTime( );
	double loopTime = 0;
	int linkageTogglePos = 0;
	int junctionHeightIndex = 0;

	public enum Speeds {

		DRIVE( 0.6, 0.8 ),
		STRAFE( 1.0, 1.0 ),
		ROTATE( .4, 0.9 );

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
		DELIVER_CONE,
		DROP_CONE,
		RESET
	}

	TeleOpStates state = TeleOpStates.DRIVER_CONTROLLED;

	@Override
	public void runOpMode( ) throws InterruptedException {
		controller1 = new GamepadEvents( gamepad1 );
		controller2 = new GamepadEvents( gamepad2 );

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		robot = new MiniBot( this );
		robot.leftLift.setEncoder( Lift.EncoderState.WITH_ENCODER );
		robot.rightLift.setEncoder( Lift.EncoderState.WITH_ENCODER );
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
			switch( state ) {
				case DRIVER_CONTROLLED:
					subsystemControls( );
//					if ( controller1.b.onPress() ) {
//						timer.reset();
//						lift.target = 0;
//						turret.target = 0;
//						state = TeleOpStates.RESET;
//					}
					break;
				case RESET:
					// add the PID loops for the lift and turret to actually be efficient and not rely on threads
//					if (Math.abs(lift.getPos - grabConeLevel) < 10 && Math.abs(robot.turret.getTurretHeading() - 2) < 4) {
//					    robot.claw.open();
//						robot.linkage.moveToExtensionDistance( 14 );
//					    timer.reset();
//						state = TeleOpStates.EXTEND_LINKAGE;
//					}
					break;
				case EXTEND_LINKAGE:
					if( timer.nanoseconds( ) > 2 * 1000000000 && controller1.a.onPress( ) ) {
						robot.claw.close( );
						timer.reset( );
						state = TeleOpStates.GRAB_CONE;
					}
					break;
				case GRAB_CONE:
					if( timer.nanoseconds( ) > 0.15 * 1000000000 ) {
//						lift.target = lift.HIGH_POS.height;
//					    turret.target = turret.CYCLE_POS;
					}
					break;
				case DELIVER_CONE:
					break;
				case DROP_CONE:
					break;
				default:
					state = TeleOpStates.DRIVER_CONTROLLED;
			}

			if( gamepad1.y && state != TeleOpStates.DRIVER_CONTROLLED ) {
				state = TeleOpStates.DRIVER_CONTROLLED;
//				lift.target = 0;
//				turret.target = 0;
			}

			if( state != TeleOpStates.DRIVER_CONTROLLED ) {
				updatePID( );
			}

			driveRobot( );
		}
	}

	public void driveRobot( ) {
		robot.drive.setWeightedDrivePower(
				new Pose2d(
						-gamepad1.left_stick_y * Speeds.DRIVE.speed( gamepad1 ),
						-gamepad1.left_stick_x * Speeds.STRAFE.speed( gamepad1 ),
						-gamepad1.right_stick_x * Speeds.ROTATE.speed( gamepad1 )
				)
		);
	}

	public void updatePID( ) {
		// update the PID loops for the lift and turret
//		lift.setPower(liftPIDController.calc(liftHeight, liftTarget));
//		turret.setPower(turretPIDController.calc(turretHeading, turretTarget));
	}

	public void subsystemControls( ) {
		if( !movingLift ) {
			robot.liftPower( (gamepad1.right_trigger + gamepad2.right_trigger) - ((gamepad1.left_trigger + gamepad2.left_trigger)) /*+ power*/ );
		} else if( gamepad1.right_trigger + gamepad1.left_trigger + gamepad2.right_trigger + gamepad2.left_trigger > 0.05 ) {
			movingLift = false;
			robot.leftLift.setTeleOpPowerMode( );
			robot.rightLift.setTeleOpPowerMode( );
		}

		if( controller2.x.onPress( ) )
			robot.turret.resetTurret( );

		if( !movingTurret ) {
			robot.turret.setTurretPower( controller2.right_stick_x * 0.5 );
		} else if( gamepad2.right_stick_x > 0.1 ) {
			movingTurret = false;
			robot.turret.motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		}

		if( controller1.a.onPress( ) || controller2.a.onPress( ) )
			robot.claw.toggle( );

		linkagePos += ((/*gamepad1.right_bumper ||*/ gamepad2.right_bumper ? 0.5 : 0) - (/*gamepad1.left_bumper ||*/ gamepad2.left_bumper ? 0.5 : 0));
		linkagePos += (controller1.left_bumper.onPress( ) ? -robot.linkage.extensionLength / 2 : (controller1.right_bumper.onPress( ) ? robot.linkage.extensionLength / 2 : 0));
//		linkagePos += (controller1.right_bumper.onPress( ) ? robot.linkage.extensionLength / 2 : (controller1.right_bumper.onPress( ) ? -robot.linkage.extensionLength / 2 : 0));
		linkagePos = Math.min( robot.linkage.extensionLength, Math.max( linkagePos, robot.linkage.retractionLength ) );
		robot.linkage.moveToExtensionDistance( linkagePos );


		dpadToLiftPos( );
		dpadToTurretPos( );

		displayTelemetry( );
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

//		telemetry.addData( "power shift", power );
		telemetry.addData( "currentL", robot.leftLift.getCurrent( CurrentUnit.AMPS ) );
		telemetry.addData( "currentR", robot.rightLift.getCurrent( CurrentUnit.AMPS ) );
		telemetry.addData( "powerL", robot.leftLift.getPower( ) );
		telemetry.addData( "powerR", robot.rightLift.getPower( ) );
//		telemetry.addData( "velocity", robot.lift.getVelocity( ) );
		telemetry.addData( "pos (ticks) L", robot.leftLift.motor.getCurrentPosition( ) );
		telemetry.addData( "pos (in) L", robot.rightLift.getPositionInch( ) );
		telemetry.addData( "pos (ticks) R", robot.rightLift.motor.getCurrentPosition( ) );
		telemetry.addData( "pos (in) R", robot.rightLift.getPositionInch( ) );
		telemetry.addData( "target pos (in) L", robot.leftLift.getTargetPosition( ) );
		telemetry.addData( "target pos (in) R", robot.rightLift.getTargetPosition( ) );

//		telemetry.addLine( );

//		telemetry.addData( "gyro", robot.gyro.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES ) );
//		telemetry.addData( "isOverJunction", robot.isOverJunction( ) );
//		telemetry.addData( "pos", robot.drive.getPoseEstimate( ) );
//	telemetry.addData( "powerOveridden", powerOverridden );
//		telemetry.addData( "turret heading", robot.turret.getTurretHeading( ) );
//		telemetry.addData( "left limit", robot.turret.getLeftLimit( ) );
//		telemetry.addData( "right limit", robot.turret.getRightLimit( ) );
//		telemetry.addData( "too far left:", robot.turret.getTurretHeading( ) < robot.turret.getLeftLimit( ) );
//		telemetry.addData( "too far right:", robot.turret.getTurretHeading( ) > robot.turret.getRightLimit( ) );

		telemetry.addData( "linkagePosition", linkagePos );

		double loop = System.nanoTime( );
		telemetry.addData( "hz ", 1000000000 / (loop - loopTime) );
		loopTime = loop;
		telemetry.update( );

//		telemetry.addLine( "Docs:\nDrive:\nNormal mecanum drive\nSubSystems:\nA = Claw\nTriggers = Lift Up and Down" );

		telemetry.update( );
	}

	public void dpadToLiftPos( ) {
//		junctionHeightIndex += gam
//		if( controller1.dpad_up.onPress( ) /*|| controller2.dpad_up.onPress( )*/ ) {
//			movingLift = true;
//			robot.junctionToLiftPos( PPField.Junction.HIGH );
////			telemetry.addLine( "high: " + PPField.Junction.HIGH.height( ) );
//		}
//		if( controller1.dpad_left.onPress( ) /*|| controller2.dpad_left.onPress( )*/ ) {
//			movingLift = true;
//			robot.junctionToLiftPos( PPField.Junction.MEDIUM );
////			telemetry.addLine( "medium: " + PPField.Junction.MEDIUM.height( ) );
//		}
//		if( controller1.dpad_down.onPress( ) /*|| controller2.dpad_down.onPress( )*/ ) {
//			movingLift = true;
//			robot.junctionToLiftPos( PPField.Junction.LOW );
////			telemetry.addLine( "low: " + PPField.Junction.LOW.height( ) );
//		}
//		if( controller1.dpad_right.onPress( ) /*|| controller2.dpad_right.onPress( )*/ ) {
//			movingLift = true;
//			robot.junctionToLiftPos( PPField.Junction.GROUND );
////			telemetry.addLine( "ground: " + PPField.Junction.GROUND.height( ) );
//		}
	}

	public void dpadToTurretPos( ) {
		if( controller2.dpad_up.onPress( ) && controller2.dpad_right.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, 45 );
		} else if( controller2.dpad_up.onPress( ) && controller2.dpad_left.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, -45 );
		} else if( controller2.dpad_down.onPress( ) && controller2.dpad_left.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, -135 );
		} else if( controller2.dpad_down.onPress( ) && controller2.dpad_right.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, -225 );
		} else if( controller2.dpad_up.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, 0 );
		} else if( controller2.dpad_left.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, -90 );
		} else if( controller2.dpad_down.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, -180 );
		}
	}
}
