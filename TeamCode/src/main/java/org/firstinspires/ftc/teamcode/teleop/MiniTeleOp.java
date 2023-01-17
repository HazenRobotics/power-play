package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.RGBLights;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@TeleOp(name = "MiniTeleOp", group = "TeleOp")

public class MiniTeleOp extends OpMode {

	MiniBot robot;
	GamepadEvents controller1;
	GamepadEvents controller2;
	boolean opened = true;
	boolean movingLift = false;
	boolean movingTurret = false;
	double maxCurrent = 10;
	float power = 0.1f;
	//	boolean powerOverridden = false;
	Orientation gyroOrientation;
	double robotTiltAngle = 0;
	double robotHeading = 0;
	boolean fieldCentricTurret = false;
	ElapsedTime time = new ElapsedTime( );
	double linkagePosition = 0;

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

	@Override
	public void init( ) {

		controller1 = new GamepadEvents( gamepad1 );
		controller2 = new GamepadEvents( gamepad2 );

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		robot = new MiniBot( this );
		robot.leftLift.setEncoder( Lift.EncoderState.WITHOUT_ENCODER );
		robot.rightLift.setEncoder( Lift.EncoderState.WITHOUT_ENCODER );
		robot.claw.init( );

		robot.drive.setLocalizer( robot.drive.getLocalizer( ) );
		robot.drive.setPoseEstimate( MiniBot.endAutoPos == null ? new Pose2d( 0, 0, 0 ) : MiniBot.endAutoPos );

		telemetry.addData( "Mode", "waiting for start??" );
		time.reset( );
		telemetry.update( );
		controller1.update( );
		controller2.update( );
	}

	@Override
	public void loop( ) {

		gyroOrientation = robot.gyro.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );

		robotTiltAngle = gyroOrientation.secondAngle;
		robotHeading = gyroOrientation.firstAngle;

		if( controller2.y.onPress( ) )
			fieldCentricTurret = !fieldCentricTurret;

		if( controller2.x.onPress( ) )
			robot.turret.resetTurret( );


//		if( Math.abs( robotTiltAngle + 90 ) > 10 )
//			robot.mecanumDrive.drive( Math.signum( robotTiltAngle ) * Drive.normalize( Math.abs( robotTiltAngle ), 0, 70, 0, 0.8 ), 0 );
//		else
//		robot.mecanumDrive.drive( -gamepad1.left_stick_y * Speeds.DRIVE.speed( gamepad1 ),
//				gamepad1.left_stick_x * Speeds.STRAFE.speed( gamepad1 ),
//				gamepad1.right_stick_x * Speeds.ROTATE.speed( gamepad1 ) );
		robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

		if( !movingLift ) {
			robot.leftLift.setPower( (gamepad1.right_trigger /*+ gamepad2.right_trigger*/)  - gamepad1.left_trigger /* + gamepad2.left_trigger) + power*/ );
			robot.rightLift.setPower( (gamepad1.right_trigger /* + gamepad2.right_trigger*/) - gamepad1.left_trigger /* + gamepad2.left_trigger) + power*/ );
		} else if( gamepad1.right_trigger + gamepad1.left_trigger /*+ gamepad2.right_trigger + gamepad2.left_trigger*/ > 0.05 ) {
			movingLift = false;
			robot.leftLift.setTeleOpPowerMode( );
			robot.rightLift.setTeleOpPowerMode( );
		}

		if( !movingTurret ) {
			robot.turret.setTurretPower( controller2.right_stick_x * 0.5 );
		} else if( gamepad2.right_stick_x > 0.1 ) {
			movingTurret = false;
			robot.turret.motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		}

		// g1/g2 a: toggle claw
		if( controller1.a.onPress( ) || controller2.a.onPress( ) )
			robot.claw.toggle( );

		linkagePosition += ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.01);
		linkagePosition = Math.min(robot.linkage.maxRotationLimit, linkagePosition);
		robot.linkage.setPosition( linkagePosition );

		// g1/g2 bumpers: rotate claw
//		if( (gamepad1.right_bumper && gamepad1.left_bumper) || (gamepad2.right_bumper && gamepad2.left_bumper) )
//			robot.claw.setState( TwoAxesClaw.HorizontalClawState.CENTER );
//		else if( controller1.left_bumper.onPress( ) || controller2.left_bumper.onPress( ) )
//			robot.claw.setState( TwoAxesClaw.HorizontalClawState.LEFT );
//		else if( controller1.right_bumper.onPress( ) || controller2.right_bumper.onPress( ) )
//			robot.claw.setState( TiltingClaw.VerticalClawState.RIGHT );

//		// g2 dpad: tilt claw
//		if( controller2.dpad_up.onPress( ) )
//			robot.claw.setState( TiltingClaw.VerticalClawState.STOWED );
//		else if( controller2.dpad_left.onPress( ) )
//			robot.claw.setState( TiltingClaw.VerticalClawState.DEPLOYED );
//		else if( controller2.dpad_down.onPress( ) )
//			robot.claw.setState( TiltingClaw.VerticalClawState.PICKUP );

		// g2 right stick X: rotate turret

//		if( fieldCentricTurret )
//			robot.turret.setLiveRotationPower( new Vector2d( controller2.right_stick_x, -controller2.right_stick_y ), robotHeading );
//		else
//			robot.turret.setPower( controller2.right_stick_x * 0.5 );

		// dpad: auto lift positions
		dpadToLiftPos( );
		dpadToLiftPos();

		if( controller1.b.onPress( ) ) {
			power += 0.05f;
		} else if( controller1.x.onPress( ) ) {
			power -= 0.05f;
		}

		// reset the lift position to its current zero position
		if( gamepad1.ps || (robot.leftLift.getCurrent( CurrentUnit.AMPS ) > maxCurrent && !movingLift) ) {
//			maxCurrent = robot.lift.getCurrent( CurrentUnit.AMPS );
			Robot.writeToDefaultFile( "Current: " + maxCurrent, true, false );
			robot.leftLift.resetLift( );
			robot.rightLift.resetLift( );
		}

		if( robot.isOverJunction( ) && !gamepad1.isRumbling( ) ) {
			gamepad1.rumble( 100 );
		} else if( !robot.isOverJunction( ) ) {
			gamepad1.stopRumble( );
		}

		if( time.startTime( ) / 1000 > 110 ) {
			robot.lights.showStatus( RGBLights.StatusLights.CELEBRATION );
			telemetry.addLine( "End Game started" );
		}

		if( robot.inSubstation( ) ) {
			robot.lights.showStatus( RGBLights.StatusLights.ERROR );
			telemetry.addLine( "Over substation" );
		}
		// update controllers and telemetry
		displayTelemetry( );
		controller1.update( );
		controller2.update( );
		// nothing else after this line

	}

	public static void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) );
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
//		telemetry.addData( "current", robot.lift.getCurrent( CurrentUnit.AMPS ) );
//		telemetry.addData( "power", robot.lift.getPower( ) );
//		telemetry.addData( "velocity", robot.lift.getVelocity( ) );
//		telemetry.addData( "pos (ticks)", robot.lift.getPosition( ) );
//		telemetry.addData( "pos (in)", robot.lift.getPositionInch( ) );
//		telemetry.addData( "target pos (in)", robot.lift.getTargetPositionInch( ) );
//		telemetry.addLine( );

//		telemetry.addData( "gyro", robot.gyro.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES ) );
		telemetry.addData( "isOverJunction", robot.isOverJunction( ) );
		telemetry.addData( "pos", robot.drive.getPoseEstimate( ) );
//		telemetry.addData( "powerOveridden", powerOverridden );

		telemetry.addData( "linkagePosition", linkagePosition );


//		telemetry.addLine( "Docs:\nDrive:\nNormal mecanum drive\nSubSystems:\nA = Claw\nTriggers = Lift Up and Down" );

		telemetry.update( );
	}

	public void dpadToLiftPos( ) {
		if( controller1.dpad_up.onPress( ) /*|| controller2.dpad_up.onPress( )*/ ) {
			movingLift = true;
			robot.junctionToLiftPos( PPField.Junction.HIGH );
//			telemetry.addLine( "high: " + PPField.Junction.HIGH.height( ) );
		}
		if( controller1.dpad_left.onPress( ) /*|| controller2.dpad_left.onPress( )*/ ) {
			movingLift = true;
			robot.junctionToLiftPos( PPField.Junction.MEDIUM );
//			telemetry.addLine( "medium: " + PPField.Junction.MEDIUM.height( ) );
		}
		if( controller1.dpad_down.onPress( ) /*|| controller2.dpad_down.onPress( )*/ ) {
			movingLift = true;
			robot.junctionToLiftPos( PPField.Junction.LOW );
//			telemetry.addLine( "low: " + PPField.Junction.LOW.height( ) );
		}
		if( controller1.dpad_right.onPress( ) /*|| controller2.dpad_right.onPress( )*/ ) {
			movingLift = true;
			robot.junctionToLiftPos( PPField.Junction.GROUND );
//			telemetry.addLine( "ground: " + PPField.Junction.GROUND.height( ) );
		}
	}

	public void dpadToTurretPos( ) {
		if( controller2.dpad_up.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, 0 );
		}
		if( controller2.dpad_left.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, 90 );
		}
		if( controller2.dpad_down.onPress( ) ) {
			movingTurret = true;
			robot.turret.setRotationPower( 0.5, 180 );
		}
	}

}
