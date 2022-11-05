package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.localization.PPField;

@TeleOp(name = "MiniTeleOp", group = "TeleOp")

public class MiniTeleOp extends OpMode {

	MiniBot robot;
	GamepadEvents controller1;
	GamepadEvents controller2;
	boolean opened = true;

	BNO055IMU imu;

	@Override
	public void init( ) {

		controller1 = new GamepadEvents( gamepad1 );
		controller2 = new GamepadEvents( gamepad2 );

		telemetry.addData( "Mode", "Initiating robot..." );
		telemetry.update( );

		imu = hardwareMap.get( BNO055IMU.class, "imu" );
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters( );
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize( parameters );
//		BNO055IMUUtil.remapZAxis( imu, AxisDirection.POS_Y );

		robot = new MiniBot( this );
		robot.verticalLift.setEncoder( Lift.EncoderState.WITHOUT_ENCODER );

		telemetry.addData( "Mode", "waiting for start??" );
		telemetry.update( );
		controller1.update( );
		controller2.update( );

//		robot.mecanumDrive.setMotorPower( 0.35, 0, 0, 0 );
//		waitRobot( 1000 );
//		robot.mecanumDrive.setMotorPower( 0, 0.35, 0, 0 );
//		waitRobot( 1000 );
//		robot.mecanumDrive.setMotorPower( 0, 0, 0.35, 0 );
//		waitRobot( 1000 );
//		robot.mecanumDrive.setMotorPower( 0, 0, 0, 0.35 );
//		waitRobot( 1000 );
//		robot.mecanumDrive.setMotorPower( 0, 0, 0, 0 );
	}

	@Override
	public void loop( ) {

		robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
		robot.verticalLift.setPower( (gamepad1.right_trigger - gamepad1.left_trigger) + (gamepad2.right_trigger - gamepad2.left_trigger) );

		// g1/g2 a: toggle claw
		if( controller1.a.onPress( ) || controller2.a.onPress( ) ) {
			telemetry.addLine( "on a press" );
			if( opened )
				robot.claw.close( );
			else
				robot.claw.open( );
			opened = !opened;
		}

		// dpad: auto lift positions
		dpadToLiftPos( );

		displayTelemetry( );
		controller1.update( );
		controller2.update( );
	}

	public void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) )
			telemetry.update( );
	}

	public void displayTelemetry( ) {
		telemetry.addData( "angles:", imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES ) );
		telemetry.addLine( );
		telemetry.addData( "ly", -gamepad1.left_stick_y );
		telemetry.addData( "lx", gamepad1.left_stick_x );
		telemetry.addData( "rx", gamepad1.right_stick_x );
		telemetry.addLine( );
		telemetry.addData( "touchpad 1x", gamepad1.touchpad_finger_1_x );
		telemetry.addData( "touchpad 1y", gamepad1.touchpad_finger_1_y );
		telemetry.addData( "touchpad 2x", gamepad1.touchpad_finger_2_x );
		telemetry.addData( "touchpad 2y", gamepad1.touchpad_finger_2_y );
		telemetry.addLine( );
		telemetry.addData( "heading x", imu.getAngularVelocity( ).xRotationRate );
		telemetry.addData( "heading y*", imu.getAngularVelocity( ).yRotationRate );
		telemetry.addData( "heading z", imu.getAngularVelocity( ).zRotationRate );
		telemetry.addLine( );
		telemetry.addLine( "Docs:\nGamepad1: Drive\nGamepad2: Turret = right stick \nBoth: Lift = Triggers (left down right up)\nClaw = A" );

		telemetry.update( );
	}

	public void dpadToLiftPos( ) {
		if( controller1.dpad_up.onPress( ) || controller2.dpad_up.onPress( ) )
			robot.junctionToLiftPos( PPField.Junction.HIGH );
		if( controller1.dpad_down.onPress( ) || controller2.dpad_down.onPress( ) )
			robot.junctionToLiftPos( PPField.Junction.MEDIUM );
		if( controller1.dpad_left.onPress( ) || controller2.dpad_left.onPress( ) )
			robot.junctionToLiftPos( PPField.Junction.LOW );
		if( controller1.dpad_right.onPress( ) || controller2.dpad_right.onPress( ) )
			robot.junctionToLiftPos( PPField.Junction.GROUND );
	}

}