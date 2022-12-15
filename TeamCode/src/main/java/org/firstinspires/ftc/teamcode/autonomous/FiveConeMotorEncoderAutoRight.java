package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.InternalIMU;

@Autonomous(group = "")
//@Disabled
public class FiveConeMotorEncoderAutoRight extends LinearOpMode {

	MecanumDrive drive;
	InternalIMU gyro;

	@Override
	public void runOpMode( ) throws InterruptedException {
		drive = new MecanumDrive( hardwareMap );
		gyro = new InternalIMU( hardwareMap );

		drive.setWheelDiameter( 3.77953 );

		drive.setMotorDirections( DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD );

		waitForStart( );

		drive.turn( 0 );

		for( int i = 0; i < 5; i++ )
			cycle();

		while (!isStopRequested()) {
			displayTelemetry();
		}

	}

	public void displayTelemetry( ) {
		telemetry.addData( "fl", drive.convertTicksDist( drive.frontLeft.getCurrentPosition( ) ) );
		telemetry.addData( "bl", drive.convertTicksDist( drive.backLeft.getCurrentPosition( ) ) );
		telemetry.addData( "fr", drive.convertTicksDist( drive.frontRight.getCurrentPosition( ) ) );
		telemetry.addData( "br", drive.convertTicksDist( drive.backRight.getCurrentPosition( ) ) );
		telemetry.addData( "gyro", gyro.getOrientation() );
		telemetry.update();
	}

	public void waitRobot( int mills ) {
		long startTime = System.currentTimeMillis( );
		while( (startTime + mills) > System.currentTimeMillis( ) );
	}

	public void cycle() {
		drive.setRunMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		drive.setRunMode( DcMotor.RunMode.RUN_USING_ENCODER );

		drive.setTargetPosition( drive.convertDistTicks( 30 ) );

		drive.setRunMode( DcMotor.RunMode.RUN_TO_POSITION );

		drive.move( 0.25 );

		while( drive.frontLeft.isBusy( ) )
			displayTelemetry();

		drive.setRunMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		drive.setRunMode( DcMotor.RunMode.RUN_USING_ENCODER );

		drive.setTargetPosition( drive.convertDistTicks( -30 ) );

		drive.setRunMode( DcMotor.RunMode.RUN_TO_POSITION );

		drive.move( -0.25 );
	}

	public void getIntoPosition() {
		drive.setRunMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		drive.setTargetPosition( drive.convertDistTicks( 48 ) );

		drive.setRunMode( DcMotor.RunMode.RUN_TO_POSITION );

		drive.move( 0.25 );

		while( drive.frontLeft.isBusy( ) )
			displayTelemetry();

		drive.setRunMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		drive.setRunMode( DcMotor.RunMode.RUN_USING_ENCODER );

		drive.turn( 0.25 );

		while( gyro.getOrientation( ).firstAngle > -87 )
			displayTelemetry();
	}
}
