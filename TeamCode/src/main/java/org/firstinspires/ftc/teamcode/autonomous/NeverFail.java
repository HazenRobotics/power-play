package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.roadrunner.MecanumDriveMini;
import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.vision.SignalUtil;
import org.firstinspires.ftc.teamcode.vision.pipelines.SignalDetector;

@Autonomous(name = "NeverFail", group = "Autonomous")
//@Disabled
public class NeverFail extends LinearOpMode {

	MiniBot robot;
	MecanumDrive drive;

	@Override
	public void runOpMode( ) {

		robot = new MiniBot( this );
		drive = new MecanumDrive( hardwareMap );
		SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
		detector.init( );

		DcMotorEx[] motors = {
				drive.frontLeft,
				drive.backLeft,
				drive.frontRight,
				drive.backRight,
		};
		for( DcMotorEx d : motors ) {
			d.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		}
		for( DcMotorEx d : motors ) {
			d.setTargetPosition( drive.convertDistTicks( 24 ) );
		}
		SignalDetector.SignalPosition park;
		while( !isStopRequested( ) && !isStarted( ) ) {
			telemetry.addData( "Element position", robot.signalUtil.getSignalPosition( ) );
			telemetry.update( );
		}
		waitForStart( );
		park = robot.signalUtil.getSignalPosition( );
		for( DcMotorEx d : motors ) {
			d.setPower( 0.25 );
		}
		while( motors[0].isBusy( ) ) ;

		if( park != SignalDetector.SignalPosition.MIDDLE ) {
			for( DcMotorEx d : motors ) {
				d.setMode( DcMotor.RunMode.RESET_ENCODERS );
			}
			for( DcMotorEx d : motors ) {
				d.setMode( DcMotor.RunMode.RUN_TO_POSITION );
			}
			for( DcMotorEx d : motors ) {
				d.setTargetPosition( drive.convertDistTicks( 24 ) );
			}
			if( park == SignalDetector.SignalPosition.LEFT ) {
				motors[0].setPower( -0.25 );
				motors[1].setPower( 0.25 );
				motors[2].setPower( -0.25 );
				motors[3].setPower( 0.25 );
			}
			if( park == SignalDetector.SignalPosition.RIGHT ) {
				motors[0].setPower( 0.25 );
				motors[1].setPower( -0.25 );
				motors[2].setPower( 0.25 );
				motors[3].setPower( -0.25 );
			}
			while( motors[0].isBusy( ) ) ;
		}
	}

}


