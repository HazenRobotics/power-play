package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robots.MiniBot;
import org.firstinspires.ftc.teamcode.vision.SignalDetector;
import org.firstinspires.ftc.teamcode.vision.SignalUtil;

@Autonomous(name = "NeverFail", group = "Autonomous")
@Disabled
public class NeverFail extends LinearOpMode {
	MiniBot robot;

	@Override
	public void runOpMode( ) {

		robot = new MiniBot( this );
		SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
		detector.init( );
		SignalDetector.SignalPosition park = SignalDetector.SignalPosition.NOT_FOUND;
		while( detector.getSignalPosition( ) == SignalDetector.SignalPosition.NOT_FOUND ) {
			park = detector.getSignalPosition( );
		}
		DcMotorEx[] motors = {
				robot.mecanumDrive.frontLeft,
				robot.mecanumDrive.backLeft,
				robot.mecanumDrive.frontRight,
				robot.mecanumDrive.backRight,
		};
		for( DcMotorEx d : motors ) {
			d.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		}
		for( DcMotorEx d : motors ) {
			d.setTargetPosition( robot.mecanumDrive.convertDistTicks( 24 ) );
		}
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
				d.setTargetPosition( robot.mecanumDrive.convertDistTicks( 24 ) );
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


