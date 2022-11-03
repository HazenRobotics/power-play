package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.LifterBot;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.SignalDetector;
import org.firstinspires.ftc.teamcode.vision.SignalUtil;

@Autonomous(name = "NeverFail", group = "Autonomous")
public class NeverFail extends LinearOpMode {
	LifterBot robot;
	@Override
	public void runOpMode( ) {

		robot = new LifterBot( this );
		SignalUtil detector = new SignalUtil( hardwareMap, "webcam1", telemetry );
		detector.init();
		SignalDetector.SignalPosition park = SignalDetector.SignalPosition.NOT_FOUND;
		while(detector.getSignalPosition()== SignalDetector.SignalPosition.NOT_FOUND) {
			park = detector.getSignalPosition();
		}
		DcMotorEx[] motors = {
				robot.mecanumDrive.frontLeft,
				robot.mecanumDrive.backLeft,
				robot.mecanumDrive.frontRight,
				robot.mecanumDrive.backRight,
		};
		for( DcMotorEx d: motors ) {
			d.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		}
		for( DcMotorEx d: motors ) {
			d.setTargetPosition( robot.mecanumDrive.convertDistTicks( 12 ) );
		}
		for( DcMotorEx d: motors ) {
			d.setPower( 0.25 );
		}
		while( motors[0].isBusy() );

		if(park!=SignalDetector.SignalPosition.MIDDLE) {
			if(park== SignalDetector.SignalPosition.LEFT) {

			}
			if(park== SignalDetector.SignalPosition.RIGHT) {

			}
		}
	}

}


