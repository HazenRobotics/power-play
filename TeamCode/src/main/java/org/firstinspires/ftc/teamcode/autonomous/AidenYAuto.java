package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous
public class AidenYAuto extends LinearOpMode {
	public DcMotorEx[] motors = new DcMotorEx[4];
	public String[] names = {"fl", "bl", "fr", "br"};
	public double ticsPerInch;
	@Override
	public void runOpMode( ) throws InterruptedException {
		for(int i=0;i<motors.length;i++) {
			motors[i] = hardwareMap.get( DcMotorEx.class, names[i] );
			motors[i].setTargetPosition(4700);
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
		//motor = hardwareMap.get( DcMotorEx.class, "motor" );
		waitForStart();
//		for(int i=0; i<motors.length;i++){
//
//		}
		setPower(1);





//		motor.setTargetPosition( 5000 );
//		motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
//		motor.setPower( 1 );
		while(motors[0].isBusy() && opModeIsActive());
		resetEncoders();
	}

	public void setPower(double power){
		for(int i=0; i<motors.length; i++){
			motors[i].setPower( power );
		}
	}
	public void resetEncoders(){
		for(int i=0; i<motors.length; i++){
			motors[i].setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
			motors[i].setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		}
	}

}
