package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AidenYAuto extends LinearOpMode {
	public DcMotorEx[] motors = new DcMotorEx[4];
	public String[] names = {"fl", "bl", "fr", "br"};
	public double TICSPERINCH = 42.8;//12.56 inches per rev //537.7 tics per rev
	public double MAGNITUDE = 0.4;
	public int DISTANCE = 30;
	@Override
	public void runOpMode( ) throws InterruptedException {
		for(int i=0;i<motors.length;i++) {
			motors[i] = hardwareMap.get( DcMotorEx.class, names[i] );
//			motors[i].setTargetPosition(4700);
			//motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
//		motor = hardwareMap.get( DcMotorEx.class, "motor" );
//		for(int i=0; i<motors.length;i++){
//		}
//		setPower(1);
		waitForStart();
		motors[0].setDirection( DcMotorSimple.Direction.REVERSE );
		motors[1].setDirection( DcMotorSimple.Direction.REVERSE );

		moveForward( DISTANCE );
		rotate( 120,false );
		moveForward(DISTANCE );
		rotate( 120,false );
		moveForward( DISTANCE );
		moveForward( DISTANCE );
		rotate( 120,true );
		moveForward( DISTANCE );
		rotate( 120,true );
		moveForward( DISTANCE );

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


	public void setPowerRotation(double left, double right,int target){
		int i = 0;
		for (;i<motors.length/2;i++){
			motors[i].setTargetPosition( (int) (target*left/MAGNITUDE) );
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motors[i].setPower(left);
		}
		for (;i<motors.length; i++){
			motors[i].setTargetPosition( (int) (target*right/MAGNITUDE) );
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motors[i].setPower(right);
		}
	}
	public void resetEncoders(){
		for(int i=0; i<motors.length; i++){
			motors[i].setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
			motors[i].setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		}
	}
	public void zeroOutEncoders(){
		for(int i=0; i<motors.length; i++){
			motors[i].setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
			//motors[i].setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		}
	}


	public void rotate(double degrees,boolean clockwise){
		//360 degrees = 4700 tics
		int target = (int) (4700 * (degrees/360));

		if (clockwise){
			setPowerRotation(MAGNITUDE,-MAGNITUDE, target);
		}
		else{
			setPowerRotation( -MAGNITUDE,MAGNITUDE,target );
		}

	}

	public void setTarget(int target){
		for (int i=0; i<motors.length; i++){
			motors[i].setTargetPosition( target );
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
	}

	public void moveForward(double distance /*Inches*/){
		for(int i=0;i<motors.length;i++) {
			setTarget( (int) (distance*TICSPERINCH) );
		}

	}
}
