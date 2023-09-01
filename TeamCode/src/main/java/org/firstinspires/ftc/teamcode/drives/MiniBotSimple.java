package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MiniBotSimple extends MecanumDrive {
	enum Action {
		STRAFE,TURN,DRIVE
	};
	DcMotor[] motors = new DcMotor[4];

	String[] motorNames = {"fl","bl","fr","br"};

	public MiniBotSimple( HardwareMap hardwareMap ) {
		super( hardwareMap );
		for(int i=0; i<motors.length; i++) {
			motors[i] = hardwareMap.get( DcMotor.class,motorNames[i] );
		}
	}
	public void strafe(double ms,double power) {
		setPowers( getRequiredPowers( power,Action.STRAFE ) );
		waitBot( ms );
	}
	public void turn(double ms,double power) {
		setPowers( getRequiredPowers( power,Action.TURN ) );
		waitBot( ms );
	}
	public void move(double ms,double power) {
		setPowers( getRequiredPowers( power,Action.DRIVE ) );
		waitBot( ms );
	}
	public void move(double ms,double[] powers) {
		setPowers( powers );
		waitBot( ms );
	}
	public void move(double ms,double power,Action action) {
		setPowers( getRequiredPowers( power,action ) );
		waitBot( ms );
	}
	public void setPowers(double[] powers) {
		for(int i=0; i<powers.length; i++) {
			motors[i].setPower( powers[i] );
		}
	}
	public double[] getRequiredPowers(double power,Action action) {
		double powerPos=power;
		double powerNeg=-power;
		if(action==Action.DRIVE) {
			return new double[] {powerPos,powerPos,powerPos,powerPos};
		} else if(action==Action.STRAFE) {
			return new double[] {powerPos,powerNeg,powerPos,powerNeg};
		} else if(action==Action.TURN) {
			return new double[] {powerPos,powerPos,powerNeg,powerNeg};
		}
		return null;
	}
	public void waitBot(double ms) {
		long startTime = System.currentTimeMillis();
		while ((startTime + ms) > System.currentTimeMillis()) ;
	}
}
