package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MultiaxialArm {
	final double PULSES_PER_REVOLUTION = 384.5;

	DcMotorEx rotationMotor;
	DcMotorEx lowerMotor;
	DcMotorEx upperMotor;

	public MultiaxialArm ( HardwareMap hm ) {
		setup(hm, "rotationMotor", "lowerMotor", "upperMotor");
	}

	public MultiaxialArm ( HardwareMap hm, String rotationMotorName, String lowerMotorName, String upperMotorName ) {
		setup( hm, rotationMotorName, lowerMotorName, upperMotorName );
	}

	public void setup ( HardwareMap hm, String rotationMotorName, String lowerMotorName, String upperMotorName ) {
		rotationMotor = hm.get( DcMotorEx.class, rotationMotorName );
		lowerMotor = hm.get( DcMotorEx.class, lowerMotorName );
		upperMotor = hm.get( DcMotorEx.class, upperMotorName );

		rotationMotor.setZeroPowerBehavior( DcMotorEx.ZeroPowerBehavior.BRAKE );
		lowerMotor.setZeroPowerBehavior( DcMotorEx.ZeroPowerBehavior.BRAKE );
		upperMotor.setZeroPowerBehavior( DcMotorEx.ZeroPowerBehavior.BRAKE );
	}

	public void controlMotors (double rotatePower, double lowerPower, double upperPower) {
		rotationMotor.setPower( rotatePower );
		lowerMotor.setPower( lowerPower );
		upperMotor.setPower( upperPower );
	}

	public void printTelemetry( Telemetry t ) {
		t.addData( "Rotate Power:", rotationMotor.getPower());
		t.addData( "Lower Power:", lowerMotor.getPower());
		t.addData( "Upper Power:", upperMotor.getPower());
	}
}
