package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RotatingClaw {

	Servo clawServo;
	Servo rotationServo;

	double[] clawPositions;

	public RotatingClaw( HardwareMap hw ) {
		this( hw, "left", "right", new double[]{ 0, 0.5 } );
	}

	public RotatingClaw( HardwareMap hw, String left, String right, double[] clawPositions ) {
		clawServo = hw.servo.get( left );
		rotationServo = hw.servo.get( right );

		this.clawPositions = clawPositions;
	}

	public double[] getPositions( ) {
		return new double[]{ clawServo.getPosition( ), rotationServo.getPosition( ) };
	}

	public void rotate( double pos) {
		rotationServo.setPosition( pos );
	}

	public double getRotatePos() {
		return rotationServo.getPosition();
	}

	public void close( ) {
		clawServo.setPosition( clawPositions[0] );
	}

	public void open( ) {
		clawServo.setPosition( clawPositions[1] );
	}


}
