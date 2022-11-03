package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Claw {

	Servo leftServo;
	Servo rightServo;

	double[] leftPos;
	double[] rightPos;

	public Claw( HardwareMap hw ) {
		this( hw, "left", "right", new double[]{ 0, 1 }, new double[]{ 0, 1 } );
	}

	public Claw( HardwareMap hw, String left, String right, double[] leftPos, double[] rightPos ) {
		leftServo = hw.servo.get( left );
		rightServo = hw.servo.get( right );

		this.leftPos = leftPos;
		this.rightPos = rightPos;
	}

	public void setPositions( double leftPos, double rightPos ) {
		leftServo.setPosition( leftPos );
		rightServo.setPosition( rightPos );
	}

	public double[] getPositions( ) {
		return new double[]{ leftServo.getPosition( ), rightServo.getPosition( ) };
	}

	public void close( ) {
		setPositions( leftPos[1], rightPos[1] );
	}

	public void open( ) {
		setPositions( leftPos[0], rightPos[0] );
	}


}
