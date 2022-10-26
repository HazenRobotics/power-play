package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

	Servo leftServo;
	Servo rightServo;

	public Claw( HardwareMap hw ) {
		this( hw, "left", "right" );
	}

	public Claw( HardwareMap hw, String left, String right ) {
		leftServo = hw.servo.get( left );
		rightServo = hw.servo.get( right );
	}

	public void setPositions( double leftPos, double rightPos ) {
		leftServo.setPosition( leftPos );
		rightServo.setPosition( rightPos );
	}

	public void close( ) {
		setPositions( 0.85,0.25 );
	}

	public void open( ) {
		setPositions( 0.4, 0.7 );
	}

}
