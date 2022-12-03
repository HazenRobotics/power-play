package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class ServoTurret {

	CRServo servo;



	public enum MovementState {
		REST, MOVING;
	}

	MovementState movementState = MovementState.REST;

	public ServoTurret( HardwareMap hw ) {
		this( hw, "turret", false );
	}

	public ServoTurret( HardwareMap hw, String servoName, boolean reverseServo ) {

		servo = hw.crservo.get( servoName );
		servo.setDirection( reverseServo ? Direction.REVERSE : Direction.FORWARD );

//		resetTurret( );
	}

	public void setPower( double power ) {
		servo.setPower( power );
	}
}
