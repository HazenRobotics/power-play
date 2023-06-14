package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ConnersEyes {
	CRServo leftEye;
	CRServo rightEye;
	public ConnersEyes( HardwareMap map){
		leftEye = map.get(CRServo.class,"le");
		rightEye = map.get(CRServo.class,"re");
	}

	public void rotate(double lTrigger, double rTrigger){
		leftEye.setPower(lTrigger - rTrigger);
		rightEye.setPower(lTrigger - rTrigger);
	}
}
