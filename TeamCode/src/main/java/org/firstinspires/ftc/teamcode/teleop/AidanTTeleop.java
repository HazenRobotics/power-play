package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.ConnersArm;
import org.firstinspires.ftc.teamcode.subsystems.ConnersEyes;
import org.firstinspires.ftc.teamcode.subsystems.MegansClaw;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "AidanTTeleop", group = "TeleOp")
public class AidanTTeleop extends OpMode {
	GamepadEvents controller;
	DcMotor[] motors=new DcMotor[4];
	String[] motorNames = {"fl","bl","fr","br"};
	//SubSystem for arm
	ConnersArm arm;
	ConnersEyes eyes;

	MegansClaw claw;
	@Override
	public void init( ) {
		for(int i=0;i<motors.length;i++){
			motors[i]= hardwareMap.get(DcMotor.class, motorNames[i]);
		}
		motors[2].setDirection( DcMotorSimple.Direction.REVERSE );
		motors[3].setDirection( DcMotorSimple.Direction.REVERSE );
		controller = new GamepadEvents( gamepad1 );
		arm = new ConnersArm(hardwareMap);
		eyes = new ConnersEyes(hardwareMap);
		claw = new MegansClaw( hardwareMap );
	}

	@Override
	public void loop( ) {
		double drive = controller.left_stick_y;
		double strafe = -controller.left_stick_x;
		double rotate = -controller.right_stick_x;

		motors[0].setPower(drive + strafe + rotate);
		motors[1].setPower(drive - strafe + rotate);
		motors[2].setPower(drive - strafe - rotate);
		motors[3].setPower(drive + strafe - rotate);

		eyes.rotate(controller.left_trigger.getTriggerValue(),controller.right_trigger.getTriggerValue());
		if (controller.a.onPress()){
			arm.move();
		}
		if (controller.b.onPress()){
			claw.motion();
		}

		controller.update();
	}
}


//classname

	//subsystem 1
		//subsystem methods
	//subsystem 2
		//subsystem methods
	//...

	//use of subsystems in conjunction (theoretical usage, i.e. linkage & claw together)

