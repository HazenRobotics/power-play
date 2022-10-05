package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "ReallyWackyDrive", group = "Test")
//@Disabled
public class ReallyWackyDrive extends OpMode {

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    boolean leftOnly = false;
    boolean rightOnly = false;
    boolean lockSingle = false;

    @Override
    public void init() {

        telemetry.addData("Mode", "Initiating robot...");
        telemetry.update();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        telemetry.addData("Mode", "waiting for start??");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            lockSingle= !lockSingle;
        }
        leftOnly = gamepad1.right_stick_y==0 && gamepad1.right_stick_x==0 || lockSingle;
        rightOnly = gamepad1.left_stick_y==0 && gamepad1.left_stick_x==0  && lockSingle;
        if(gamepad1.left_stick_x==0 && gamepad1.right_stick_x==0 ) {
            setPowers(gamepad1.left_stick_y,gamepad1.right_stick_x,rightOnly,leftOnly);
        } else {
            float drive = gamepad1.left_stick_y+gamepad1.right_stick_y;
            float strafe = gamepad1.left_stick_x+gamepad1.right_stick_x;
            move(drive,strafe);
        }


        telemetry.addData("l: ", gamepad1.left_stick_y);
        telemetry.addData("r: ", gamepad1.right_stick_y);

        telemetry.update();
    }
    public void setPowers(float leftPower,float rightPower,boolean left,boolean right) {
        if(left && right) {
            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);
        } else if (left) {
            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(leftPower);
            backRightMotor.setPower(leftPower);
        } else if(right) {
            frontLeftMotor.setPower(rightPower);
            backLeftMotor.setPower(rightPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);
        }
    }

    public void move( float drive, float strafe ) {
        double frontLeftPower = drive + strafe;
        double backLeftPower = drive - strafe;
        double frontRightPower = drive - strafe;
        double backRightPower = drive + strafe;
        frontLeftMotor.setPower( frontLeftPower );
        backLeftMotor.setPower( backLeftPower );
        frontRightMotor.setPower( frontRightPower );
        backRightMotor.setPower( backRightPower );
    }
    public void displayTelemetry() {
        telemetry.addLine("Docs:");
        telemetry.addLine("Each stick controls a side of the robot");
        telemetry.addLine("However moving only a single stick moves everything");
        telemetry.addLine("You can lock single stick mode by pressing A");
        telemetry.addLine("");
        telemetry.addData("L Drive",leftOnly);
        telemetry.addData("R Drive",rightOnly);
        telemetry.addData("Single Lock ",lockSingle);
        telemetry.addLine("");
        telemetry.addData( "ly: ", -gamepad1.left_stick_y );
        telemetry.addData( "lx: ", gamepad1.left_stick_x );
        telemetry.addData( "rx: ", gamepad1.right_stick_x );
        telemetry.addLine("");
        telemetry.addData("flp: ", frontLeftMotor.getPower());
        telemetry.addData("blp: ", backLeftMotor.getPower());
        telemetry.addData("frp: ", frontRightMotor.getPower());
        telemetry.addData("brp: ", backRightMotor.getPower());

        telemetry.update( );
    }

}