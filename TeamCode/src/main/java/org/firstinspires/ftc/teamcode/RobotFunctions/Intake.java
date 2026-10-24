package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final DcMotorEx motor;
    public Servo servo;
    public Servo servo1;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "Intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Uncomment if the intake spins the wrong way:
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void run() {
        motor.setPower(1);
        servo.setDirection(Servo.Direction.FORWARD);
        servo1.setDirection(Servo.Direction.FORWARD);

    }
    public void reverse() {

        motor.setPower(-1);
    }
    public void stop() {

        motor.setPower(0);
    }
    public void setPower(double power) {

        motor.setPower(power);
    }
}