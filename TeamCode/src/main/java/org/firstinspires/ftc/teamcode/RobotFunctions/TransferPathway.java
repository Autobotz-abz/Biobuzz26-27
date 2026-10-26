
package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferPathway {

    private final DcMotorEx motor;

    public TransferPathway(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "Transfer");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        motor.setPower(1);
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

