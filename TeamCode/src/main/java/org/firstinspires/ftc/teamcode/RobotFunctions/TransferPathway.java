
package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Transfer motor plus the two gate servos that hold the balls back from the shooter.
 * The gates close as soon as this is created (in init), so preloaded balls stay put.
 */
public class TransferPathway {

    // ============ GATES: CHANGE THESE ============
    private static final String GATE1_NAME = "Gate1Servo";   // names in your configuration
    private static final String GATE2_NAME = "Gate2Servo";
    private static final double GATE_OPEN_POS = 0.5;         // <-- SET
    private static final double GATE_CLOSED_POS = 0.0;       // <-- SET
    // Set true if the second gate servo is mirrored and has to turn the other way.
    private static final boolean GATE2_REVERSED = false;

    private final DcMotorEx motor;
    private final Servo gate1;
    private final Servo gate2;

    public TransferPathway(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "Transfer");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate1 = hardwareMap.get(Servo.class, GATE1_NAME);
        gate2 = hardwareMap.get(Servo.class, GATE2_NAME);
        closeGates();
    }

    // ================= MOTOR =================

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

    // ================= GATES =================

    /** TeleOp: hold square = gate 1 open, hold circle = gate 2 open, let go = closed. */
    public void gate(Gamepad gamepad1) {
        if (gamepad1.square) {
            gate1.setPosition(GATE_OPEN_POS);
        } else {
            gate1.setPosition(GATE_CLOSED_POS);
        }

        if (gamepad1.circle) {
            gate2.setPosition(GATE_OPEN_POS);
        } else {
            gate2.setPosition(GATE_CLOSED_POS);
        }
    }

    // Used by the autonomous (no gamepad) and at init.
    public void openGates() {
        gate1.setPosition(GATE_OPEN_POS);
        gate2.setPosition(GATE_OPEN_POS);
    }

    public void closeGates() {
        gate1.setPosition(GATE_CLOSED_POS);
        gate2.setPosition(GATE_CLOSED_POS);
    }
}
