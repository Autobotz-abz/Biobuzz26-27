package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Test for the outtake wheel and the hood servo.
 * The wheel runs at a fixed power. Circle moves the hood servo up one step, square moves it
 * down one step. The telemetry shows the servo position, so you can write down the
 * numbers for the hood end stops (HOOD_POS_AT_MIN and HOOD_POS_AT_MAX in AutoShooter).
 */
@TeleOp(name = "Hood Test", group = "Test")
public class test extends LinearOpMode {

    // ============ CHANGE THESE ============// wheel power, 0 to 1
    private static final String GATE1_NAME = "Gate1Servo";   // names in your configuration
    private static final String GATE2_NAME = "Gate2Servo";
    private static final double GATE_OPEN_POS = 1.0;
    private static final double GATE_CLOSED_POS = 0.5;       // <-- SET

    public DcMotorEx leftRear, rightRear, leftFront, rightFront;
    public Servo gate1, gate2;

    @Override
    public void runOpMode() {
        gate1 = hardwareMap.get(Servo.class, "Gate1Servo");
        gate2 = hardwareMap.get(Servo.class, "Gate2Servo");
        gate2.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            gate();
        }


    }
    private void gate() {
        if (gamepad1.square) {
            gate1.setPosition(0.0);
        } else {
            gate1.setPosition(0.5);
        }

        if (gamepad1.circle) {
            gate2.setPosition(GATE_OPEN_POS);
        } else {
            gate2.setPosition(GATE_CLOSED_POS);
        }
    }
}