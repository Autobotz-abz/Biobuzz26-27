package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Test for the outtake wheel and the hood servo.
 * The wheel holds a set speed. Circle moves the hood servo up one step, square moves it
 * down one step. The telemetry shows the servo position, so you can write down the
 * numbers for the hood end stops (HOOD_POS_AT_MIN and HOOD_POS_AT_MAX in AutoShooter).
 */
@TeleOp(name = "Hood Test", group = "Test")
public class test extends LinearOpMode {

    // ============ CHANGE THESE ============
    private static final double OUTTAKE_RPM = 2500;      // wheel speed to hold
    private static final double STEP_DEG = 5;            // servo turns this many degrees per press
    private static final double SERVO_RANGE_DEG = 300;   // full travel of your servo for positions 0 to 1
    // (check the data sheet, often 180 or 300)
    private static final double START_POSITION = 0.5;    // where the servo goes when you press start

    private static final double TICKS_PER_REV = 28;      // goBILDA 5203, 6000 rpm

    @Override
    public void runOpMode() {
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        ServoImplEx hood = hardwareMap.get(ServoImplEx.class, "HoodServo");

        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE); // if the wheel spins backwards
        // hood.setDirection(Servo.Direction.REVERSE);            // if circle moves the hood the wrong way

        double stepPosition = STEP_DEG / SERVO_RANGE_DEG;   // one press, in servo position units
        double position = START_POSITION;

        telemetry.addData("Ready", "Press start. The wheel will spin at %.0f rpm.", OUTTAKE_RPM);
        telemetry.update();
        waitForStart();

        hood.setPosition(position);

        while (opModeIsActive()) {
            if (gamepad1.circleWasPressed()) position += stepPosition;   // up
            if (gamepad1.squareWasPressed()) position -= stepPosition;   // down
            position = Math.max(0, Math.min(1, position));
            hood.setPosition(position);
            outtake.setVelocity(OUTTAKE_RPM / 60.0 * TICKS_PER_REV);


            telemetry.addData("Servo position", "%.3f", position);
            telemetry.addData("Servo angle (deg)", "%.1f", position * SERVO_RANGE_DEG);
            telemetry.addData("Target RPM", "%.0f", OUTTAKE_RPM);
            telemetry.addData("Current RPM", "%.0f", outtake.getVelocity() / TICKS_PER_REV * 60.0);
            telemetry.update();
        }
    }
}