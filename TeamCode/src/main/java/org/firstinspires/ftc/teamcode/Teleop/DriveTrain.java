package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.AutoShooter;
import org.firstinspires.ftc.teamcode.RobotFunctions.TransferPathway;
import org.firstinspires.ftc.teamcode.RobotFunctions.Intake;

/**
 * Controls (gamepad 1):
 *   Sticks  = drive
 *   Square  = hold to open gate 1, let go to close it
 *   Circle  = hold to open gate 2, let go to close it
 * The shooter runs by itself: the wheel power and the hood angle follow the distance
 * to the hive tags.
 */
@TeleOp
public class DriveTrain extends LinearOpMode {

    // ============ GATES: CHANGE THESE ============
    private static final String GATE1_NAME = "Gate1Servo";   // names in your configuration
    private static final String GATE2_NAME = "Gate2Servo";
    private static final double GATE_OPEN_POS = 0.5;         // <-- SET
    private static final double GATE_CLOSED_POS = 0.0;       // <-- SET

    public DcMotorEx leftRear, rightRear, leftFront, rightFront;
    public Servo gate1, gate2;

    @Override
    public void runOpMode() {
        MovementSetUp();
        GateSetUp();
        Intake intake = new Intake(hardwareMap);
        TransferPathway transfer = new TransferPathway(hardwareMap);
        AutoShooter shooter = new AutoShooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            intake.run();
            transfer.run();

            // --- Gates ---
            // Each gate is open only while its button is held (square = gate 1, circle = gate 2).
            gate();

            // --- Auto shooter ---
            // Always on: wheel power and hood angle are set from the distance to the tags.
            shooter.update(true);

            // --- Driving ---
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(rx), 1);
            leftFront.setPower((y + rx + x) / denominator);
            leftRear.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - rx - x) / denominator);
            rightRear.setPower((y + x - rx) / denominator);

            // --- Telemetry ---
            telemetry.addData("Gate 1", gamepad1.square ? "OPEN" : "closed");
            telemetry.addData("Gate 2", gamepad1.circle ? "OPEN" : "closed");
            telemetry.addData("HuskyLens", shooter.isHuskyConnected() ? "connected" : "NOT FOUND");
            telemetry.addData("Tag IDs seen", shooter.getTagIdsSeen());
            telemetry.addData("Tag width (px)", "%.1f", shooter.getTagWidthPx());
            telemetry.addData("Distance (in)", "%.1f", shooter.getDistanceIn());
            telemetry.addData("Hood angle (deg)", "%.1f", shooter.getTargetAngleDeg());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Outtake power", "%.2f", shooter.getOuttakePower());
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRpm());
            telemetry.addData("Slip factor", "%.2f", shooter.getSlipFactor());
            telemetry.addData("Can shoot", shooter.canShoot());
            telemetry.addData("Ready", shooter.isReady());
            telemetry.update();
        }
    }

    private void MovementSetUp() {
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void GateSetUp() {
        gate1 = hardwareMap.get(Servo.class, GATE1_NAME);
        gate2 = hardwareMap.get(Servo.class, GATE2_NAME);

        gate1.setDirection(Servo.Direction.FORWARD);
        gate2.setDirection(Servo.Direction.FORWARD);   // REVERSE if gate 2 is mirrored

        // Start closed so the preloaded balls stay in.
        gate1.setPosition(GATE_CLOSED_POS);
        gate2.setPosition(GATE_CLOSED_POS);
    }

    private void gate() {
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
}