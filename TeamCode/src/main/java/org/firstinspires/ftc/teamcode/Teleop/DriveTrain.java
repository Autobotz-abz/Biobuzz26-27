package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotFunctions.AutoShooter;
import org.firstinspires.ftc.teamcode.RobotFunctions.TransferPathway;
import org.firstinspires.ftc.teamcode.RobotFunctions.Intake;

@TeleOp
public class DriveTrain extends LinearOpMode {

    public DcMotorEx leftRear, rightRear, leftFront, rightFront;

    @Override
    public void runOpMode() {
        MovementSetUp();
        Intake intake = new Intake(hardwareMap);
        TransferPathway transfer = new TransferPathway(hardwareMap);
        AutoShooter shooter = new AutoShooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            intake.run();
            transfer.run();

            // --- Auto shooter ---
            // Hold right bumper: the wheel spins up and the hood moves, both set from the
            // distance to the hive tags. Dpad up / down = a little faster / slower wheel.
            if (gamepad1.dpadUpWasPressed())   shooter.adjustSlipFactor(0.02);
            if (gamepad1.dpadDownWasPressed()) shooter.adjustSlipFactor(-0.02);
            shooter.update(gamepad1.right_bumper);

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
            telemetry.addData("HuskyLens", shooter.isHuskyConnected() ? "connected" : "NOT FOUND");
            telemetry.addData("Tag IDs seen", shooter.getTagIdsSeen());
            telemetry.addData("Tag width (px)", "%.1f", shooter.getTagWidthPx());
            telemetry.addData("Distance (in)", "%.1f", shooter.getDistanceIn());
            telemetry.addData("Hood angle (deg)", "%.1f", shooter.getTargetAngleDeg());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
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
}