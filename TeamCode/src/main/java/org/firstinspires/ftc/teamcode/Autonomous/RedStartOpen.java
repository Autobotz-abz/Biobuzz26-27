package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.AutoShooter;
import org.firstinspires.ftc.teamcode.RobotFunctions.Intake;
import org.firstinspires.ftc.teamcode.RobotFunctions.TransferPathway;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive to the shooting spot, shoot the preload, then leave.
 * Uses the same parts as the DriveTrain TeleOp: Intake, TransferPathway, AutoShooter
 * (wheel power and hood angle from the HuskyLens distance) and the two gate servos.
 */
@Autonomous
public class RedStartOpen extends OpMode {

    /* ==================== SPEED ==================== */
    private static final double FAST_SPEED = 1.0;

    /* ==================== SHOOTING TIMES ==================== */
    // Longest wait for the shooter to be ready (tag seen, wheel spun up) before
    // opening the gates anyway, so the auto never gets stuck.
    private static final double MAX_READY_WAIT_S = 2.0;
    // How long the gates stay open to let all the balls through.
    private static final double GATE_OPEN_TIME_S = 2.5;

    /* ==================== POSES ==================== */
    private final Pose start = new Pose(56, 8, Math.toRadians(90));
    private final Pose shoot0 = new Pose(56, 25, Math.toRadians(90));
    private final Pose parkHold = new Pose(22.03, 56);                     // curve control point
    private final Pose leave = new Pose(13, 108, Math.toRadians(270));

    private PathChain toShoot0, toLeave;

    /* ==================== PARTS ==================== */
    private Follower follower;
    private Intake intake;
    private TransferPathway transfer;
    private AutoShooter shooter;

    /* ==================== STATES ==================== */
    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        LEAVE,
        IDLE
    }

    private enum ShotStep { WAIT_FOR_PATH, WAIT_FOR_READY, GATES_OPEN }

    private PathState pathState;
    private ShotStep shotStep = ShotStep.WAIT_FOR_PATH;
    private final Timer shotTimer = new Timer();

    /* ==================== PATHS ==================== */

    public void buildPaths() {
        toShoot0 = follower.pathBuilder()
                .addPath(new BezierLine(start, shoot0))
                .setLinearHeadingInterpolation(start.getHeading(), shoot0.getHeading())
                .build();

        toLeave = follower.pathBuilder()
                .addPath(new BezierCurve(shoot0, parkHold, leave))
                .setTangentHeadingInterpolation()
                .build();
    }

    /* ==================== STATE MACHINE ==================== */

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_TO_SHOOT:
                follower.followPath(toShoot0, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (shoot()) setPathState(PathState.LEAVE);
                break;

            case LEAVE:
                if (!follower.isBusy()) {
                    follower.followPath(toLeave, true);
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;
        }
    }

    /**
     * Call every loop while shooting. Waits for the path to finish, waits for the
     * shooter to be ready (or MAX_READY_WAIT_S), opens the gates for GATE_OPEN_TIME_S,
     * then closes them. Returns true when the shot is done.
     */
    private boolean shoot() {
        switch (shotStep) {
            case WAIT_FOR_PATH:
                if (!follower.isBusy()) {
                    shotStep = ShotStep.WAIT_FOR_READY;
                    shotTimer.resetTimer();
                }
                return false;

            case WAIT_FOR_READY:
                if (shooter.isReady() || shotTimer.getElapsedTimeSeconds() > MAX_READY_WAIT_S) {
                    transfer.openGates();
                    shotStep = ShotStep.GATES_OPEN;
                    shotTimer.resetTimer();
                }
                return false;

            case GATES_OPEN:
                if (shotTimer.getElapsedTimeSeconds() > GATE_OPEN_TIME_S) {
                    transfer.closeGates();
                    return true;
                }
                return false;
        }
        return false;
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        shotStep = ShotStep.WAIT_FOR_PATH;
    }

    /* ==================== INIT ==================== */

    @Override
    public void init() {
        pathState = PathState.DRIVE_TO_SHOOT;

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(FAST_SPEED);

        intake = new Intake(hardwareMap);
        transfer = new TransferPathway(hardwareMap);   // also closes the gates
        shooter = new AutoShooter(hardwareMap);

        buildPaths();
        follower.setPose(start);
    }

    /** Shows what the HuskyLens sees before start, so you can check it is aimed. */
    @Override
    public void init_loop() {
        shooter.update(false);
        telemetry.addData("HuskyLens", shooter.isHuskyConnected() ? "connected" : "NOT FOUND");
        telemetry.addData("Tag IDs seen", shooter.getTagIdsSeen());
        telemetry.addData("Distance (in)", "%.1f", shooter.getDistanceIn());
        telemetry.update();
    }

    /* ==================== LOOP ==================== */

    @Override
    public void loop() {
        intake.run();
        transfer.run();
        // Wheel power and hood angle follow the distance to the tags, same as TeleOp.
        shooter.update(true);

        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Shot step", shotStep);
        telemetry.addData("Distance (in)", "%.1f", shooter.getDistanceIn());
        telemetry.addData("Hood angle (deg)", "%.1f", shooter.getTargetAngleDeg());
        telemetry.addData("Outtake power", "%.2f", shooter.getOuttakePower());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.update(false);
        intake.stop();
        transfer.stop();
    }
}
