package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Auto-aim shooter. The HuskyLens reads the hive AprilTags, the tag size gives the
 * distance, and the distance plus the hive height give the hood angle and the wheel speed.
 * No lookup table and no calibration OpMode.
 *
 * Use:  AutoShooter shooter = new AutoShooter(hardwareMap);
 *       shooter.update(gamepad1.right_bumper);   // every loop
 */
public class AutoShooter {

    // ============ 1. FIELD (BioBuzz game manual, Field Acceptance Checklist) ============
    // Aim point = middle of the upright CELL opening. Its bottom edge is 53.5 in above the
    // tiles and it is 14 in tall along a 30 degree tilt, so the middle is about 59.5 in up.
    private static final double OPENING_CENTER_HEIGHT_IN = 59.5;
    private static final double TAG_SIZE_IN = 3.25;
    // Flat distance from the tags to the middle of the opening. 0 is fine to start
    // because the opening is 20 in wide.
    private static final double TAG_TO_TARGET_IN = 0;

    // ============ 2. YOUR ROBOT: SET THESE ONCE ============
    // Names in your robot configuration.
    private static final String HUSKY_NAME = "HuskyLens";
    private static final String OUTTAKE_NAME = "Outtake";
    private static final String HOOD_NAME = "HoodServo";

    // Height of the ball when it leaves the shooter, measured from the tiles.
    // 16.3 is the shooterHeight from your old code.
    private static final double BALL_EXIT_HEIGHT_IN = 16.3;

    // Hood end stops: the launch angle at each end, and the servo position at each end.
    // These four are placeholders. Read the angles from your CAD (or an angle finder).
    private static final double HOOD_MIN_ANGLE_DEG = 45;   // <-- SET
    private static final double HOOD_MAX_ANGLE_DEG = 80;   // <-- SET
    private static final double HOOD_POS_AT_MIN = 0.20;    // <-- SET
    private static final double HOOD_POS_AT_MAX = 0.80;    // <-- SET

    // Tag IDs to aim at. Empty = aim at any tag. The telemetry shows the IDs your
    // HuskyLens really reports. Put your alliance's IDs here to ignore the other hive.
    private static final int[] TARGET_TAG_IDS = {};

    // ============ 3. CAMERA (from your STEP file and your old code) ============
    // Focal length in pixels (image is 320 x 240). Your old code had (70 * 28) / 7 = 280.
    // 290 sits between that and 301, which is what a 6.5 in tag would give.
    private static final double FOCAL_LENGTH_PX = 290;
    // Lens angle above horizontal. Your STEP file gives 55 (model Y axis = up).
    private static final double CAMERA_PITCH_DEG = 55;
    // The camera sits this far behind the point where the ball leaves (STEP file).
    private static final double CAMERA_BEHIND_EXIT_IN = 3.5;
    private static final double IMAGE_CENTER_Y = 120;

    // ============ 4. FLYWHEEL ============
    private static final double WHEEL_RADIUS_IN = 36.0 / 25.4; // 72 mm wheel
    private static final double TICKS_PER_REV = 28;            // goBILDA 5203, 6000 rpm
    private static final double MAX_MOTOR_RPM = 6000;
    private static final double MOTOR_TO_WHEEL = 1.0;          // 1:1 belt in your STEP file

    // ============ 5. TUNING (the defaults should work) ============
    // Angle the ball comes down at. The CELL mouth faces about 30 degrees above
    // horizontal, so 30 drops the ball straight in.
    private static final double DESCENT_ANGLE_DEG = 30;
    // Shots that would arrive flatter than this are refused.
    private static final double MIN_DESCENT_DEG = 20;
    // Wheel surface speed divided by ball speed. A ball squeezed between a wheel and a
    // fixed hood leaves at about half the wheel speed, so this is close to 2.
    // Nudge it live with adjustSlipFactor() if shots land short or long everywhere.
    private double slipFactor = 1.9;

    // ============ OTHER ============
    private static final double TARGET_HEIGHT_IN = OPENING_CENTER_HEIGHT_IN - BALL_EXIT_HEIGHT_IN;
    private static final double G = 386.09;                    // in/s^2
    private static final double SMOOTHING = 0.2;               // 0 to 1, higher = faster
    private static final double READY_TOLERANCE_RPM = 50;
    private static final long TAG_TIMEOUT_MS = 500;

    private final HuskyLens husky;
    private final DcMotorEx outtake;
    private final ServoImplEx hood;
    private final boolean huskyConnected;

    private boolean hasDistance = false;
    private double distanceIn = 0;
    private double tagWidthPx = 0;
    private String idsSeen = "none";
    private double targetAngleDeg = (HOOD_MIN_ANGLE_DEG + HOOD_MAX_ANGLE_DEG) / 2;
    private double targetRpm = 0;
    private boolean canShoot = false;
    private long lastSeenMs = 0;

    public AutoShooter(HardwareMap hw) {
        husky = hw.get(HuskyLens.class, HUSKY_NAME);
        outtake = hw.get(DcMotorEx.class, OUTTAKE_NAME);
        hood = hw.get(ServoImplEx.class, HOOD_NAME);

        huskyConnected = husky.knock();
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // outtake.setDirection(DcMotorSimple.Direction.REVERSE); // if the wheel spins backwards

        hood.setPosition(angleToServo(targetAngleDeg));
    }

    /** Call every loop. spinUp = true runs the wheel at the calculated speed. */
    public void update(boolean spinUp) {
        // 1. Read the tags.
        Block[] blocks = null;
        try {
            blocks = husky.blocks();
        } catch (Exception ignored) { }

        double maxWidth = 0;
        StringBuilder ids = new StringBuilder();
        if (blocks != null) {
            for (Block b : blocks) {
                if (ids.length() > 0) ids.append(", ");
                ids.append(b.id);
                if (isTarget(b.id)) maxWidth = Math.max(maxWidth, b.width);
            }
        }
        idsSeen = ids.length() > 0 ? ids.toString() : "none";

        // 2. Distance. A cluster has up to 4 tags, so average the biggest ones. Tags much
        //    smaller than the biggest belong to something farther away and are skipped.
        double distSum = 0;
        double widthSum = 0;
        int count = 0;
        if (blocks != null && maxWidth > 0) {
            for (Block b : blocks) {
                if (isTarget(b.id) && b.width >= 0.75 * maxWidth) {
                    distSum += distanceFromTag(b.width, b.y);
                    widthSum += b.width;
                    count++;
                }
            }
        }
        if (count > 0) {
            double raw = Math.max(distSum / count, 1);
            distanceIn = hasDistance ? distanceIn + SMOOTHING * (raw - distanceIn) : raw;
            tagWidthPx = widthSum / count;
            hasDistance = true;
            lastSeenMs = System.currentTimeMillis();
        }

        // 3. Distance and height -> hood angle and wheel speed.
        //    If the tag is lost, the last distance is reused, so the shot is held.
        if (hasDistance) {
            double[] shot = solve(distanceIn, TARGET_HEIGHT_IN);
            if (shot != null) {
                double wheelRpm = shot[1] / (2 * Math.PI * WHEEL_RADIUS_IN) * 60.0 * slipFactor;
                double motorRpm = wheelRpm / MOTOR_TO_WHEEL;
                targetAngleDeg = shot[0];
                targetRpm = Math.min(motorRpm, MAX_MOTOR_RPM);
                canShoot = motorRpm <= MAX_MOTOR_RPM;
            } else {
                canShoot = false;
            }
        }

        // 4. Move the hood and the wheel.
        hood.setPosition(angleToServo(targetAngleDeg));
        if (spinUp) {
            outtake.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
        } else {
            outtake.setPower(0);
        }
    }

    // ================= DISTANCE FROM ONE TAG =================

    /**
     * Flat distance (in) from the point where the ball leaves to the middle of the opening,
     * from one tag that is widthPx wide and centered at row centerYPx of the image.
     *
     * The tag width gives the straight line distance (pinhole camera). The camera looks up
     * at CAMERA_PITCH_DEG, and a tag higher in the image is seen at a steeper angle, so the
     * flat distance is the straight line distance times the cosine of that angle.
     */
    public static double distanceFromTag(double widthPx, double centerYPx) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        // Angle of the tag above the middle of the image. Higher in the image = bigger.
        double beta = Math.atan((IMAGE_CENTER_Y - centerYPx) / FOCAL_LENGTH_PX);
        // Straight line distance from the camera to the tag.
        double slant = FOCAL_LENGTH_PX * TAG_SIZE_IN / (Math.max(widthPx, 1) * Math.cos(beta));
        // Flat distance along the floor, then move the start point from the camera to the
        // ball exit and the end point from the tag to the middle of the opening.
        double flat = slant * Math.cos(pitch + beta);
        return flat - CAMERA_BEHIND_EXIT_IN + TAG_TO_TARGET_IN;
    }

    // ================= THE SHOOTING MATH =================

    /**
     * Returns {hoodAngleDeg, ballSpeedInPerSec} for a target d inches away and h inches
     * above the ball exit, or null if the hood cannot make the shot (too close, too high).
     *
     * Many angle and speed pairs reach the same spot, so the angle is picked so the
     * ball comes down into the hive at DESCENT_ANGLE_DEG:
     *     tan(launch angle) = 2h/d + tan(descent angle)
     * Then the speed comes from the normal projectile equation.
     */
    public static double[] solve(double d, double h) {
        d = Math.max(d, 1);
        double tanDescent = Math.tan(Math.toRadians(DESCENT_ANGLE_DEG));
        double angle = Math.toDegrees(Math.atan(2 * h / d + tanDescent));
        angle = clamp(angle, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);

        double v = ballSpeed(d, h, angle);
        // Hood too flat to reach the height: tilt it up until it can.
        while (Double.isNaN(v) && angle < HOOD_MAX_ANGLE_DEG) {
            angle = Math.min(angle + 0.5, HOOD_MAX_ANGLE_DEG);
            v = ballSpeed(d, h, angle);
        }
        if (Double.isNaN(v)) return null;

        // Angle the ball arrives at: tan(descent) = tan(launch) - 2h/d
        double descent = Math.toDegrees(Math.atan(Math.tan(Math.toRadians(angle)) - 2 * h / d));
        if (descent < MIN_DESCENT_DEG) return null;

        return new double[]{angle, v};
    }

    /** Ball speed (in/s) that passes through (d, h) at this launch angle. NaN if impossible. */
    private static double ballSpeed(double d, double h, double angleDeg) {
        double a = Math.toRadians(angleDeg);
        double c = Math.cos(a);
        double denom = 2 * c * c * (d * Math.tan(a) - h);
        if (denom <= 0) return Double.NaN;
        return Math.sqrt(G * d * d / denom);
    }

    private static double angleToServo(double angleDeg) {
        double t = (angleDeg - HOOD_MIN_ANGLE_DEG) / (HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG);
        t = clamp(t, 0, 1);
        return HOOD_POS_AT_MIN + t * (HOOD_POS_AT_MAX - HOOD_POS_AT_MIN);
    }

    // ================= STATUS =================

    public boolean tagVisible() {
        return System.currentTimeMillis() - lastSeenMs < TAG_TIMEOUT_MS;
    }

    public double getCurrentRpm() {
        return outtake.getVelocity() / TICKS_PER_REV * 60.0;
    }

    /** True when the shot is possible, the tag is seen, and the wheel is at speed. */
    public boolean isReady() {
        return canShoot && tagVisible()
                && Math.abs(getCurrentRpm() - targetRpm) <= READY_TOLERANCE_RPM;
    }

    public boolean isHuskyConnected() { return huskyConnected; }
    public boolean canShoot()         { return canShoot; }
    public double getDistanceIn()     { return distanceIn; }
    public double getTagWidthPx()     { return tagWidthPx; }
    public String getTagIdsSeen()     { return idsSeen; }
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public double getTargetRpm()      { return targetRpm; }

    public double getSlipFactor()     { return slipFactor; }
    public void adjustSlipFactor(double delta) {
        slipFactor = Math.max(0.5, slipFactor + delta);
    }

    // ================= HELPERS =================

    private static boolean isTarget(int id) {
        if (TARGET_TAG_IDS.length == 0) return true;
        for (int t : TARGET_TAG_IDS) {
            if (t == id) return true;
        }
        return false;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}