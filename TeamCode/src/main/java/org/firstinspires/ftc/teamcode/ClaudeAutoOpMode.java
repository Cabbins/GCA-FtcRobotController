package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * ============================================================
 *  FTC Autonomous — Limelight Target, Shoot, Collect, Repeat
 * ============================================================
 *  Alignment is two phases:
 *   Phase 1: Drive fwd/back to reach TY_TARGET, turning to
 *            keep the april tag in the camera view
 *   Phase 2: Strafe/turn to reach TX_TARGET
 *
 *  Every 2 cycles: turn ~90°, drive forward with intake + diverter,
 *  drive forward more, then reverse back and re-align.
 * ============================================================
 */
@Autonomous
public class ClaudeAutoOpMode extends LinearOpMode {

    // ─── Drive Motors ───────────────────────────────────────────────────────────
    private DcMotor FRmotor, FLmotor, BRmotor, BLmotor;

    // ─── Shooter Motors ─────────────────────────────────────────────────────────
    private DcMotor RShootMotor, LShootMotor;

    // ─── Intake ─────────────────────────────────────────────────────────────────
    private DcMotor IntakeMotor;

    // ─── Servos ─────────────────────────────────────────────────────────────────
    private CRServo Servo_Fix_L;
    private CRServo Servo_Fix_R;
    private Servo Servo_sorting;

    // ─── Limelight ──────────────────────────────────────────────────────────────
    private Limelight3A limelight;
    private static final int LIMELIGHT_PIPELINE = 8;

    // ─── Tunable Constants ──────────────────────────────────────────────────────

    // Shooter
    private static final double SHOOT_POWER        = 0.75;
    private static final double SHOOT_SPIN_UP_SEC  = 0.8;
    private static final double FEED_POWER         = -1.0;
    private static final double FEED_DWELL_SEC     = 0.7;
    private static final double SORT_LEFT_POS      = 0.25;
    private static final double SORT_RIGHT_POS     = 0.75;
    private static final double BETWEEN_SHOT_SEC   = 0.15;

    // Intake
    private static final double INTAKE_POWER            = -1.0;
    private static final double INTAKE_DURATION_SEC     = 2.5;
    private static final double INTAKE_RIGHT_DWELL_SEC  = 1.0;

    // Limelight alignment targets
    private static final double TX_TARGET          = -6.1;
    private static final double TX_DEADBAND        = .5;
    private static final double TY_TARGET          = -3.52;
    private static final double TY_DEADBAND        = .3;

    // Phase 1: Drive fwd/back to TY_TARGET
    private static final double ALIGN_DRIVE_KP             = .3;
    private static final double ALIGN_MAX_DRIVE            = 1;
    private static final double ALIGN_TRACK_TURN_KP        = 0.040;
    private static final double ALIGN_MAX_TRACK_TURN       = 0.5;
    private static final double ALIGN_DISTANCE_TIMEOUT_SEC = 2.0;

    // Phase 2: Strafe/turn to TX_TARGET
    private static final double ALIGN_STRAFE_KP            = 0.06;
    private static final double ALIGN_MAX_STRAFE           = 0.60;
    private static final double ALIGN_TURN_KP              = 0.04;
    private static final double ALIGN_MAX_TURN             = 0.45;
    private static final double ALIGN_LATERAL_TIMEOUT_SEC  = 2.5;

    // Blind reverse at start
    private static final double REVERSE_SEARCH_POWER       = -0.5;
    private static final double REVERSE_SEARCH_TIMEOUT_SEC = 5.0;

    // ─── Strafe to Collection Point (TUNE THESE TO YOUR FIELD) ──────────────────
    private static final double STRAFE_TO_COLLECT_POWER = -.5;
    private static final double STRAFE_TO_COLLECT_SEC   = 1.3;
    private static final double DRIVE_TO_COLLECT_POWER  = 0.3;
    private static final double DRIVE_TO_COLLECT_SEC    = 0.5;

    // ─── Strafe Back to Shoot Position (TUNE THESE TO YOUR FIELD) ───────────────
    private static final double STRAFE_TO_SHOOT_POWER   = .5;
    private static final double STRAFE_TO_SHOOT_SEC     = 1.3;
    private static final double DRIVE_TO_SHOOT_POWER    = -0.3;
    private static final double DRIVE_TO_SHOOT_SEC      = 0.5;

    // ─── Field Pickup Maneuver (every 2 cycles) — TUNE THESE ────────────────────
    private static final int    CYCLES_BEFORE_FIELD_PICKUP = 2;
    private static final double FIELD_TURN_POWER           = 0.5;
    private static final double FIELD_TURN_SEC             = 0.9;
    private static final double FIELD_DRIVE1_POWER         = 0.4;
    private static final double FIELD_DRIVE1_SEC           = 0.8;
    private static final double FIELD_DIVERT_SEC           = 0.3;
    private static final double FIELD_DRIVE2_POWER         = 0.4;
    private static final double FIELD_DRIVE2_SEC           = 1.0;
    private static final double FIELD_REVERSE_POWER        = -0.4;
    private static final double FIELD_REVERSE_SEC          = 1.8;
    private static final double FIELD_TURN_BACK_POWER      = -0.5;
    private static final double FIELD_TURN_BACK_SEC        = 0.9;

    // ─── Match Timer ────────────────────────────────────────────────────────────
    private static final double MATCH_DURATION_SEC = 30.0;
    private static final double CYCLE_RESERVE_SEC  = 4.5;

    private ElapsedTime matchTimer = new ElapsedTime();
    private int cycleCount = 0;

    // ========================================================================
    @Override
    public void runOpMode() {

        initHardware();
        initLimelight();

        telemetry.addLine("Ready — waiting for start");
        telemetry.update();
        waitForStart();

        matchTimer.reset();

        // ── ONE-TIME: if target not visible, reverse until it is ─────────────
        if (!canSeeTarget()) {
            telemetry.addLine("No target — reversing...");
            telemetry.update();
            reverseUntilTargetFound();
        }

        // ── Main autonomous loop ─────────────────────────────────────────────
        while (opModeIsActive() && matchTimer.seconds() < MATCH_DURATION_SEC) {

            double timeLeft = MATCH_DURATION_SEC - matchTimer.seconds();
            if (timeLeft < CYCLE_RESERVE_SEC) {
                telemetry.addLine("Time nearly up — stopping.");
                telemetry.update();
                break;
            }

            // 1. Align: distance first, then lateral
            telemetry.addLine("Aligning to target...");
            telemetry.update();
            alignToTarget();

            if (!opModeIsActive()) break;

            // 2. Shoot: 1 right, then 2 left (intake on during shooting)
            telemetry.addLine("Shooting...");
            telemetry.update();
            shootBalls();

            if (!opModeIsActive()) break;

            cycleCount++;

            // 3. Every N cycles, do field pickup maneuver instead of normal collect
            if (cycleCount % CYCLES_BEFORE_FIELD_PICKUP == 0) {
                telemetry.addLine("Field pickup maneuver...");
                telemetry.update();
                fieldPickupManeuver();
            } else {
                // Normal: strafe to collect, collect, strafe back
                telemetry.addLine("Moving to collection point...");
                telemetry.update();
                strafeToCollectionPoint();

                if (!opModeIsActive()) break;

                telemetry.addLine("Collecting...");
                telemetry.update();
                collectBalls(INTAKE_DURATION_SEC);

                if (!opModeIsActive()) break;

                telemetry.addLine("Returning to shoot area...");
                telemetry.update();
                strafeToShootPosition();
            }

            if (!opModeIsActive()) break;

            // 4. Re-align to Limelight target after collecting
            telemetry.addLine("Re-aligning to target...");
            telemetry.update();
            alignToTarget();
        }

        stopAll();
        telemetry.addLine("Autonomous complete.");
        telemetry.update();
    }

    // ========================================================================
    //  HARDWARE INIT
    // ========================================================================
    private void initHardware() {
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");

        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RShootMotor = hardwareMap.get(DcMotor.class, "RShootMotor");
        LShootMotor = hardwareMap.get(DcMotor.class, "LShootMotor");
        LShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        Servo_Fix_L   = hardwareMap.get(CRServo.class, "Servo_Fix_L");
        Servo_Fix_R   = hardwareMap.get(CRServo.class, "Servo_Fix_R");
        Servo_sorting = hardwareMap.get(Servo.class,   "Servo_sorting");
        Servo_Fix_R.setDirection(CRServo.Direction.REVERSE);

        Servo_Fix_L.setPower(0);
        Servo_Fix_R.setPower(0);
        Servo_sorting.setPosition(SORT_LEFT_POS);
    }

    // ========================================================================
    //  LIMELIGHT INIT
    // ========================================================================
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        sleep(250);
    }

    // ========================================================================
    //  CAN SEE TARGET?
    // ========================================================================
    private boolean canSeeTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    // ========================================================================
    //  REVERSE UNTIL TARGET FOUND (only called once at start)
    // ========================================================================
    private void reverseUntilTargetFound() {
        ElapsedTime searchTimer = new ElapsedTime();

        while (opModeIsActive() && searchTimer.seconds() < REVERSE_SEARCH_TIMEOUT_SEC) {
            setDrivePower(REVERSE_SEARCH_POWER, 0);

            if (canSeeTarget()) {
                setDrivePower(0, 0);
                return;
            }

            telemetry.addLine("Backing up to find target...");
            telemetry.addData("Search time", "%.1f / %.1f s",
                    searchTimer.seconds(), REVERSE_SEARCH_TIMEOUT_SEC);
            telemetry.update();
            sleep(50);
        }

        setDrivePower(0, 0);
    }

    // ========================================================================
    //  ALIGN TO TARGET  (two phases)
    // ========================================================================
    private void alignToTarget() {
        alignDistance();
        if (!opModeIsActive()) return;
        alignLateral();
    }

    // ========================================================================
    //  PHASE 1 — Drive fwd/back to TY_TARGET
    // ========================================================================
    private void alignDistance() {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < ALIGN_DISTANCE_TIMEOUT_SEC) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                setMecanumPower(0, 0, 0);
                telemetry.addLine("Phase 1: no target — stopped");
                telemetry.update();
                sleep(50);
                continue;
            }

            double tx = result.getTx();
            double ty = result.getTy();
            double tyError = ty - TY_TARGET;

            boolean atDistance = Math.abs(tyError) < TY_DEADBAND;

            telemetry.addData("Phase", "1 — Distance");
            telemetry.addData("ty", "%.2f  (target %.2f)", ty, TY_TARGET);
            telemetry.addData("tx", "%.2f  (tracking)", tx);
            telemetry.addData("tyError", "%.2f", tyError);
            telemetry.addData("At distance", atDistance);
            telemetry.update();

            if (atDistance) {
                setMecanumPower(0, 0, 0);
                break;
            }

            double drivePower = clamp(-tyError * ALIGN_DRIVE_KP,
                    -ALIGN_MAX_DRIVE, ALIGN_MAX_DRIVE);

            double turnPower = clamp(tx * ALIGN_TRACK_TURN_KP,
                    -ALIGN_MAX_TRACK_TURN, ALIGN_MAX_TRACK_TURN);

            setMecanumPower(drivePower, 0, turnPower);
            sleep(20);
        }

        setMecanumPower(0, 0, 0);
    }

    // ========================================================================
    //  PHASE 2 — Strafe / turn to TX_TARGET
    // ========================================================================
    private void alignLateral() {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < ALIGN_LATERAL_TIMEOUT_SEC) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                setMecanumPower(0, 0, 0);
                telemetry.addLine("Phase 2: no target");
                telemetry.update();
                sleep(50);
                continue;
            }

            double tx = result.getTx();
            double txError = tx - TX_TARGET;

            boolean aimed = Math.abs(txError) < TX_DEADBAND;

            telemetry.addData("Phase", "2 — Lateral");
            telemetry.addData("tx", "%.2f  (target %.2f)", tx, TX_TARGET);
            telemetry.addData("txError", "%.2f", txError);
            telemetry.addData("Aimed", aimed);
            telemetry.update();

            if (aimed) {
                setMecanumPower(0, 0, 0);
                break;
            }

            double strafePower = clamp(txError * ALIGN_STRAFE_KP,
                    -ALIGN_MAX_STRAFE, ALIGN_MAX_STRAFE);

            double turnPower = clamp(tx * ALIGN_TURN_KP,
                    -ALIGN_MAX_TURN, ALIGN_MAX_TURN);

            setMecanumPower(0, strafePower, turnPower);
            sleep(20);
        }

        setMecanumPower(0, 0, 0);
    }

    // ========================================================================
    //  SHOOT — 1 right, then 2 left (intake runs during shooting)
    // ========================================================================
    private void shootBalls() {
        IntakeMotor.setPower(INTAKE_POWER);

        RShootMotor.setPower(SHOOT_POWER);
        LShootMotor.setPower(SHOOT_POWER);
        sleep((long)(SHOOT_SPIN_UP_SEC * 1000));

        // ── Shot 1: RIGHT ───────────────────────────────────────────────────
        Servo_sorting.setPosition(SORT_RIGHT_POS);
        sleep(100);
        Servo_Fix_R.setPower(FEED_POWER);
        sleep((long)(FEED_DWELL_SEC * 1000));
        Servo_Fix_R.setPower(0);

        sleep((long)(BETWEEN_SHOT_SEC * 1000));
        telemetry.addData("Shot", "1 / 3 (right)");
        telemetry.update();

        if (!opModeIsActive()) { stopShooter(); IntakeMotor.setPower(0); return; }

        // ── Shot 2: LEFT ────────────────────────────────────────────────────
        Servo_sorting.setPosition(SORT_LEFT_POS);
        sleep(100);
        Servo_Fix_L.setPower(FEED_POWER);
        sleep((long)(FEED_DWELL_SEC * 1000));
        Servo_Fix_L.setPower(0);

        sleep((long)(BETWEEN_SHOT_SEC * 1000));
        telemetry.addData("Shot", "2 / 3 (left)");
        telemetry.update();

        if (!opModeIsActive()) { stopShooter(); IntakeMotor.setPower(0); return; }

        // ── Shot 3: LEFT (diverter already there) ───────────────────────────
        Servo_Fix_L.setPower(FEED_POWER);
        sleep((long)(FEED_DWELL_SEC * 1000));
        Servo_Fix_L.setPower(0);

        telemetry.addData("Shot", "3 / 3 (left)");
        telemetry.update();

        stopShooter();
        IntakeMotor.setPower(0);
        Servo_sorting.setPosition(SORT_LEFT_POS);
    }

    private void stopShooter() {
        RShootMotor.setPower(0);
        LShootMotor.setPower(0);
    }

    // ========================================================================
    //  FIELD PICKUP MANEUVER
    //  Turn ~90°, drive forward, divert, drive forward more with intake,
    //  then reverse back and turn back to face the target.
    // ========================================================================
    private void fieldPickupManeuver() {
        // Turn ~90°
        telemetry.addLine("Field pickup: turning...");
        telemetry.update();
        setMecanumPower(0, 0, FIELD_TURN_POWER);
        sleep((long)(FIELD_TURN_SEC * 1000));
        setMecanumPower(0, 0, 0);

        if (!opModeIsActive()) return;

        // Drive forward (first segment)
        IntakeMotor.setPower(INTAKE_POWER);
        telemetry.addLine("Field pickup: driving forward...");
        telemetry.update();
        setMecanumPower(FIELD_DRIVE1_POWER, 0, 0);
        sleep((long)(FIELD_DRIVE1_SEC * 1000));
        setMecanumPower(0, 0, 0);

        if (!opModeIsActive()) { IntakeMotor.setPower(0); return; }

        // Activate diverter
        telemetry.addLine("Field pickup: diverting...");
        telemetry.update();
        Servo_sorting.setPosition(SORT_RIGHT_POS);
        sleep((long)(FIELD_DIVERT_SEC * 1000));

        if (!opModeIsActive()) { IntakeMotor.setPower(0); return; }

        // Drive forward more with intake running
        telemetry.addLine("Field pickup: collecting forward...");
        telemetry.update();
        setMecanumPower(FIELD_DRIVE2_POWER, 0, 0);
        sleep((long)(FIELD_DRIVE2_SEC * 1000));
        setMecanumPower(0, 0, 0);

        IntakeMotor.setPower(0);
        Servo_sorting.setPosition(SORT_LEFT_POS);

        if (!opModeIsActive()) return;

        // Reverse back
        telemetry.addLine("Field pickup: reversing...");
        telemetry.update();
        setMecanumPower(FIELD_REVERSE_POWER, 0, 0);
        sleep((long)(FIELD_REVERSE_SEC * 1000));
        setMecanumPower(0, 0, 0);

        if (!opModeIsActive()) return;

        // Turn back ~90° (opposite direction)
        telemetry.addLine("Field pickup: turning back...");
        telemetry.update();
        setMecanumPower(0, 0, FIELD_TURN_BACK_POWER);
        sleep((long)(FIELD_TURN_BACK_SEC * 1000));
        setMecanumPower(0, 0, 0);
    }

    // ========================================================================
    //  STRAFE TO COLLECTION POINT (blind — no Limelight)
    // ========================================================================
    private void strafeToCollectionPoint() {
        setMecanumPower(0, STRAFE_TO_COLLECT_POWER, 0);
        sleep((long)(STRAFE_TO_COLLECT_SEC * 1000));

        if (Math.abs(DRIVE_TO_COLLECT_POWER) > 0.01) {
            setMecanumPower(DRIVE_TO_COLLECT_POWER, 0, 0);
            sleep((long)(DRIVE_TO_COLLECT_SEC * 1000));
        }

        setMecanumPower(0, 0, 0);
    }

    // ========================================================================
    //  STRAFE BACK TO SHOOT POSITION (blind — Limelight re-aligns after)
    // ========================================================================
    private void strafeToShootPosition() {
        setMecanumPower(0, STRAFE_TO_SHOOT_POWER, 0);
        sleep((long)(STRAFE_TO_SHOOT_SEC * 1000));

        if (Math.abs(DRIVE_TO_SHOOT_POWER) > 0.01) {
            setMecanumPower(DRIVE_TO_SHOOT_POWER, 0, 0);
            sleep((long)(DRIVE_TO_SHOOT_SEC * 1000));
        }

        setMecanumPower(0, 0, 0);
    }

    // ========================================================================
    //  COLLECT — diverter RIGHT for 1st ball, snap LEFT for other 2
    // ========================================================================
    private void collectBalls(double totalSeconds) {
        Servo_sorting.setPosition(SORT_RIGHT_POS);
        IntakeMotor.setPower(INTAKE_POWER);

        ElapsedTime t = new ElapsedTime();
        boolean switchedToLeft = false;

        while (opModeIsActive() && t.seconds() < totalSeconds) {
            if (!switchedToLeft && t.seconds() >= INTAKE_RIGHT_DWELL_SEC) {
                Servo_sorting.setPosition(SORT_LEFT_POS);
                switchedToLeft = true;
            }

            telemetry.addData("Collecting", "%.1f / %.1f s", t.seconds(), totalSeconds);
            telemetry.addData("Diverter", switchedToLeft ? "LEFT" : "RIGHT");
            telemetry.addData("Match time left", "%.1f s",
                    MATCH_DURATION_SEC - matchTimer.seconds());
            telemetry.update();
            sleep(50);
        }

        IntakeMotor.setPower(0);
    }

    // ========================================================================
    //  DRIVE HELPERS
    // ========================================================================

    /** Tank-style: drive + turn. */
    private void setDrivePower(double drive, double turn) {
        double left  = drive + turn;
        double right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) { left /= max; right /= max; }

        FLmotor.setPower(left);
        BLmotor.setPower(left);
        FRmotor.setPower(right);
        BRmotor.setPower(right);
    }

    /** Mecanum: drive (fwd/back) + strafe (right/left) + turn. */
    private void setMecanumPower(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        FLmotor.setPower(fl);
        FRmotor.setPower(fr);
        BLmotor.setPower(bl);
        BRmotor.setPower(br);
    }

    private void stopAll() {
        setMecanumPower(0, 0, 0);
        RShootMotor.setPower(0);
        LShootMotor.setPower(0);
        IntakeMotor.setPower(0);
        Servo_Fix_L.setPower(0);
        Servo_Fix_R.setPower(0);
        limelight.stop();
    }

    // ========================================================================
    //  UTIL
    // ========================================================================
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
