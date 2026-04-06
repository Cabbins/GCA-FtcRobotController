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
 *  Sequence:
 *   1. Use Limelight (pipeline 8) to align/drive to shoot position
 *   2. Shoot 3 balls (alternating L/R paths via diverter servo)
 *   3. Run intake to collect more balls
 *   4. Return to shoot position and shoot again
 *   5. Repeat until 30-second timer expires
 * ============================================================
 */
@Autonomous(name = "LimelightAuto", group = "Competition")
public class ClaudeAutoOpMode extends LinearOpMode {

    // ─── Drive Motors ───────────────────────────────────────────────────────────
    private DcMotor FRmotor, FLmotor, BRmotor, BLmotor;

    // ─── Shooter Motors ─────────────────────────────────────────────────────────
    private DcMotor RShootMotor, LShootMotor;

    // ─── Intake ─────────────────────────────────────────────────────────────────
    private DcMotor IntakeMotor;

    // ─── Servos ─────────────────────────────────────────────────────────────────
    private CRServo Servo_Fix_L;  // Left feeder CRServo — spins to push ball into left wheel
    private CRServo Servo_Fix_R;  // Right feeder CRServo — spins to push ball into right wheel
    private Servo Servo_sorting;  // Diverter: standard servo, decides left vs right shoot path

    // ─── Limelight ──────────────────────────────────────────────────────────────
    private Limelight3A limelight;
    private static final int LIMELIGHT_PIPELINE = 8;

    // ─── Tunable Constants ──────────────────────────────────────────────────────
    // Shooter
    private static final double SHOOT_POWER        = 0.8;  // Motor power for shooter wheels
    private static final double SHOOT_SPIN_UP_SEC  = 1.2;   // Seconds to spin up wheels before feeding
    private static final double FEED_POWER         = -1.0;   // CRServo power to push ball forward
    private static final double FEED_DWELL_SEC     = 0.35;  // How long CRServo runs to push one ball
    private static final double SORT_LEFT_POS      = 0.25;  // Diverter → left path
    private static final double SORT_RIGHT_POS     = 0.75;  // Diverter → right path
    //private static final double FEED_DWELL_SEC     = 0.35;  // How long servo holds "out" position
    private static final double BETWEEN_SHOT_SEC   = 0.4;   // Pause between shots
    private static final int    BALLS_PER_CYCLE    = 3;     // Balls to shoot each cycle

    // Intake
    private static final double INTAKE_POWER       = -1.0;
    private static final double INTAKE_DURATION_SEC = 3.5;  // Seconds to run intake per collection

    // Limelight / Drive alignment
    private static final double TX_DEADBAND         = 1.5;   // degrees — acceptable aim error
    private static final double TY_TARGET           = -4.6; // desired vertical angle (tune to field)
    private static final double TY_DEADBAND         = 2.0;   // degrees — acceptable distance error
    private static final double ALIGN_TURN_KP       = 0.030; // P-gain for turning to target
    private static final double ALIGN_DRIVE_KP      = 0.025; // P-gain for driving to target distance
    private static final double ALIGN_MAX_TURN      = 0.40;  // Max turn power during alignment
    private static final double ALIGN_MAX_DRIVE     = 0.3;  // Max drive power during alignment
    private static final double ALIGN_TIMEOUT_SEC   = 4.0;   // Give up aligning after this long

    // ─── Match Timer ────────────────────────────────────────────────────────────
    private static final double MATCH_DURATION_SEC  = 30.0;
    // Reserve time so we don't start a cycle we can't finish
    private static final double CYCLE_RESERVE_SEC   = 8.0;

    private ElapsedTime matchTimer = new ElapsedTime();

    // ========================================================================
    @Override
    public void runOpMode() {

        initHardware();
        initLimelight();

        telemetry.addLine("Ready — waiting for start");
        telemetry.update();
        waitForStart();

        matchTimer.reset();

        // ── Main autonomous loop ─────────────────────────────────────────────
        while (opModeIsActive() && matchTimer.seconds() < MATCH_DURATION_SEC) {

            double timeLeft = MATCH_DURATION_SEC - matchTimer.seconds();

            // Don't start another cycle if we're almost out of time
            if (timeLeft < CYCLE_RESERVE_SEC) {
                telemetry.addLine("Time nearly up — stopping.");
                telemetry.update();
                break;
            }

            // 1. Drive to shoot position using Limelight
            telemetry.addLine("Aligning to target...");
            telemetry.update();
            alignToTarget();

            if (!opModeIsActive()) break;

            // 2. Shoot BALLS_PER_CYCLE balls
            telemetry.addLine("Shooting...");
            telemetry.update();
            shootBalls(BALLS_PER_CYCLE);

            if (!opModeIsActive()) break;

            // 3. Collect more balls with intake
            telemetry.addLine("Collecting...");
            telemetry.update();
            runIntake(INTAKE_DURATION_SEC);
        }

        // Safety: stop all motors
        stopAll();
        telemetry.addLine("Autonomous complete.");
        telemetry.update();
    }

    // ========================================================================
    //  HARDWARE INIT
    // ========================================================================
    private void initHardware() {
        // Drive
        FRmotor  = hardwareMap.get(DcMotor.class, "FRmotor");
        FLmotor  = hardwareMap.get(DcMotor.class, "FLmotor");
        BRmotor  = hardwareMap.get(DcMotor.class, "BRmotor");
        BLmotor  = hardwareMap.get(DcMotor.class, "BLmotor");

        // Reverse left-side motors so positive power = forward
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter
        RShootMotor = hardwareMap.get(DcMotor.class, "RShootMotor");
        LShootMotor = hardwareMap.get(DcMotor.class, "LShootMotor");
        LShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        // Servos
        Servo_Fix_L   = hardwareMap.get(CRServo.class, "Servo_Fix_L");
        Servo_Fix_R   = hardwareMap.get(CRServo.class, "Servo_Fix_R");
        Servo_sorting = hardwareMap.get(Servo.class,   "Servo_sorting");

        // Stop CRServos at rest (setPower(0) = stopped)
        Servo_Fix_L.setPower(0);
        Servo_Fix_R.setPower(0);
        Servo_sorting.setPosition(SORT_LEFT_POS); // default diverter to left path
    }

    // ========================================================================
    //  LIMELIGHT INIT
    // ========================================================================
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        sleep(250); // brief settle time
    }

    // ========================================================================
    //  ALIGN TO TARGET  (P-loop on Limelight tx and ty)
    // ========================================================================
    private void alignToTarget() {
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();

        while (opModeIsActive() && alignTimer.seconds() < ALIGN_TIMEOUT_SEC) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                // No target — stop and wait a frame
                setDrivePower(0, 0);
                telemetry.addLine("Limelight: no target detected");
                telemetry.update();
                sleep(50);
                continue;
            }

            double tx = result.getTx(); // horizontal offset (degrees)
            double ty = result.getTy(); // vertical offset (degrees)

            boolean aligned   = Math.abs(tx) < TX_DEADBAND;
            boolean atDistance = Math.abs(ty - TY_TARGET) < TY_DEADBAND;

            telemetry.addData("tx", "%.2f°", tx);
            telemetry.addData("ty", "%.2f°  (target %.2f°)", ty, TY_TARGET);
            telemetry.addData("Aligned", aligned);
            telemetry.addData("At distance", atDistance);
            telemetry.update();

            if (aligned && atDistance) {
                setDrivePower(0, 0);
                break; // We're in position
            }

            // P-controller: turn to zero tx, drive to reach TY_TARGET
            double turnPower  = aligned   ? 0 : clamp(-tx * ALIGN_TURN_KP, -ALIGN_MAX_TURN, ALIGN_MAX_TURN);
            double drivePower = atDistance ? 0 : clamp((ty - TY_TARGET) * ALIGN_DRIVE_KP, -ALIGN_MAX_DRIVE, ALIGN_MAX_DRIVE);

            setDrivePower(drivePower, turnPower);
            sleep(20);
        }

        setDrivePower(0, 0);
    }

    // ========================================================================
    //  SHOOT  (alternates left / right paths)
    // ========================================================================
    private void shootBalls(int count) {
        // Spin up shooter wheels
        RShootMotor.setPower(SHOOT_POWER);
        LShootMotor.setPower(SHOOT_POWER);
        sleep((long)(SHOOT_SPIN_UP_SEC * 1000));

        for (int i = 0; i < count && opModeIsActive(); i++) {
            boolean useLeft = (i % 2 == 0); // alternate paths

            if (useLeft) {
                Servo_sorting.setPosition(SORT_LEFT_POS);
                sleep(100); // let diverter settle
                Servo_Fix_L.setPower(FEED_POWER);           // spin to push ball
                sleep((long)(FEED_DWELL_SEC * 1000));
                Servo_Fix_L.setPower(0);                    // stop
            } else {
                Servo_sorting.setPosition(SORT_RIGHT_POS);
                sleep(100);
                Servo_Fix_R.setPower(FEED_POWER);           // spin to push ball
                sleep((long)(FEED_DWELL_SEC * 1000));
                Servo_Fix_R.setPower(0);                    // stop
            }

            sleep((long)(BETWEEN_SHOT_SEC * 1000));

            telemetry.addData("Shot", "%d / %d", i + 1, count);
            telemetry.update();
        }

        // Spin down
        RShootMotor.setPower(0);
        LShootMotor.setPower(0);

        // Reset diverter to default
        Servo_sorting.setPosition(SORT_LEFT_POS);
    }

    // ========================================================================
    //  INTAKE
    // ========================================================================
    private void runIntake(double seconds) {
        IntakeMotor.setPower(INTAKE_POWER);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            telemetry.addData("Intake", "%.1f / %.1f s", t.seconds(), seconds);
            telemetry.addData("Match time left", "%.1f s", MATCH_DURATION_SEC - matchTimer.seconds());
            telemetry.update();
            sleep(50);
        }
        IntakeMotor.setPower(0);
    }

    // ========================================================================
    //  DRIVE HELPERS
    // ========================================================================
    /**
     * Tank-style drive where drivePower controls forward/back
     * and turnPower controls left/right rotation.
     */
    private void setDrivePower(double drive, double turn) {
        double left  = drive + turn;
        double right = drive - turn;

        // Normalize if either side exceeds 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) { left /= max; right /= max; }

        FLmotor.setPower(left);
        BLmotor.setPower(left);
        FRmotor.setPower(right);
        BRmotor.setPower(right);
    }

    private void stopAll() {
        setDrivePower(0, 0);
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
