/* Copyright (c) 2025 Your Team. All rights reserved.
 *
 * Autonomous OpMode: Limelight-guided shoot-collect cycle.
 *
 * Hardware assumptions (rename to match your robot config):
 *   Motors   : "FLmotor", "FRmotor", "BLmotor", "BRmotor"  (mecanum)
 *              "IntakeMotor"   – runs intake/collector
 *              "LShootMotor"  – spins up flywheel shooter on Left
 *              "RShootMotor"  _ spins up flywheel shooter on Right
 *   Servos   : "Servo_Fix_L"  – pushes ball into shooter
 *              "Servo_Fix_R"  – pushes ball into shooter on Right
 *              "Servo_sorting" - decides which side the ball is on
 *   Sensor   : "limelight"      – Limelight 3A on USB
 *
 * Limelight pipeline assumptions:
 *   Pipeline 8 →  detects the SHOOT target  (e.g. goal AprilTag or retroreflective tape)
 *
 * Tune the constants in the CONFIG section before your first test run.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Limelight Shoot & Collect", group = "Autonomous")
public class AgustAutoOpMode extends LinearOpMode {

    // =========================================================================
    // CONFIG  –  tune these values for your robot
    // =========================================================================

    // How many full shoot→collect cycles to run
    private static final int TOTAL_CYCLES = 3;

    // How many balls to shoot each cycle before going to collect
    private static final int BALLS_PER_SHOOT_CYCLE = 3;

    // Limelight pipelines
    private static final int PIPELINE_SHOOT_TARGET = 8;


    // Drive gains for Limelight-based alignment (proportional control)
    private static final double TURN_GAIN   = 0.025;  // tx error  → turn power
    private static final double STRAFE_GAIN = 0.020;  // tx error  → strafe power
    private static final double DRIVE_GAIN  = 0.030;  // ty error  → forward power

    // Target ty values (vertical angle from Limelight)
    // Positive ty = target is above centre; tune so robot stops at the right distance
    private static final double SHOOT_TARGET_TY  =  8.0;   // stop when goal is this far up
    private static final double BALL_COLLECT_TY  = -10.0;  // stop when ball is at this angle

    // Alignment dead-zones (degrees) – stop correcting when error is within this band
    private static final double TX_DEADZONE = 1.5;
    private static final double TY_DEADZONE = 1.5;

    // Motor speeds
    private static final double SHOOTER_SPEED    = 1.0;
    private static final double INTAKE_IN_SPEED  = 0.8;
    private static final double INTAKE_OUT_SPEED = -0.8;
    private static final double MAX_DRIVE_SPEED  = 0.5;

    // Shooter spin-up time (ms) before indexing a ball
    private static final int SHOOTER_SPINUP_MS = 1200;

    // Indexer servo positions
    private static final double INDEXER_REST  = 0.0;
    private static final double INDEXER_PUSH  = 0.5;
    private static final double INDEXER_DWELL_MS = 300;   // time to hold push position

    // Collection drive time (ms) to scoop balls once aligned
    private static final int COLLECT_DRIVE_MS = 1500;

    // Timeout for each alignment phase (ms) – abort if we can't find the target
    private static final int ALIGN_TIMEOUT_MS = 5000;

    // =========================================================================
    // HARDWARE
    // =========================================================================

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor, shooterMotorR, shooterMotorL;
    private Servo   indexerServoR, indexerServoL;
    private Limelight3A limelight;

    private final ElapsedTime timer = new ElapsedTime();

    // =========================================================================
    // MAIN
    // =========================================================================

    @Override
    public void runOpMode() {

        initHardware();

        telemetry.addLine("Ready. Press PLAY to start.");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // ── Main cycle loop ─────────────────────────────────────────────────
        for (int cycle = 0; cycle < TOTAL_CYCLES && opModeIsActive(); cycle++) {

            telemetry.addData("Cycle", "%d / %d", cycle + 1, TOTAL_CYCLES);
            telemetry.update();

            // 1. Navigate to the shooting position using Limelight
            alignToTarget(PIPELINE_SHOOT_TARGET,
                    SHOOT_TARGET_TY,
                    "Aligning to goal…");

            // 2. Shoot BALLS_PER_SHOOT_CYCLE balls
            shootBalls(BALLS_PER_SHOOT_CYCLE);


            // 3. Drive forward while running intake to collect
            collectBalls();
        }

        // ── Cleanup ─────────────────────────────────────────────────────────
        stopAll();
        limelight.stop();
        telemetry.addLine("Autonomous complete.");
        telemetry.update();
    }

    // =========================================================================
    // PHASE METHODS
    // =========================================================================

    /**
     * Uses Limelight proportional control to align the robot to a target.
     * Switches to the requested pipeline, then drives until both tx and ty
     * are inside their dead-zones or the timeout expires.
     *
     * @param pipeline  Limelight pipeline index to use
     * @param targetTy  Desired vertical angle (ty) at which to stop
     * @param label     Telemetry label for this phase
     */
    private void alignToTarget(int pipeline, double targetTy, String label) {

        limelight.pipelineSwitch(pipeline);
        sleep(200); // let pipeline switch settle

        timer.reset();

        while (opModeIsActive() && timer.milliseconds() < ALIGN_TIMEOUT_MS) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                // Target not visible – spin slowly in place to search
                setDrivePowers(0, 0, 0.2, 0);
                telemetry.addData(label, "Searching…");
                telemetry.update();
                continue;
            }

            double tx = result.getTx(); // horizontal error  (+ = target right of centre)
            double ty = result.getTy(); // vertical error    (+ = target above centre)

            double tyError = ty - targetTy;

            // Calculate drive components
            double turn   = -tx       * TURN_GAIN;    // negative: turn toward target
            double drive  = -tyError  * DRIVE_GAIN;   // negative: drive toward target
            double strafe = 0;                         // optional: use tx for strafing instead

            // Clamp to safe speed
            drive  = clamp(drive,  -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            turn   = clamp(turn,   -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            strafe = clamp(strafe, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

            // Check dead-zones
            boolean txOk = Math.abs(tx)      < TX_DEADZONE;
            boolean tyOk = Math.abs(tyError) < TY_DEADZONE;

            if (txOk && tyOk) {
                stopDrive();
                telemetry.addData(label, "Aligned!");
                telemetry.update();
                break;
            }

            setDrivePowers(drive, strafe, turn, 0);

            telemetry.addData(label, "tx=%.1f  ty=%.1f  tyErr=%.1f", tx, ty, tyError);
            telemetry.update();
        }

        stopDrive();
    }

    /**
     * Spins up the shooter and indexes the requested number of balls.
     */
    private void shootBalls(int count) {

        telemetry.addData("Shooting", "%d balls", count);
        telemetry.update();

        // Spin up flywheel
        shooterMotorR.setPower(SHOOTER_SPEED);
        sleep(SHOOTER_SPINUP_MS);
        shooterMotorL.setPower(SHOOTER_SPEED);
        sleep(SHOOTER_SPINUP_MS);


        for (int i = 0; i < count && opModeIsActive(); i++) {

            // Push ball into shooter
            indexerServoL.setPosition(INDEXER_PUSH);
            sleep((long) INDEXER_DWELL_MS);

            // Retract indexer
            indexerServoL.setPosition(INDEXER_REST);
            sleep((long) INDEXER_DWELL_MS);

            telemetry.addData("Shot", "%d / %d", i + 1, count);
            telemetry.update();
        }

        // Stop flywheel
        shooterMotorR.setPower(0);
        shooterMotorL.setPower(0);
    }

    /**
     * Runs the intake and drives forward briefly to scoop up balls.
     */
    private void collectBalls() {

        telemetry.addData("Collecting", "Running intake…");
        telemetry.update();

        intakeMotor.setPower(INTAKE_IN_SPEED);

        // Drive forward slowly while intake runs
        setDrivePowers(0.3, 0, 0, 0);
        sleep(COLLECT_DRIVE_MS);

        stopDrive();
        intakeMotor.setPower(0);
    }

    // =========================================================================
    // DRIVE HELPERS
    // =========================================================================

    /**
     * Mecanum drive power distribution.
     *
     * @param fwd    forward/backward  (+1 = forward)
     * @param strafe left/right strafe (+1 = right)
     * @param turn   rotation          (+1 = clockwise)
     * @param unused reserved for future use
     */
    private void setDrivePowers(double fwd, double strafe, double turn, double unused) {
        double fl = fwd + strafe + turn;
        double fr = fwd - strafe - turn;
        double bl = fwd - strafe + turn;
        double br = fwd + strafe - turn;

        // Normalise if any value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl  / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl   / max);
        backRight.setPower(br  / max);
    }

    private void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

    private void stopAll() {
        stopDrive();
        intakeMotor.setPower(0);
        shooterMotorR.setPower(0);
        shooterMotorL.setPower(0);
        indexerServoL.setPosition(INDEXER_REST);
        indexerServoR.setPosition(INDEXER_REST);
    }

    // =========================================================================
    // HARDWARE INITIALISATION
    // =========================================================================

    private void initHardware() {

        // Drive motors
        frontLeft  = hardwareMap.get(DcMotor.class, "FLmotor");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backLeft   = hardwareMap.get(DcMotor.class, "BLmotor");
        backRight  = hardwareMap.get(DcMotor.class, "BRmotor");

        // Left side motors run in reverse on most mecanum drivetrains
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Mechanism motors
        intakeMotor  = hardwareMap.get(DcMotor.class, "IntakeMotor");
        shooterMotorL = hardwareMap.get(DcMotor.class, "LShootMotor");
        shooterMotorR = hardwareMap.get(DcMotor.class, "RShootMotor");

        // Indexer servo
        indexerServoL = hardwareMap.get(Servo.class, "Servo_Fix_L");
        indexerServoL.setPosition(INDEXER_REST);
        indexerServoR = hardwareMap.get(Servo.class, "Servo_Fix_R");
        indexerServoR. setPosition(INDEXER_REST);
        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_SHOOT_TARGET);
        limelight.start();

        telemetry.addData("Hardware", "Initialized");
        telemetry.update();
    }

    // =========================================================================
    // UTILITY
    // =========================================================================

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}