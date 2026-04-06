/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp
//@Disabled
public class OneDriver extends OpMode {
    final double FEED_TIME_SECONDS = 0.75; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1150; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1125; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1400; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1375; //minimum required to start a shot for far goal.

    final double LAUNCHER_KILLER_TARGET_VELOCITY = 2000;
    final double LAUNCHER_KILLER_MIN_VELOCITY = 1975;

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 1; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    // Declare OpMode members.
    private DcMotor FLmotor = null;
    private DcMotor FRmotor = null;
    private DcMotor BLmotor = null;
    private DcMotor BRmotor = null;
    private DcMotorEx LShootMotor = null;
    private DcMotorEx RShootMotor = null;
    private DcMotor intake = null;
    private CRServo Servo_Fix_L = null;
    private CRServo Servo_Fix_R = null;
    private Servo diverter = null;

    private Servo light = null;
    ElapsedTime Servo_Fix_LTimer = new ElapsedTime();
    ElapsedTime Servo_Fix_RTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR,
        KILLER;
    }

    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        LShootMotor = hardwareMap.get(DcMotorEx.class, "LShootMotor");
        RShootMotor = hardwareMap.get(DcMotorEx.class, "RShootMotor");
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        Servo_Fix_L = hardwareMap.get(CRServo.class, "Servo_Fix_L");
        Servo_Fix_R = hardwareMap.get(CRServo.class, "Servo_Fix_R");
        diverter = hardwareMap.get(Servo.class, "Servo_sorting");
        light = hardwareMap.get(Servo.class, "light");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        FRmotor.setDirection(DcMotor.Direction.FORWARD);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);
        BRmotor.setDirection(DcMotor.Direction.FORWARD);

        LShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        LShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        FLmotor.setZeroPowerBehavior(BRAKE);
        FRmotor.setZeroPowerBehavior(BRAKE);
        BLmotor.setZeroPowerBehavior(BRAKE);
        BRmotor.setZeroPowerBehavior(BRAKE);
        LShootMotor.setZeroPowerBehavior(BRAKE);
        RShootMotor.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        Servo_Fix_L.setPower(STOP_SPEED);
        Servo_Fix_R.setPower(STOP_SPEED);


        LShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        RShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        Servo_Fix_L.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

        //Light
        light.setPosition(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            LShootMotor.setVelocity(launcherTarget);
            RShootMotor.setVelocity(launcherTarget);
        } else if (gamepad1.b) { // stop flywheel
            LShootMotor.setVelocity(STOP_SPEED);
            RShootMotor.setVelocity(STOP_SPEED);
        }

        if (gamepad1.dpadDownWasPressed()) {
            switch (diverterDirection){
                case LEFT:
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    break;
                case RIGHT:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    break;
            }
        }

        if (gamepad1.aWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(1);
                    break;
            }
        }
        //-----------DUMP-----------
        if (gamepad1.xWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.ON;
                    intake.setPower(-1);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(-1);
                    break;
            }
        }
        //-----------DUMP-----------

        if (gamepad1.dpadUpWasPressed()) {
            switch (launcherDistance) {
                case CLOSE:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                    light.setPosition(0.666);
                    break;
                case FAR:
                    launcherDistance = LauncherDistance.KILLER;
                    launcherTarget = LAUNCHER_KILLER_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_KILLER_MIN_VELOCITY;
                    light.setPosition(0.279);
                    break;
                case KILLER:
                    launcherDistance = LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    light.setPosition(0.357);
                    break;
            }
        }

        /*
         * Now we call our "Launch" function.
         */
        launchLeft(gamepad1.leftBumperWasPressed());
        launchRight(gamepad1.rightBumperWasPressed());


        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", leftLaunchState);
        telemetry.addData("launch distance", launcherDistance);
        telemetry.addData("Left Launcher Velocity", LShootMotor.getVelocity());
        telemetry.addData("Right Launcher Velocity", RShootMotor.getVelocity());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        FLmotor.setPower(leftFrontPower);
        FRmotor.setPower(rightFrontPower);
        BLmotor.setPower(leftBackPower);
        BRmotor.setPower(rightBackPower);

    }

    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                LShootMotor.setVelocity(launcherTarget);
                RShootMotor.setVelocity(launcherTarget);
                if (LShootMotor.getVelocity() > launcherMin) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                Servo_Fix_L.setPower(FULL_SPEED);
                Servo_Fix_LTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (Servo_Fix_LTimer.seconds() > FEED_TIME_SECONDS) {
                    leftLaunchState = LaunchState.IDLE;
                    Servo_Fix_L.setPower(STOP_SPEED);
                }
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                LShootMotor.setVelocity(launcherTarget);
                RShootMotor.setVelocity(launcherTarget);
                if (LShootMotor.getVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                Servo_Fix_R.setPower(FULL_SPEED);
                Servo_Fix_RTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (Servo_Fix_RTimer.seconds() > FEED_TIME_SECONDS) {
                    rightLaunchState = LaunchState.IDLE;
                    Servo_Fix_R.setPower(STOP_SPEED);
                }
                break;
        }
    }
}