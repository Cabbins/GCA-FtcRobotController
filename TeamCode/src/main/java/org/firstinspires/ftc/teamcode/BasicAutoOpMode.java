package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BasicAutoOpMode extends OpMode {

    private DcMotor FLmotor = null;
    private DcMotor FRmotor = null;
    private DcMotor BLmotor = null;
    private DcMotor BRmotor = null;
    private DcMotor LShootMotor = null;
    private DcMotor RShootMotor = null;
    private DcMotor intake = null;
    private Servo Servo_Fix_L = null;
    private Servo Servo_Fix_R = null;
    private Servo diverter = null;

    ElapsedTime autoTimer = new ElapsedTime();

    public static double RUN_TIME = 0.5;

    @Override
    public void init() {
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        FRmotor.setDirection(DcMotor.Direction.FORWARD);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);
        BRmotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void start() {
        mecanumDrive(0, 0, 0);
        autoTimer.reset();
    }

    @Override
    public void loop() {
        if(autoTimer.seconds() > 1) {
            mecanumDrive(0,0, 1);

        }

    }

    void mecanumDrive(double forward, double strafe, double rotate){

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

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
}
