package org.firstinspires.ftc.teamcode.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;

@TeleOp
public class PIDTele extends OpMode {

    private double looptime = 0, targetVelocity = 0;
    private boolean far = false;
    private double hoodTarget = HOOD_CLOSE;
    private double lKickerTarget = LKICKER_DOWN;
    private double mKickerTarget = MKICKER_DOWN;
    private double rKickerTarget = RKICKER_DOWN;
    private double intakePower = INTAKE_OFF;
//    private double shooterPower = SHOOTER_OFF;

    DcMotor fl, bl, fr, br, intake;
    DcMotorEx lShooter, rShooter;
    Servo lKicker, mKicker, rKicker, hood;

    public void init() {

        fl = hardwareMap.dcMotor.get(ApolloConstants.dt.fl);
        bl = hardwareMap.dcMotor.get(ApolloConstants.dt.bl);
        fr = hardwareMap.dcMotor.get(ApolloConstants.dt.fr);
        br = hardwareMap.dcMotor.get(ApolloConstants.dt.br);
        intake = hardwareMap.dcMotor.get("intake");
        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");

        lKicker = hardwareMap.servo.get("lKicker");
        mKicker = hardwareMap.servo.get("mKicker");
        rKicker = hardwareMap.servo.get("rKicker");
        hood = hardwareMap.servo.get("hood");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rShooter.setMode(RUN_USING_ENCODER);
        lShooter.setMode(RUN_WITHOUT_ENCODER);

        fl.setDirection(flDir);
        bl.setDirection(blDir);
        fr.setDirection(frDir);
        br.setDirection(brDir);
        intake.setDirection(intakeDir);
        lShooter.setDirection(lShooterDir);
        rShooter.setDirection(rShooterDir);

//        liftPID = new PIDController(lp, li, ld);
//        extendoPID = new PIDController(ep, ei, ed);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakePower = INTAKE_OFF;
//        shooterPower = SHOOTER_OFF;

        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);
        hood.setPosition(hoodTarget);
        intake.setPower(intakePower);
//        lShooter.setPower(shooterPower);
//        rShooter.setPower(shooterPower);

        rShooter.setVelocity(targetVelocity, AngleUnit.DEGREES);
        lShooter.setPower(rShooter.getPower());

    }

    public void init_loop(){

        intake.setPower(0);
        lShooter.setPower(0);
        rShooter.setPower(0);
        telemetry.update();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        if (gamepad1.right_bumper) intakePower = INTAKE_IN;
        else if (gamepad1.left_bumper) intakePower = INTAKE_OUT;
        else intakePower = INTAKE_OFF;

        if (gamepad1.x) lKickerTarget = LKICKER_UP; else lKickerTarget = LKICKER_DOWN;
        if (gamepad1.y) mKickerTarget = MKICKER_UP; else mKickerTarget = MKICKER_DOWN;
        if (gamepad1.b) rKickerTarget = RKICKER_UP; else rKickerTarget = RKICKER_DOWN;

        if (gamepad1.dpad_up) far = false;
        if (gamepad1.dpad_down) far = true;

        if(far){
//            shooterPower = SHOOTER_FAR;
            targetVelocity = VELOCITY_FAR;
            hoodTarget = HOOD_FAR;
        } else {
//            shooterPower = SHOOTER_CLOSE;
            targetVelocity = VELOCITY_CLOSE;
            hoodTarget = HOOD_CLOSE;
        }

        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);
        hood.setPosition(hoodTarget);
        intake.setPower(intakePower);
//        lShooter.setPower(shooterPower);
//        rShooter.setPower(shooterPower);

        rShooter.setVelocity(targetVelocity, AngleUnit.DEGREES);
        lShooter.setPower(rShooter.getPower());

//        telemetry.addData("Left Kicker ", lKicker.getPosition());
//        telemetry.addData("Middle Kicker", mKicker.getPosition());
//        telemetry.addData("Right Kicker", rKicker.getPosition());
//        telemetry.addData("Hood", hood.getPosition());
//        telemetry.addData("Intake", intake.getPower());
        telemetry.addData("Left Shooter", lShooter.getPower());
        telemetry.addData("Right Shooter", rShooter.getPower());
        telemetry.addData("Left Shooter Vel", lShooter.getVelocity());
        telemetry.addData("Right Shooter Vel", rShooter.getVelocity());
        telemetry.addData("Shooter Vel Reported", targetVelocity);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - looptime));
        looptime = loop;
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() { super.stop(); }

}