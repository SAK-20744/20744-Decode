package org.firstinspires.ftc.teamcode.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;
import static org.firstinspires.ftc.teamcode.util.Alliance.*;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp
public class PIDTele extends OpMode {

    private double looptime = 0, targetVelocity = 0;
    private boolean far = false;
    Shooter shooter;
//    private double hoodTarget = HOOD_CLOSE;
    private double lKickerTarget = LKICKER_DOWN;
    private double mKickerTarget = MKICKER_DOWN;
    private double rKickerTarget = RKICKER_DOWN;
    private double intakePower = INTAKE_OFF;

    private double turretTarget = TURRET_MIDDLE;
//    private double shooterPower = SHOOTER_OFF;

    private Limelight3A l;
    private Alliance a = RED;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;

    DcMotor fl, bl, fr, br, intake;
    DcMotorEx turret;
    Servo lKicker, mKicker, rKicker/*, hood*/;

    private PIDController turretPIDLarge, turretPIDSmall;

    public void init() {

        fl = hardwareMap.dcMotor.get(ApolloConstants.dt.fl);
        bl = hardwareMap.dcMotor.get(ApolloConstants.dt.bl);
        fr = hardwareMap.dcMotor.get(ApolloConstants.dt.fr);
        br = hardwareMap.dcMotor.get(ApolloConstants.dt.br);
        intake = hardwareMap.dcMotor.get("intake");

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        l = hardwareMap.get(Limelight3A.class, "limelight");

        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);
//        hood = hardwareMap.servo.get("hood");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        turret.setTargetPosition(TURRET_MIDDLE);
//        turret.setMode(RUN_TO_POSITION);

        fl.setDirection(flDir);
        bl.setDirection(blDir);
        fr.setDirection(frDir);
        br.setDirection(brDir);
        turret.setDirection(turretDir);
        intake.setDirection(intakeDir);


        turretPIDLarge = new PIDController(tpl, til, tdl);
        turretPIDSmall = new PIDController(tps, tis, tds);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakePower = INTAKE_OFF;
//        shooterPower = SHOOTER_OFF;

        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);
//        hood.setPosition(hoodTarget);
        intake.setPower(intakePower);
//        lShooter.setPower(shooterPower);
//        rShooter.setPower(shooterPower);



    }

    public void init_loop(){

        intake.setPower(0);
        telemetry.update();

        if (gamepad1.x)
            resetTurret();

        if(gamepad1.dpad_left)
            a=BLUE;
        if(gamepad1.dpad_right)
            a=RED;

        telemetry.addData("TurretTargetPos", turret.getTargetPosition());
        telemetry.addData("TurretRealPos", turret.getCurrentPosition());
        telemetry.addData("Turrettarget", turretTarget);
        telemetry.addData("Alliance", a);
        telemetry.update();

    }

    @Override
    public void loop() {

        boolean reverse = false;
        if(gamepad1.dpad_left)
            reverse = true;
        if(gamepad1.dpad_right)
            reverse = false;

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;


        if(reverse){
            y*=-1;
            x*=-1;
        }

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

//        if (gamepad1.dpad_up) far = false;
//        if (gamepad1.dpad_down) far = true;

        if (gamepad1.dpad_up) far = false;
        if (gamepad1.dpad_down) far = true;

        if(far){
            shooter.up();
            shooter.far();
//            shooterPower = SHOOTER_FAR;
            targetVelocity = VELOCITY_FAR;
//            hoodTarget = HOOD_FAR;

            if(a==RED)
                turretTarget = angleFromRed() * 24746.6674795/180 - OFFSET;
            else if(a==BLUE)
                turretTarget = angleFromBlue() * 24746.6674795/180 - OFFSET;
            else
                turretTarget = TURRET_MIDDLE;


        } else {
            shooter.down();
            shooter.close();
//            shooterPower = SHOOTER_CLOSE;
            targetVelocity = VELOCITY_CLOSE;
//            hoodTarget = HOOD_CLOSE;


            if(a==RED)
                turretTarget = angleFromRed() * 24746.6674795/180 - OFFSET;
            else if(a==BLUE)
                turretTarget = angleFromBlue() * 24746.6674795/180 - OFFSET;
            else
                turretTarget = TURRET_MIDDLE;


        }




        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);
//        hood.setPosition(hoodTarget);
        intake.setPower(intakePower);

//        turret.setPower(1);
//        turret.setTargetPosition((int) turretTarget);
        shooter.periodic();
//        lShooter.setPower(shooterPower);
//        rShooter.setPower(shooterPower);

        turretPIDSmall.setPID(tps,tis,tds);
        turretPIDLarge.setPID(tpl,til,tdl);
        int tpos = turret.getCurrentPosition();
        double tpower;
        if(Math.abs(tpos - turretTarget) > TURRET_THRESHOLD)
            tpower = turretPIDLarge.calculate(tpos, turretTarget);
        else
            tpower = turretPIDSmall.calculate(tpos, turretTarget);
        telemetry.addData("tPower", tpower);
        turret.setPower(tpower);



//        rShooter.setVelocity(targetVelocity, AngleUnit.DEGREES);
// replaced default velocity with PID variant. lets try this in tele later.


//        telemetry.addData("Left Kicker ", lKicker.getPosition());
//        telemetry.addData("Middle Kicker", mKicker.getPosition());
//        telemetry.addData("Right Kicker", rKicker.getPosition());
//        telemetry.addData("Hood", hood.getPosition());
//        telemetry.addData("Intake", intake.getPower());

//        telemetry.addData("TurretTargetPos", turret.getTargetPosition());
        telemetry.addData("TurretRealPos", turret.getCurrentPosition());
        telemetry.addData("Turrettarget", turretTarget);

//        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Shooter Vel", shooter.getVelocity());
        telemetry.addData("Shooter Target Vel", shooter.getTarget());

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

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
        l.setPollRateHz(20);
        l.start();
    }

    public double angleFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null)
            return target.getTargetXDegrees();

        return 0;
    }

    public double angleFromBlue() {
        return angleFromTag(20);
    }

    public double angleFromRed() {
        return angleFromTag(24);
    }

    public void resetTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}