package org.firstinspires.ftc.teamcode.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.HOOD_CLOSE;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.HOOD_FAR;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.OFFSET;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.TURRET_MIDDLE;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.TURRET_THRESHOLD;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.VELOCITY_CLOSE;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.VELOCITY_FAR;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.blDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.brDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.dt;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.flDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.frDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.intakeDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.lShooterDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.rShooterDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tdl;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tds;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.til;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tis;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tpl;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tps;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.turretDir;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp
public class PIDTelefieldcentric extends OpMode {

    private double looptime = 0, targetVelocity = 0;
    private boolean far = false;
    private double hoodTarget = HOOD_CLOSE;
    private double lKickerTarget = LKICKER_DOWN;
    private double mKickerTarget = MKICKER_DOWN;
    private double rKickerTarget = RKICKER_DOWN;
    private double intakePower = INTAKE_OFF;

    private double turretTarget = TURRET_MIDDLE;
//    private double shooterPower = SHOOTER_OFF;

    private Robot r;

    private Limelight3A l;
    private Alliance a = RED;
    private Follower follower;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;

    private IMU imu;

    private boolean field = false;

    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;

    DcMotor fl, bl, fr, br, intake;
    DcMotorEx lShooter, rShooter, turret;
    Servo lKicker, mKicker, rKicker, hood;

    private PIDController turretPIDLarge, turretPIDSmall;

    public void init() {

        r = new Robot(hardwareMap, Alliance.BLUE);

        fl = hardwareMap.dcMotor.get(dt.fl);
        bl = hardwareMap.dcMotor.get(dt.bl);
        fr = hardwareMap.dcMotor.get(dt.fr);
        br = hardwareMap.dcMotor.get(dt.br);
        intake = hardwareMap.dcMotor.get("intake");
        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        l = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);

//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);
        hood = hardwareMap.servo.get("hood");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        turret.setTargetPosition(TURRET_MIDDLE);

        rShooter.setMode(RUN_USING_ENCODER);
        lShooter.setMode(RUN_WITHOUT_ENCODER);
//        turret.setMode(RUN_TO_POSITION);

        fl.setDirection(flDir);
        bl.setDirection(blDir);
        fr.setDirection(frDir);
        br.setDirection(brDir);
        turret.setDirection(turretDir);
        intake.setDirection(intakeDir);
        lShooter.setDirection(lShooterDir);
        rShooter.setDirection(rShooterDir);

        turretPIDLarge = new PIDController(tpl, til, tdl);
        turretPIDSmall = new PIDController(tps, tis, tds);

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

        if (gamepad1.x)
            resetTurret();


        if(gamepad1.dpad_left) {
            a = BLUE;
            r.a = Alliance.BLUE;
        }
        if(gamepad1.dpad_right) {
            a = RED;
            r.a = RED;
        }

        follower.setStartingPose(Robot.endPose);

        telemetry.addData("TurretTargetPos", turret.getTargetPosition());
        telemetry.addData("TurretRealPos", turret.getCurrentPosition());
        telemetry.addData("Turrettarget", turretTarget);
        telemetry.addData("Alliance", a);
        telemetry.update();

    }

    @Override
    public void loop() {

//        if (gamepad1.options) {
//            imu.resetYaw();
//        }

        if(gamepad1.dpad_left)
            field = true;
        if(gamepad1.dpad_right)
            field = false;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;


        if(field){
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = follower.getPose().getHeading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;
        }
        else {

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
        }

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

        if(a==RED) {
            turretTarget = angleFromRed() * 24746.6674795 / 180 - OFFSET;
        }else if(a==BLUE) {
            turretTarget = angleFromBlue() * 24746.6674795 / 180 - OFFSET;
        }else
            turretTarget = TURRET_MIDDLE;


        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);
        hood.setPosition(hoodTarget);
        intake.setPower(intakePower);

//        turret.setPower(1);
//        turret.setTargetPosition((int) turretTarget);
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
//        telemetry.addData("tPower", tpower);
        turret.setPower(tpower);



        rShooter.setVelocity(targetVelocity, AngleUnit.DEGREES);
        lShooter.setPower(rShooter.getPower());

//        telemetry.addData("Left Kicker ", lKicker.getPosition());
//        telemetry.addData("Middle Kicker", mKicker.getPosition());
//        telemetry.addData("Right Kicker", rKicker.getPosition());
//        telemetry.addData("Hood", hood.getPosition());
//        telemetry.addData("Intake", intake.getPower());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

//        telemetry.addData("TurretTargetPos", turret.getTargetPosition());
        telemetry.addData("TurretRealPos", turret.getCurrentPosition());
        telemetry.addData("TurretTarget", turretTarget);

//        telemetry.addData("Left Shooter", lShooter.getPower());
//        telemetry.addData("Right Shooter", rShooter.getPower());
//        telemetry.addData("Left Shooter Vel", lShooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Right Shooter Vel", rShooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Target Vel", targetVelocity);

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