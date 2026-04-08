package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors2;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.KickersV2;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp ()
public class LLBlue extends LinearOpMode {
    Turret turret;
    Follower drive;
    Tilt tilt;
//    Servo lKicker, mKicker, rKicker;
    Shooter shooter;
    DcMotor fl, bl, fr, br, intake;

    Servo light;
    ElapsedTime lightTimer = new ElapsedTime();

    BallSensors2 bs;

    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;
    private Limelight3A l;

//    private double lKickerTarget = LKICKER_DOWN;
//    private double mKickerTarget = MKICKER_DOWN;
//    private double rKickerTarget = RKICKER_DOWN;
    private double intakePower = INTAKE_OFF;

    boolean targetPressed = false;
    double looptime = 0;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    boolean field = true;

    @Override
    public void runOpMode() {
        drive = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.off();
        shooter = new Shooter(hardwareMap);
        tilt = new Tilt(hardwareMap);
        tilt.retract();

        intake = hardwareMap.dcMotor.get("intake");
        l = hardwareMap.get(Limelight3A.class, "limelight");
        light = hardwareMap.get(Servo.class, "light");

        bs = new BallSensors2(hardwareMap);
        bs.motif(motif);

//        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
//        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
//        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);
        KickersV2 kickers = new KickersV2(hardwareMap);
        kickers.init();
        intake.setDirection(intakeDir);

        fl = hardwareMap.dcMotor.get(ApolloConstants.dt.fl);
        bl = hardwareMap.dcMotor.get(ApolloConstants.dt.bl);
        fr = hardwareMap.dcMotor.get(ApolloConstants.dt.fr);
        br = hardwareMap.dcMotor.get(ApolloConstants.dt.br);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(flDir);
        bl.setDirection(blDir);
        fr.setDirection(frDir);
        br.setDirection(brDir);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        switchToShoot();
        while (opModeInInit()) {
            if (gamepad1.x) turret.resetTurret();
            telemetry.addData("Turret Angle",turret.getYaw());
            telemetry.addData("Get Turret", turret.getTurret());
            telemetry.update();

            intakePower = INTAKE_OFF;
            intake.setPower(intakePower);
        }
        switchToShoot();
        waitForStart();
        lightTimer.reset();

        if (Robot.endPose != null) {
            drive.setStartingPose(Robot.endPose);
        }

        boolean fieldToggle = false;

        shooter.close();
        shooter.down();
        while (opModeIsActive()) {
            turret.on();
            {
                Drawing.drawDebug(drive);
            }

            if(gamepad1.dpad_left)
                drive.setPose(new Pose(FieldPoses.blueReset.getX(), FieldPoses.blueReset.getY(),drive.getHeading()));

            if(gamepad1.dpad_down || gamepad2.dpad_down){
                shooter.up();
                shooter.far();
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                shooter.down();
                shooter.close();
            }

//            if (gamepad1.x) lKickerTarget = LKICKER_UP; else lKickerTarget = LKICKER_DOWN;
//            if (gamepad1.y) mKickerTarget = MKICKER_UP; else mKickerTarget = MKICKER_DOWN;
//            if (gamepad1.b) rKickerTarget = RKICKER_UP; else rKickerTarget = RKICKER_DOWN;
            if (gamepad1.xWasPressed()) kickers.kick(Kicker.L);
            if (gamepad1.yWasPressed()) kickers.kick(Kicker.M);
            if (gamepad1.bWasPressed()) kickers.kick(Kicker.R);
            if (gamepad1.aWasPressed()) {
                bs.read();
                kickers.kickSequenced(bs.shootSequence());
            }
            kickers.slowed = shooter.isFar;
            kickers.periodic();

            if (gamepad2.y) {tilt.extend(); shooter.off(); turret.off();}
            if (gamepad2.a) {tilt.retract(); shooter.on(); turret.on();}

            if(gamepad1.options)
                drive.setPose(new Pose(drive.getPose().getX(), drive.getPose().getY() , -Math.PI/2));

            if (gamepad2.x && !fieldToggle)
                toggleField();
            fieldToggle = gamepad2.x;

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            if(field){
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double botHeading = drive.getPose().getHeading() + Math.PI/2;

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same r  atio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
            }
            else {

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
            }

            if (gamepad1.right_bumper) intakePower = INTAKE_IN;
            else if (gamepad1.left_bumper) intakePower = INTAKE_OUT;
            else intakePower = INTAKE_OFF;

            intake.setPower(intakePower);

            Pose correctedPose = drive.getPose()
                    .plus(new Pose(
                            drive.getVelocity().getXComponent() * ApolloConstants.VELOCITY_CORRECTION,
                            drive.getVelocity().getYComponent() * ApolloConstants.VELOCITY_CORRECTION
                    ));
            Pose goal = FieldPoses.blueHoop;
            if (shooter.isFar) goal = FieldPoses.blueHoopFar;
            turret.face(goal, correctedPose);

//            lKicker.setPosition(lKickerTarget);
//            mKicker.setPosition(mKickerTarget);
//            rKicker.setPosition(rKickerTarget);

            double t = lightTimer.seconds();
            double phase = (t % (2.0 * UP_TIME)) / UP_TIME;
            double tri = (phase <= 1.0) ? phase : (2.0 - phase);
            double pos = REDLIGHT + tri * (PURPLELIGHT - REDLIGHT);

            light.setPosition(pos);

            turret.periodic();
            shooter.periodic();
            drive.updatePose();

            visionRelocalizeLoop(turret.getYaw());

            turret.setOffset(TurretScaling * (gamepad2.left_trigger - gamepad2.right_trigger));
            turret.periodic();

            telemetry.addData("Robot Pose", drive.getPose());
            telemetry.addData("Turret Angle",turret.getYaw());
            telemetry.addData("Get Turret", turret.getTurret());
            telemetry.addData("Turret Target", turret.getTurretTarget());

            telemetry.addData("Shooter Current", shooter.getVelocity());
            telemetry.addData("Shooter  Target", shooter.getTarget());
            telemetry.addData("Distance to Goal", drive.getPose().distanceFrom(FieldPoses.blueHoop));

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - looptime));
            looptime = loop;
            telemetry.update();
        }
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
        l.setPollRateHz(20);
        l.start();
    }

    public void visionRelocalizeLoop(double turretYawRad) {

        // Feed robot heading to Limelight (MegaTag2 needs this)
        l.updateRobotOrientation(drive.getPose().getHeading());

        // Get MegaTag2 result
        LLResult result = l.getLatestResult();
        if (result == null || !result.isValid()) return;

        // Read MegaTag2 field pose (INCHES)
        double camX = result.getBotpose_MT2()
                .getPosition()
                .toUnit(DistanceUnit.INCH).x;

        double camY = result.getBotpose_MT2()
                .getPosition()
                .toUnit(DistanceUnit.INCH).y;

        // Heading (radians) from Pedro
        double headingRad = drive.getPose().getHeading();

        // Turret + camera offset correction (camera pose -> robot-center pose)
        double camYawField = headingRad + turretYawRad;

        double c = Math.cos(camYawField);
        double s = Math.sin(camYawField);

        double offX = CAM_FWD_IN * c - CAM_LEFT_IN * s;
        double offY = CAM_FWD_IN * s + CAM_LEFT_IN * c;

        double robotX = camX - offX;
        double robotY = camY - offY;

        // Tight gate + smooth blend
        Pose cur = drive.getPose();
        double jump = Math.hypot(robotX - cur.getX(), robotY - cur.getY());
        if (jump > VISION_MAX_JUMP_IN) return;

        double newX = cur.getX() + VISION_ALPHA * (robotX - cur.getX());
        double newY = cur.getY() + VISION_ALPHA * (robotY - cur.getY());
        double newH = cur.getHeading() + VISION_ALPHA * wrapRad(headingRad - cur.getHeading());

        drive.setPose(new Pose(newX, newY, newH));
    }



    private static double wrapRad(double a) {
        a %= (Math.PI * 2.0);
        if (a >= Math.PI) a -= Math.PI * 2.0;
        if (a < -Math.PI) a += Math.PI * 2.0;
        return a;
    }


    private void toggleField() {
        if (field)
            field = false;
        else
            field = true;
    }
}
