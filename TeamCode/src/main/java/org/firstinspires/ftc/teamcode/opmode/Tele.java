package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.blDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.brDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.dt.bl;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.dt.br;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.dt.fl;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.dt.fr;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.flDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.frDir;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Tuning.telemetryM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.cursedKicker;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp
@Config
public class Tele extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Robot r;
//    Kickers kickers;

    public boolean shoot = true, manual = false, field = true, hold = false, autoFlipping = false, manualFlip = false;
    public double intakeOn = 0, dist;
    public static double shootTarget = 1200;
    private double looptime = 0;
    private final Timer upTimer = new Timer(), autoFlipTimer = new Timer();

    private double lKickerTarget = LKICKER_DOWN;
    private double mKickerTarget = MKICKER_DOWN;
    private double rKickerTarget = RKICKER_DOWN;

    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

    Servo lKicker, mKicker, rKicker;


    DcMotor fl, bl, fr, br;

    @Override
    public void init() {
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


        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);

        r = new Robot(hardwareMap, Alliance.BLUE);
//        kickers = new Kickers(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        r.s.down();
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            r.a = Alliance.BLUE;
        }

        if (gamepad1.bWasPressed()) {
            r.a = Alliance.RED;
        }

        if (gamepad1.xWasPressed())
            r.t.resetTurret();

        if (gamepad1.dpad_up) {
            Robot.endPose = null;
        }

        telemetryM.addData("Turret", r.t.getTurret());
        if (Robot.endPose != null) telemetryM.addData("endPose", Robot.endPose.toString());
        else telemetryM.addLine("endPose is null");
        telemetryM.addData("Alliance", r.a);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        r.setShootTarget();

        if (Robot.endPose == null) {
//            r.f.setStartingPose(r.a.equals(Alliance.BLUE) ? Robot.defaultPose.mirror() : Robot.defaultPose);
            r.f.setStartingPose(Robot.defaultPose);
        } else {
            r.f.setStartingPose(Robot.endPose);
        }

//        r.periodic();
        r.f.startTeleopDrive();

        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);

        upTimer.resetTimer();
    }

    @Override
    public void loop() {
        r.periodic();

        if (gamepad1.x) lKickerTarget = LKICKER_UP; else lKickerTarget = LKICKER_DOWN;
        if (gamepad1.y) mKickerTarget = MKICKER_UP; else mKickerTarget = MKICKER_DOWN;
        if (gamepad1.b) rKickerTarget = RKICKER_UP; else rKickerTarget = RKICKER_DOWN;

        lKicker.setPosition(lKickerTarget);
        mKicker.setPosition(mKickerTarget);
        rKicker.setPosition(rKickerTarget);


//        if(gamepad1.dpad_left)
//            field = true;
//        if(gamepad1.dpad_right)
//            field = false;
//
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x * 1.1;
//        double rx = gamepad1.right_stick_x;


//        if(field){
//            double botHeading = r.f.getPose().getHeading();
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//
//            frontLeftPower = (rotY + rotX + rx) / denominator;
//            backLeftPower = (rotY - rotX + rx) / denominator;
//            frontRightPower = (rotY - rotX - rx) / denominator;
//            backRightPower = (rotY + rotX - rx) / denominator;
//        }
//        else {
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            frontLeftPower = (y + x + rx) / denominator;
//            backLeftPower = (y - x + rx) / denominator;
//            frontRightPower = (y - x - rx) / denominator;
//            backRightPower = (y + x - rx) / denominator;
//        }
//
//        fl.setPower(frontLeftPower);
//        bl.setPower(backLeftPower);
//        fr.setPower(frontRightPower);
//        br.setPower(backRightPower);


        if (!hold)
            if (field)
                r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, true);

        if (upTimer.getElapsedTimeSeconds() > 1 && r.s.atUp() && manualFlip)
            gamepad1.rumbleBlips(1);

//        if (gamepad1.xWasPressed())
//            kickers.kick(Kickers.Kicker.L);
//
//        if (gamepad1.yWasPressed())
//            kickers.kick(Kickers.Kicker.M);
//
//        if (gamepad1.bWasPressed())
//            kickers.kick(Kickers.Kicker.R);

        if (gamepad1.rightBumperWasPressed())
            if (intakeOn == 1)
                intakeOn = 0;
            else
                intakeOn = 1;

        if (gamepad1.leftBumperWasPressed())
            if (intakeOn == 2)
                intakeOn = 0;
            else
                intakeOn = 2;

        if (intakeOn == 1)
            r.i.spinIn();
        else if (intakeOn == 2)
            r.i.spinOut();
        else
            r.i.spinIdle();

        if (shoot) {
            r.s.on();
            r.t.on();

            if (manual) {
                r.t.manual(-gamepad1.right_trigger + gamepad1.left_trigger);
                r.s.setTarget(shootTarget);
            } else {
                dist = r.getShootTarget().distanceFrom(r.f.getPose());
                r.s.forDistance(dist);
                r.t.face(r.getShootTarget(), r.f.getPose());
                r.t.automatic();
            }
        } else {
            r.s.off();
            r.t.off();
        }

//        if (gamepad1.aWasPressed())
//            if (manualFlip) {
//                upTimer.resetTimer();
//                r.s.flip();
//                autoFlipping = false;
//            } else {
//                autoFlipping = true;
//                autoFlipTimer.resetTimer();
//            }

        if (!manualFlip && autoFlipping) {
            if (autoFlipTimer.getElapsedTimeSeconds() > 1.75) {
                r.s.down();
                autoFlipping = false;
            } else if (autoFlipTimer.getElapsedTimeSeconds() > 1.5)
                r.s.up();
            else if (autoFlipTimer.getElapsedTimeSeconds() > 1.25)
                r.s.down();
            else if (autoFlipTimer.getElapsedTimeSeconds() > 1)
                r.s.up();
            else if (autoFlipTimer.getElapsedTimeSeconds() > .75)
                r.s.down();
            else if (autoFlipTimer.getElapsedTimeSeconds() > .5)
                r.s.up();
            else if (autoFlipTimer.getElapsedTimeSeconds() > 0.25)
                r.s.down();
            else if (autoFlipTimer.getElapsedTimeSeconds() > 0)
                r.s.up();
        }

        if (gamepad1.leftStickButtonWasPressed())
            manualFlip = !manualFlip;

        if (gamepad1.touchpadWasPressed())
            shoot = !shoot;

//        if (gamepad1.dpadUpWasPressed()) {
//            if (r.a.equals(Alliance.BLUE)) {
//                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
//            } else {
//                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
//            }
//        }

        if (gamepad1.dpadLeftWasPressed())
            manual = !manual;
//
        if (gamepad1.dpadRightWasPressed())
            field = !field;

        if (gamepad1.dpadDownWasPressed()) {
            hold = !hold;

            if (hold) {
                r.f.holdPoint(new BezierPoint(r.f.getPose()), r.f.getHeading(), false);
            } else {
                r.f.startTeleopDrive();
            }
        }

        if (gamepad1.rightStickButtonWasPressed())
            r.t.resetTurret();


//        TelemetryPacket packet = new TelemetryPacket();
//        packet.addLine("Follower Pose: " + r.f.getPose().toString());
//        packet.addLine("Shooter Velocity: " + r.s.getVelocity());
//        packet.addLine("Shooter Target: " + r.s.getTarget());
//        packet.addLine("Shooter Distance: " + dist);
//        packet.addLine("Turret Yaw: " + r.t.getYaw());
//        packet.addLine("Turret Target: " + r.t.getTurretTarget());
//        packet.addLine("Turret Ticks: " + r.t.getTurret());
//        packet.addLine("Shooter On: " + shoot);
//        packet.addLine("Flipped Up: " + r.s.atUp());
//        packet.addLine("Distance from Target: " + dist);
//        packet.addLine("Manual Shooter + Turret: " + manual);
//        packet.addLine("Field Centric: " + field);
//        packet.addLine("Hold Position: " + hold);
//        packet.addLine("hz: " + 1000000000 / (loop - looptime));
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);

//        telemetryM.addData("Follower Pose", r.f.getPose().toString());
//        telemetryM.addData("Shooter Velocity", r.s.getVelocity());
//        telemetryM.addData("Shooter Target", r.s.getTarget());
//        telemetryM.addData("Shooter Distance", dist);
//        telemetryM.addData("Turret Yaw", r.t.getYaw());
//        telemetryM.addData("Turret Target", r.t.getTurretTarget());
//        telemetryM.addData("Turret Ticks", r.t.getTurret());
//        telemetryM.addData("Shooter On", shoot);
//        telemetryM.addData("Flipped Up", r.s.atUp());
//        telemetryM.addData("Distance from Target", dist);
//        telemetryM.addData("Manual Shooter + Turret", manual);
//        telemetryM.addData("Field Centric", field);
//        telemetryM.addData("Hold Position", hold);
//        telemetryM.addData("hz ", 1000000000 / (loop - looptime));
//
//        telemetryM.debug("Pos");
//        telemetryM.update(telemetry);

//        kickers.periodic();

        telemetry.addData("Follower Pose", r.f.getPose().toString());
        telemetry.addData("Shooter Velocity", r.s.getVelocity());
        telemetry.addData("Shooter Target", r.s.getTarget());
        telemetry.addData("Shooter Pos Target", r.getShootTarget().toString());
//        telemetry.addData("Shooter Distance", dist);
        telemetry.addData("Turret Yaw", r.t.getYaw());
        telemetry.addData("Turret Target", r.t.getTurretTarget());
        telemetry.addData("Turret Ticks", r.t.getTurret());
        telemetry.addData("Shooter On", shoot);
        telemetry.addData("Flipped Up", r.s.atUp());
//        telemetry.addData("Distance from Target", dist);
        telemetry.addData("Manual Shooter + Turret", manual);
        telemetry.addData("Field Centric", field);
        telemetry.addData("Hold Position", hold);
    double loop = System.nanoTime();
    telemetry.addData("hz ", 1000000000 / (loop - looptime));
    looptime = loop;
//        telemetry.addData("Kicker Up", kickers.getUp());
//        telemetry.addData("Kicker Queued", kickers.getQueued());
//        telemetry.addData("Kicker Up", kickers.);

//        telemetry.debug("Pos");
        telemetry.update();
    }


    @Override
    public void stop() {
        r.stop();
    }

}