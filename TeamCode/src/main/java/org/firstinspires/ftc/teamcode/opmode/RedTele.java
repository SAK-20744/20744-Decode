package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.blDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.brDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.flDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.frDir;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp
@Config
public class RedTele extends OpMode {
    TelemetryManager telemetryM;
    MultipleTelemetry multipleTelemetry;

    Robot r;
//    Kickers kickers;

    private boolean far = false;


    public boolean shoot = true, manual = false, field = true, hold = false, autoFlipping = false, manualFlip = false;
    public double intakeOn = 0, dist;
    public static double shootTarget = 1200;
    private double looptime = 0;
    private final Timer upTimer = new Timer(), autoFlipTimer = new Timer();

    private double lKickerTarget = LKICKER_DOWN;
    private double mKickerTarget = MKICKER_DOWN;
    private double rKickerTarget = RKICKER_DOWN;

    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;

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

        r = new Robot(hardwareMap, Alliance.RED);
//        kickers = new Kickers(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        r.s.down();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            r.t.resetTurret();

        telemetryM.addData("Turret", r.t.getTurret());

        if (Robot.endPose == null)
            Robot.endPose = new Pose(0,0,0);

        telemetryM.addData("endPose", Robot.endPose.toString());
        telemetryM.addData("Alliance", r.a);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        r.setShootTarget();

        if (Robot.endPose == null) {
            r.f.setStartingPose(r.a.equals(Alliance.BLUE) ? Robot.defaultPose.mirror() : Robot.defaultPose);
        } else {
            r.f.setStartingPose(Robot.endPose);
        }

        r.periodic();
        r.t.reset();
        r.t.on();
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

        if (!hold)
            if (field)
                r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);
            else
                r.f.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, shoot ? -gamepad1.right_stick_x * 0.5 : -gamepad1.right_stick_x * 0.75, true);

        if (gamepad1.right_bumper)
            r.i.spinIn();
        else if (gamepad1.left_bumper)
            r.i.spinOut();
        else
            r.i.spinIdle();

        r.t.face(FieldPoses.redHoop, r.f.getPose());


//        if (shoot) {
//            r.s.on();
//            r.t.on();
//            dist = r.getShootTarget().distanceFrom(r.f.getPose());
//            r.s.forDistance(dist);
//            r.t.face(r.getShootTarget(), r.f.getPose());
//            r.t.automatic();
//        } else {
//            r.s.off();
//            r.t.off();
//        }
//
//        if (gamepad1.touchpadWasPressed())
//            shoot = !shoot;

        if(far){
            r.s.up();
            r.s.far();
        } else {
            r.s.down();
            r.s.close();
            }

        if(gamepad1.dpad_up)
            far = false;
        if(gamepad1.dpad_down)
            far = true;

        if (gamepad1.dpadRightWasPressed())
            field = !field;

        if (gamepad1.dpadLeftWasPressed()) {
            hold = !hold;

            if (hold) {
                r.f.holdPoint(new BezierPoint(r.f.getPose()), r.f.getHeading(), false);
            } else {
                r.f.startTeleopDrive();
            }
        }

        if (gamepad1.rightStickButtonWasPressed())
            r.t.resetTurret();

        telemetry.addData("Follower Pose", r.f.getPose().toString());
        telemetry.addData("Shooter Velocity", r.s.getVelocity());
        telemetry.addData("Shooter Target", r.s.getTarget());
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
        telemetry.update();
    }


    @Override
    public void stop() {
        r.stop();
    }

}