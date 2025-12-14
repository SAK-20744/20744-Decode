package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.autoTurret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.subsystems.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;

@Autonomous
public class redFarSix extends OpMode {
    Shooter shooter;
    Turret turret;
    Kickers kickers;
    DcMotor intake;
    Follower drive;

    private double looptime = 0;


    PathChain toBall1Start, toBall1End, toLaunch1, toBall2Start, toBall2End, toLaunch2;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        kickers = new Kickers(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(ApolloConstants.intakeDir);

        drive = Constants.createFollower(hardwareMap);
        drive.setPose(FieldPoses.redFarStart);

        toBall1Start = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redFarStart, FieldPoses.redBall1Start))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toBall1End = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redBall1Start, FieldPoses.redBall1End))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toLaunch1 = drive.pathBuilder()
                .addPath(new BezierCurve(FieldPoses.redBall1End, FieldPoses.redBall1Start, FieldPoses.redFarStart))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toBall2Start = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redFarStart, FieldPoses.redBall2Start))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toBall2End = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redBall2Start, FieldPoses.redBall2End))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toLaunch2 = drive.pathBuilder()
                .addPath(new BezierCurve(FieldPoses.redBall2End, FieldPoses.redBall2Start, FieldPoses.redFarStart))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
//        turret.off();
    }
    @Override
    public void start() {
//        turret.face(FieldPoses.redHoop,drive.getPose());
        turret.setYaw(Math.toRadians(autoTurret));
        Shoot();
        while(drive.isBusy()) { update(); }
        drive.followPath(toBall1Start); // Put drive to human player path in here
        while(drive.isBusy()) { update(); }
        intake.setPower(1);
        drive.followPath(toBall1End);
        while(drive.isBusy()) { update(); }
        intake.setPower(0);
        drive.followPath(toLaunch1);
        while(drive.isBusy()) { update(); }
        Shoot();
        while(drive.isBusy()) { update(); }
        drive.followPath(toBall2Start); // Put drive to human player path in here
        while(drive.isBusy()) { update(); }
        intake.setPower(1);
        drive.followPath(toBall2End);
        while(drive.isBusy()) { update(); }
        intake.setPower(0);
        drive.followPath(toLaunch2);
        while(drive.isBusy()) { update(); }
        Shoot();
        while(drive.isBusy()) { update(); }

//        drive.followPath(toLaunch); // Drive back to redFarStart to be back in far launch zone
//        while (drive.isBusy()) {
//            update();
//        }
//        Shoot();
    }
    @Override
    public void loop() {

    }
    public void Shoot() {
        //Shoot stuff here
        shooter.far();
        while (!shooter.atTarget()) a();
        kickers.kick(Kickers.Kicker.L);
        while (kickers.kickerDown()) b();
        kickers.kick(Kickers.Kicker.M);
        while (kickers.kickerDown()) b();
        kickers.kick(Kickers.Kicker.R);
        while (kickers.kickerDown()) b();
        shooter.setTarget(0);
        shooter.setPower(0);

    }
    public void a() {telemetry.addLine("Getting Up to Speed\n");update();}
    public void b() {telemetry.addLine("Kicking\n");update();}
    public void update() {
        drive.update();
        shooter.periodic();
        turret.periodic();
        kickers.periodic();

        telem();
    }
    public void telem() {

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - looptime));
        looptime = loop;

        telemetry.addData("Shooter Vel", shooter.getVelocity());
        telemetry.addData("Shooter Target", shooter.getTarget());
        telemetry.addData("Turret Angle", turret.getYaw());
        telemetry.addData("Bot Pose",drive.getPose());
        telemetry.addData("Path Completion",drive.getPathCompletion());
        telemetry.update();
    }
    private void sleep(int ms) {ElapsedTime timer = new ElapsedTime(); while (timer.milliseconds() < ms);}
}
