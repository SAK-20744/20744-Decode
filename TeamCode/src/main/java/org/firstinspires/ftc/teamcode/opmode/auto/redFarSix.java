package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.follower.Follower;
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

    PathChain toHumanPlayer, toLaunch;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        kickers = new Kickers(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(ApolloConstants.intakeDir);

        drive = Constants.createFollower(hardwareMap);
        drive.setPose(FieldPoses.redFarStart);

        toHumanPlayer = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redFarStart, FieldPoses.redHPPickup))
                .build();
        toLaunch = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redHPPickup, FieldPoses.redFarStart))
                .build();
    }
    @Override
    public void start() {
        turret.face(drive.getPose(),FieldPoses.redHoop);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 2)
            update();
        Shoot();
        drive.followPath(toHumanPlayer); // Put drive to human player path in here
        while(drive.isBusy()) {
            update();
            if (drive.getPathCompletion() > 0.75) {
                intake.setPower(1);
            }
            telem();
        }
        drive.followPath(toLaunch); // Drive back to redFarStart to be back in far launch zone
        while (drive.isBusy()) {
            update();
            telem();
        }
        Shoot();
    }
    @Override
    public void loop() {

    }
    public void Shoot() {
        //Shoot stuff here
        shooter.far();
        while (!shooter.atTarget()) update();
        kickers.kick(Kickers.Kicker.L);
        while (!kickers.kickerDown()) update();
        kickers.kick(Kickers.Kicker.M);
        while (!kickers.kickerDown()) update();
        kickers.kick(Kickers.Kicker.R);
        shooter.setTarget(0);
        shooter.setPower(0);

    }
    public void update() {
        drive.update();
        shooter.periodic();
        turret.periodic();
        kickers.periodic();
    }
    public void telem() {
        telemetry.addData("Bot Pose",drive.getPose());
        telemetry.addData("Path Completion",drive.getPathCompletion());
        telemetry.update();
    }
    private void sleep(int ms) {ElapsedTime timer = new ElapsedTime(); while (timer.milliseconds() < ms);}
}
