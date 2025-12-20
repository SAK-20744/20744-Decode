package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
//import org.firstinspires.ftc.teamcode.subsystems.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors;
import org.firstinspires.ftc.teamcode.subsystems.cursedKicker;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Pattern;

import java.util.List;

@Autonomous
public class redFarSix extends OpMode {
    Shooter shooter;
    Turret turret;
    //    Kickers kickers;
    cursedKicker kickers;
    DcMotor intake;
    Follower drive;
    Limelight3A l;

    BallSensors ballSensors;
    private Alliance a = RED;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;


    private double looptime = 0;

    ElapsedTime kTimer = new ElapsedTime();

    private Pattern p = Pattern.NONE;

    PathChain toBall1Start, toBall1End, toLaunch1, toBall2Start, toBall2End, toLaunch2, toPark;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
//        kickers = new Kickers(hardwareMap);

        kickers = new cursedKicker(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(ApolloConstants.intakeDir);

        ballSensors = new BallSensors(hardwareMap);


        l = hardwareMap.get(Limelight3A.class, "limelight");


        drive = Constants.createFollower(hardwareMap);
        drive.setPose(FieldPoses.redFarStart);

        toBall1Start = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redFarStart, FieldPoses.redBall1Start))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toBall1End = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redBall1Start, FieldPoses.redBall1End))
                .setVelocityConstraint(intakeMovementSpeed)
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toLaunch1 = drive.pathBuilder()
                .addPath(new BezierCurve(FieldPoses.redBall1End, FieldPoses.redBall1Start, FieldPoses.shooting))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toBall2Start = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.shooting, FieldPoses.redBall2Start))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toBall2End = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.redBall2Start, FieldPoses.redBall2End))
                .setVelocityConstraint(intakeMovementSpeed)
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toLaunch2 = drive.pathBuilder()
                .addPath(new BezierCurve(FieldPoses.redBall2End, FieldPoses.redBall2Start, FieldPoses.shooting))
                .setConstantHeadingInterpolation(FieldPoses.redFarStart.getHeading())
                .build();
        toPark = drive.pathBuilder()
                .addPath(new BezierLine(FieldPoses.shooting, FieldPoses.park))
                .setLinearHeadingInterpolation(FieldPoses.redFarStart.getHeading(), FieldPoses.park.getHeading())
                .build();
//        turret.off();
    }
    @Override
    public void init_loop() {
        if (gamepad1.x)
            turret.resetTurret();

        if(detectedID() == 21)
            p = Pattern.GPP21;
        else if(detectedID() == 22)
            p = Pattern.PGP22;
        else if(detectedID() == 23)
            p = Pattern.PPG23;
        ballSensors.motif(p);

        telemetry.addData("Detected ID: ", p);
        telemetry.addData("Turret Angle:", turret.getTurret());
        telemetry.update();



    }

    @Override
    public void start() {
        shooter.up();
        intake.setPower(1);
        shooter.far();
//        turret.face(FieldPoses.redHoop,drive.getPose());
        turret.setYaw(Math.toRadians(autoTurret));
//        Shoot();
//        while(shooter.isActivated()) { update(); }
        kTimer.reset();
        cursedShoot();

//        shooter.far();
//        while(!shooter.atTarget()) { update(); }
//        lKick();
//        while(!shooter.atTarget()) { update(); }
//        rKick();
//        while(!shooter.atTarget()) { update(); }
//        mKick();
//        while(!shooter.atTarget()) { update(); }

        drive.followPath(toBall1Start); // Put drive to human player path in here
        while(drive.isBusy()) { update(); }
//        intake.setPower(1);
        drive.followPath(toBall1End);
        while(drive.isBusy()) { update(); }
//        intake.setPower(0.4);

        drive.followPath(toLaunch1);
        while(drive.isBusy()) { update(); }
        turret.setYaw(Math.toRadians(autoTurret2));
        cursedShootSensor();
        drive.followPath(toBall2Start); // Put drive to human player path in here
        while(drive.isBusy()) { update(); }
//        intake.setPower(1);
        drive.followPath(toBall2End);
        while(drive.isBusy()) { update(); }
//        intake.setPower(0.4);
        drive.followPath(toLaunch2);
        while(drive.isBusy()) { update(); }
        turret.setYaw(Math.toRadians(autoTurret3));
        cursedShootSensor();
        drive.followPath(toPark);
        turret.setYaw(Math.toRadians(0));
        while(drive.isBusy()) { update(); }
        intake.setPower(0);
        shooter.off();
        while(shooter.isActivated()) { update(); }

//        drive.followPath(toLaunch); // Drive back to redFarStart to be back in far launch zone
//        while (drive.isBusy()) {
//            update();
//        }
//        Shoot();
    }
    @Override
    public void loop() {

    }
//    public void Shoot() {
//        //Shoot stuff here
//        shooter.far();
//        while (!shooter.atTarget()) a();
//        kickers.kick(Kickers.Kicker.L);
//        while (!kickers.kickerDown()) b();
//        kickers.kick(Kickers.Kicker.M);
//        while (!kickers.kickerDown()) b();
//        kickers.kick(Kickers.Kicker.R);
//        while (!kickers.kickerDown()) b();
//        shooter.setTarget(0);
//        shooter.setPower(0);
//
//    }

    public void cursedShoot(){

        if(p == Pattern.GPP21){
            shoot21GPP();
            return;
        }
        if(p == Pattern.PGP22){
            shoot22PGP();
            return;
        }
        if(p == Pattern.PPG23){
            shoot23PPG();
            return;
        }
        return;
    }
    public void cursedShootSensor(){
        ballSensors.periodic();
        String[] sequence = ballSensors.shootSequence();

        shooter.far();
        while(!shooter.atTarget()) { update(); }
        shootStr(sequence[0]);
        while(!shooter.atTarget()) { update(); }
        shootStr(sequence[1]);
        while(!shooter.atTarget()) { update(); }
        shootStr(sequence[2]);
        while(!shooter.atTarget()) { update(); }
        if (sequence[0] == "m") mKick();
    }
    public void shootStr(String a) {
        switch (a.toLowerCase()) {
            case "l": lKick();break;
            case "m": mKick();break;
            case "r": rKick();break;
        }
    }
    public void shoot21GPP() {
        shooter.far();
        while(!shooter.atTarget()) { update(); }
        mKick();
        while(!shooter.atTarget()) { update(); }
        lKick();
        while(!shooter.atTarget()) { update(); }
        rKick();
        while(!shooter.atTarget()) { update(); }
        mKick();
        while(!shooter.atTarget()) { update(); }
    }

    public void shoot22PGP() {
        shooter.far();
        while(!shooter.atTarget()) { update(); }
        lKick();
        while(!shooter.atTarget()) { update(); }
        mKick();
        while(!shooter.atTarget()) { update(); }
        rKick();
        while(!shooter.atTarget()) { update(); }
    }

    public void shoot23PPG() {
        shooter.far();
        while(!shooter.atTarget()) { update(); }
        lKick();
        while(!shooter.atTarget()) { update(); }
        rKick();
        while(!shooter.atTarget()) { update(); }
        mKick();
        while(!shooter.atTarget()) { update(); }
    }


    public void lKick (){
        kTimer.reset();
        kickers.lKickerUp(); kTimer.reset();
        while(kTimer.milliseconds()<KUP) {update();}
        kickers.lKickerDown();  kTimer.reset();
        while(kTimer.milliseconds()<KDOWN) {update();}
    }

    public void mKick (){
        kTimer.reset();
        kickers.mKickerUp(); kTimer.reset();
        while(kTimer.milliseconds()<KUP) {update();}
        kickers.mKickerDown(); kTimer.reset();
        while(kTimer.milliseconds()<KDOWN) {update();}
    }

    public void rKick (){
        kTimer.reset();
        kickers.rKickerUp(); kTimer.reset();
        while(kTimer.milliseconds()<KUP) {update();}
        kickers.rKickerDown(); kTimer.reset();
        while(kTimer.milliseconds()<KDOWN) {update();}
    }




    public void a() {telemetry.addLine("Getting Up to Speed\n");update();}
    public void b() {telemetry.addLine("Kicking\n");update();}
    public void update() {
        drive.update();
        shooter.periodic();
        turret.periodic();
        kickers.periodic();
//        ballSensors.periodic();

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



    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
        l.setPollRateHz(20);
        l.start();
    }

    public double detectedID() {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() == 21)
                return 21;
            else if (i != null && i.getFiducialId() == 22)
                return 22;
            else if (i != null && i.getFiducialId() == 23)
                return 23;
        }

        return 0;
    }



    private void sleep(int ms) {ElapsedTime timer = new ElapsedTime(); while (timer.milliseconds() < ms);}
}