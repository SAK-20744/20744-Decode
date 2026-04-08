package org.firstinspires.ftc.teamcode.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors2;
import org.firstinspires.ftc.teamcode.subsystems.KickersV2;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;

import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@Autonomous(name="Close Flex Blue")
public class Blue_Flex extends NextFTCOpMode {
    public Blue_Flex() {}

    Robot Apollo;
    Follower follower;
    SequentialGroupFixed routines;
    SequentialGroup preloads, close, middle, far, gate, end;
    PathChain scorePreloads;
    BallSensors2 ballSensor;
    Limelight limelight;
    ArrayList<SequentialGroup> commands;
    boolean doingMiddles = false;
    Pose currentPose;
    public TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    boolean isUpdating;

    boolean isA = false;
    boolean isB = false;
    boolean isX = false;
    boolean isY = false;

    Pose scorePose;

    // Config
    public static double gateX = 14.5;
    public static double gateY = 58.75;
    public static double gateHeading = 150;
    public static double shootVel = 1240;
    public static double turretPos = 60;
    public static double distCheck = 2;

    @Override
    public void onInit() {
        // Initialize hardware and software
        follower = Constants.createFollower(hardwareMap);
        Apollo = new Robot(hardwareMap, Alliance.BLUE);
        ballSensor = new BallSensors2(hardwareMap);
        limelight = new Limelight(hardwareMap);

        commands = new ArrayList<>();

        // Create Poses
        Pose starting = new Pose(60, 84, Math.toRadians(180));
        scorePose = new Pose(52, 80, Math.toRadians(180));
        Pose intakeCloseStart = new Pose(44, 84, Math.toRadians(180));
        Pose intakeCloseEnd = new Pose(18, 84, Math.toRadians(180));
        Pose intakeMiddleStart = new Pose(44, 60, Math.toRadians(180));
        Pose intakeMiddleEnd = new Pose(12, 60, Math.toRadians(180));
        Pose intakeFarStart = new Pose(44, 36, Math.toRadians(180));
        Pose intakeFarEnd = new Pose(12, 36, Math.toRadians(180));
        Pose intakeGateEnd = new Pose(gateX, gateY, gateHeading);
        Pose endPose = new Pose(36, 72, Math.toRadians(180));
        
        // Create Paths
        
        PathChain intakeClose = follower.pathBuilder()
                .addPath(new BezierLine(intakeCloseStart, intakeCloseEnd))
                .build();
        
        PathChain intakeMiddle = follower.pathBuilder()
                .addPath(new BezierLine(intakeMiddleStart, intakeMiddleEnd))
                .build();

        PathChain intakeFar = follower.pathBuilder()
                .addPath(new BezierLine(intakeFarStart, intakeFarEnd))
                .build();
        
        PathChain intakeGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeGateEnd))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeGateEnd.getHeading())
                .build();

        
        PathChain scoreClose = follower.pathBuilder()
                .addPath(new BezierLine(intakeCloseEnd, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        
        PathChain scoreMiddle = follower.pathBuilder()
                .addPath(new BezierLine(intakeMiddleEnd, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        
        PathChain scoreFar = follower.pathBuilder()
                .addPath(new BezierLine(intakeFarEnd, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        PathChain scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(intakeGateEnd, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();


        close = new SequentialGroup(
                new InstantCommand(() -> {
                    Apollo.i.spinIn();
                    follower.followPath(intakeClose);
                }),
                new WaitUntil(() -> ballSensor.isFull()),
                new InstantCommand(() -> {
                    Apollo.i.spinIdle();
                    follower.followPath(scoreClose);
                }),
                new WaitUntil(this::isDonePathing),
                new InstantCommand(this::Shoot),
                new WaitUntil(this::shootingDone)
        );

        middle = new SequentialGroup(
                new InstantCommand(() -> {
                    Apollo.i.spinIn();
                    follower.followPath(intakeMiddle);
                }),
                new WaitUntil(() -> ballSensor.isFull()),
                new InstantCommand(() -> {
                    Apollo.i.spinIdle();
                    follower.followPath(scoreMiddle);
                }),
                new WaitUntil(this::isDonePathing),
                new InstantCommand(this::Shoot),
                new WaitUntil(this::shootingDone)
        );

        far = new SequentialGroup(
                new InstantCommand(() -> {
                    Apollo.i.spinIn();
                    follower.followPath(intakeFar);
                }),
                new WaitUntil(() -> ballSensor.isFull()),
                new InstantCommand(() -> {
                    Apollo.i.spinIdle();
                    follower.followPath(scoreFar);
                }),
                new WaitUntil(this::isDonePathing),
                new InstantCommand(this::Shoot),
                new WaitUntil(this::shootingDone)
        );

        gate = new SequentialGroup(
                new InstantCommand(() -> {
                    Apollo.i.spinIn();
                    follower.followPath(intakeGate);
                }),
                new WaitUntil(() -> ballSensor.isFull()),
                new InstantCommand(() -> {
                    Apollo.i.spinIdle();
                    follower.followPath(scoreGate);
                }),
                new WaitUntil(this::isDonePathing),
                new InstantCommand(this::Shoot),
                new WaitUntil(this::shootingDone)
        );

        close.setName("close");
        middle.setName("middle");
        far.setName("far");
        gate.setName("gate");

        follower.setStartingPose(starting);

        Apollo.t.face(new Pose(0, 144), scorePose);
        Apollo.t.on();

        Apollo.k.init();
    }

    @Override
    public void onWaitForStart() {
        currentPose = follower.getPose();
        telemetryManager.addData("x", currentPose.getX());
        telemetryManager.addData("y", currentPose.getY());
        telemetryManager.addData("heading", Math.toDegrees(currentPose.getHeading()));
        for (int i = 0; i < commands.size(); i++) {
            telemetryManager.addData("path " + (i + 1), commands.get(i).name());
        }
        telemetryManager.update(telemetry);
        follower.updatePose();

        Apollo.t.periodic();

        if (gamepad1.left_bumper) {
            scorePreloads = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), scorePose))
                    .setLinearHeadingInterpolation(follower.getHeading(), scorePose.getHeading())
                    .build();
            preloads = new SequentialGroup(
                    new InstantCommand(() -> {
                        Apollo.s.close();
                        follower.followPath(scorePreloads);
                    }),
                    new WaitUntil(this::isDonePathing),
                    new WaitUntil(() -> Apollo.s.atTarget()),
                    new InstantCommand(this::Shoot),
                    new WaitUntil(this::shootingDone)
            );
        }

        if (gamepad1.x && !isX) {
            if (!doingMiddles) {
                commands.add(middle);
                doingMiddles = true;
            }

            commands.add(gate);
            isX = true;
        } else if (!gamepad1.x && isX) {
            isX = false;
        }

        if (gamepad1.y && !isY) {
            commands.add(close);
            isY = true;
        } else if (!gamepad1.y && isY) {
            isY = false;
        }

        if (gamepad1.b && !isB) {
            commands.add(middle);
            doingMiddles = true;
            isB = true;
        } else if (!gamepad1.b && isB) {
            isB = false;
        }

        if (gamepad1.a && !isA) {
            commands.add(far);
            isA = true;
        } else if (!gamepad1.a && isA) {
            isA = false;
        }
    }

    @Override
    public void onStartButtonPressed() {
        routines = new SequentialGroupFixed(preloads);
        commands.add(end);
        for (int i = 0; i < commands.size(); i++) {
            routines.add(commands.get(i));
        }
        routines.schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();

        currentPose = follower.getPose();
        telemetryManager.addData("x", currentPose.getX());
        telemetryManager.addData("y", currentPose.getY());
        telemetryManager.addData("heading", Math.toDegrees(currentPose.getHeading()));
        telemetryManager.addData("velocity", follower.getVelocity().getMagnitude());
        telemetryManager.addData("t", follower.getCurrentTValue());

        if (isUpdating) {
            telemetryManager.update(telemetry);
            isUpdating = false;
        } else isUpdating = true;

        Apollo.t.periodic();
        Apollo.s.periodic();
        Apollo.k.periodic();
        ballSensor.read();
    }

    public void Shoot() {
        Apollo.k.kickSequenced(ballSensor.shootSequence());
    }

    public boolean shootingDone() {
        return !Apollo.k.kickersActive();
    }

    public boolean isDonePathing() {
        return follower.getPose().distanceFrom(follower.getCurrentPathChain().endPoint()) <= distCheck || follower.atParametricEnd();
    }
}
