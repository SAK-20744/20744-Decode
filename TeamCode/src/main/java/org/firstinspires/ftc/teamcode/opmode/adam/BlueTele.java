//package org.firstinspires.ftc.teamcode.opmode.adam;
//
//import android.util.Pair;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTeleop")
//@Config
//public class BlueTele extends OpMode {
//
//    private Drivebase drivebase; private Intake intake; private AdamShooter shooter; private Turret turret;
//
//    GamepadEx controller;
//
//    private boolean isDpadDownPressed = false, isDpadRightPressed = false, isAPressed = false, isDpadUpPressed = false, shootingMode = false, close = true, flapUp = false;
//
//    private GoBildaPinpointDriver pinpoint;
//    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;
//
//    public double xPos, yPos, meters;
//
//    @Config
//    public class Drivebase {
//
//        private DcMotor fL, fR, bL, bR;
//
//        private double xIn, yIn, zIn, normalizer;
//
//        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
//
//        private boolean tele = false;
//
//        public Drivebase(HardwareMap hardwareMap, boolean tele) {
//            fL = hardwareMap.dcMotor.get("fL");
//            bR = hardwareMap.dcMotor.get("bR");
//            bL = hardwareMap.dcMotor.get("bL");
//            fR = hardwareMap.dcMotor.get("fR");
//
//            fL.setZeroPowerBehavior(zeroPowerBehavior);
//            bL.setZeroPowerBehavior(zeroPowerBehavior);
//            fR.setZeroPowerBehavior(zeroPowerBehavior);
//            bR.setZeroPowerBehavior(zeroPowerBehavior);
//
//            fL.setDirection(DcMotorSimple.Direction.REVERSE);
//            bL.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            this.tele = tele;
//        }
//
//        public void takeTeleInput(double gpy, double gpx, double gpt) {
//            xIn = gpy;
//            yIn = gpx;
//            zIn= gpt;
//            normalizer = Math.max(Math.abs(xIn) + Math.abs(yIn) + Math.abs(zIn), 1);
//        }
//
//        public void update() {
//            if (tele) {
//                fL.setPower((xIn + yIn + zIn)/normalizer);
//                bL.setPower((xIn - yIn + zIn)/normalizer);
//                fR.setPower((xIn - yIn - zIn)/normalizer);
//                bR.setPower((xIn + yIn - zIn)/normalizer);
//            }
//        }
//
//        public void updateFC(double headingRad) {
//            double rotX = xIn * Math.cos(headingRad) - yIn * Math.sin(headingRad);
//            double rotY = xIn * Math.sin(headingRad) + yIn * Math.cos(headingRad);
//            normalizer = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(zIn), 1);
//            fL.setPower((rotX + rotY + zIn)/normalizer);
//            bL.setPower((rotX - rotY + zIn)/normalizer);
//            fR.setPower((rotX - rotY - zIn)/normalizer);
//            bR.setPower((rotX + rotY - zIn)/normalizer);
//        }
//
//    }
//
//
//    @Override
//    public void init() {
//        drivebase = new Drivebase(hardwareMap, true);
//        intake = new Intake(hardwareMap);
//
//        controller = new GamepadEx(gamepad1);
//
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//
//        pinpoint.setEncoderDirections(xDirection, yDirection);
//
//        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);
//
//        //UHHH REPLACE
//        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0));
//
//        turret = new Turret(hardwareMap, false);
//
//        shooter = new AdamShooter(hardwareMap, telemetry);
//    }
//
//    double looptime = 0; boolean turretUpdateFlag = true;
//    ElapsedTime looptimer = new ElapsedTime();
//
//    @Override
//    public void loop() {
//
//        looptimer.reset();
//
//        telemetry.addData("loop time (ms)", looptime);
//        telemetry.addData("loop time (hz)", (1000/looptime));
//
//        pinpoint.update();
//
//        if (gamepad1.a && !isAPressed) {
//            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 144-8.795, 8, AngleUnit.RADIANS, 0));
//        } isAPressed = gamepad1.a;
//
//        double heading = pinpoint.getHeading(AngleUnit.RADIANS); xPos = pinpoint.getPosX(DistanceUnit.INCH); yPos = pinpoint.getPosY(DistanceUnit.INCH);
//        drivebase.takeTeleInput(controller.getLeftY(), controller.getLeftX(), controller.getRightX());
//
//        turret.setPose(new Pair<>(xPos, yPos), Math.toDegrees(heading));
//
//        if (gamepad1.right_bumper && !isDpadDownPressed) {
//            shootingMode = !shootingMode;
//            close = true;
//        } else if(gamepad1.dpad_down && !isDpadDownPressed) {
//            shootingMode = !shootingMode;
//            close = false;
//        } isDpadDownPressed = gamepad1.right_bumper || gamepad1.dpad_down;
//
//        if (gamepad1.left_bumper) {
//            intake.spinIn();
//        } else if(gamepad1.left_bumper){
//            intake.spinOut();
//        }
//        else intake.spinIdle();
//
//        if (shootingMode) {
//            turret.setMode(Turret.Mode.odo);
//            meters = turret.distanceToGoal(xPos, yPos) * 0.0254;
//            if (close) {
//                shooter.setTargetRPM(shooter.getRPMForShot(meters) + c);
//                shooter.setHoodAngle(shooter.getHoodAngle(meters));
//                telemetry.addData("target rpm", shooter.getRPMForShot(meters) + c);
//            } else {
//                shooter.setTargetRPM(shooter.getRPMForShot(meters) + f);
//                shooter.setHoodAngle(Math.min(shooter.getHoodAngle(meters), 47));
//                telemetry.addData("target rpm", shooter.getRPMForShot(meters) + f);
//            }
//
//            telemetry.addData("meters", meters);
//
//            shooter.runShooter();
//        }
//
//        telemetry.addData("is shootingModeOn", shootingMode);
//        telemetry.addData("is close", close);
//        telemetry.addData("pose x", xPos);
//        telemetry.addData("pose y", yPos);
//        telemetry.addData("heading", Math.toDegrees(heading));
//
//        drivebase.update();
//        turret.update();
//        telemetry.update();
//
//        looptime = looptimer.milliseconds();
//    }
//}