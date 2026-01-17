//package org.firstinspires.ftc.teamcode.opmode.tests;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.util.PoseHistory;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.config.FieldPoses;
//import org.firstinspires.ftc.teamcode.subsystems.CrapTurret;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.util.Drawing;
//
//@TeleOp (group="UnitTest")
//public class CrapTurretTest extends LinearOpMode {
//    CrapTurret turret;
//    Follower drive;
//
//    boolean tOnPressed = false, tOffPressed = false;
//    boolean targetPressed = false;
//    double looptime = 0;
//
//    @IgnoreConfigurable
//    static PoseHistory poseHistory;
//
//    @IgnoreConfigurable
//    public static TelemetryManager telemetryM;
//
//
//
//    @Override
//    public void runOpMode() {
//        drive = Constants.createFollower(hardwareMap);
//        turret = new CrapTurret(hardwareMap);
//        turret.off();
//
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        Drawing.init();
//
//
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        while (opModeInInit()) {
//            if (gamepad1.x) turret.resetTurret();
//            telemetry.addData("Turret Angle",turret.getYaw());
//            telemetry.addData("Get Turret", turret.getTurret());
//            telemetry.update();
//        }
//        waitForStart();
//        while (opModeIsActive()) {
//
//            {
//                Drawing.drawDebug(drive);
//            }
//
//            if (gamepad1.a && !tOnPressed)
//                turret.on();
//            tOnPressed = gamepad1.a;
//            if (gamepad1.b && !tOffPressed)
//                turret.off();
//            tOffPressed = gamepad1.b;
//
//            if (gamepad1.x && !targetPressed)
//                turret.setYaw(-Math.PI/2);
//            targetPressed = gamepad1.x;
//
//            if (gamepad1.y) turret.face(FieldPoses.redHoop, drive.getPose());
//
//            turret.periodic();
//            drive.updatePose();
//            telemetry.addData("Robot Pose", drive.getPose());
//            telemetry.addData("Turret Angle",turret.getYaw());
//            telemetry.addData("Get Turret", turret.getTurret());
//            telemetry.addData("Turret Target", turret.getTurretTarget());
//            double loop = System.nanoTime();
//            telemetry.addData("hz ", 1000000000 / (loop - looptime));
//            looptime = loop;
//            telemetry.update();
//        }
//    }
//}
