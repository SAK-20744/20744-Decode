//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.config.FieldPoses;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
//
//@Autonomous
//public class redFarSix extends OpMode {
//    Follower drive;
//    @Override
//    public void init() {
//        drive = Constants.createFollower(hardwareMap);
//        drive.setPose(FieldPoses.redFarStart);
//    }
//    @Override
//    public void start() {
//        Shoot();
//        drive.followPath(); // Put drive to human player path in here
//        while(drive.isBusy()) {
//            drive.update();
//            if (drive.getPathCompletion() > 0.75) {
//                //start intaking
//            }
//            Telemetry();
//        }
//        drive.followPath(); // Drive back to redFarStart to be back in far launch zone
//        while (drive.isBusy()) {
//            drive.update();
//            Telemetry();
//        }
//        Shoot();
//    }
//    @Override
//    public void loop() {
//
//    }
//    public void Shoot() {
//        //Shoot stuff here
//    }
//    public void Telemetry() {
//        telemetry.addData("Bot Pose",drive.getPose());
//        telemetry.addData("Path Completion",drive.getPathCompletion());
//        telemetry.update();
//    }
//}
