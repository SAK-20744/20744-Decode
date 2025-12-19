package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KICKER_DOWNTIME;
//import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KICKER_UPTIME;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;

public class cursedKicker {
    public enum Kicker {
        L,M,R
    }

    Servo lKicker, mKicker, rKicker;
//    double kickerUpTime = KICKER_UPTIME, kickerDownTime = KICKER_DOWNTIME; //Time kicker waits till going back down and time before next kicker can go up
    ElapsedTime kickerTimer = new ElapsedTime();
//    Kickers.Kicker currentUp = null, queuedUp = null;
    public cursedKicker(HardwareMap hardwareMap) {
        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);

    }

    public void lKickerUp() {
        lKicker.setPosition(LKICKER_UP);
    }

    public void mKickerUp() {
        mKicker.setPosition(MKICKER_UP);
    }

    public void rKickerUp() {
        rKicker.setPosition(RKICKER_UP);
    }

    public void periodic(){}

    public void lKickerDown() {
        lKicker.setPosition(LKICKER_DOWN);
    }

    public void mKickerDown() {
        mKicker.setPosition(MKICKER_DOWN);
    }

    public void rKickerDown() {
        rKicker.setPosition(RKICKER_DOWN);
    }
}
