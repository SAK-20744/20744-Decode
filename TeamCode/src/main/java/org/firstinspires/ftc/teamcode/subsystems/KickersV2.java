package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KDOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.KUP;
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

import java.util.Queue;
import java.util.ArrayDeque;
public class KickersV2 {
    Servo lKicker, mKicker, rKicker;
    ElapsedTime kickerTimer = new ElapsedTime();
    Kicker currentUp = null;
    Queue<Kicker> queue = new ArrayDeque<Kicker>();
    public KickersV2(HardwareMap hardwareMap) {
        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);
    }
    public void init() {
        lKicker.setPosition(LKICKER_DOWN);
        mKicker.setPosition(MKICKER_DOWN);
        rKicker.setPosition(RKICKER_DOWN);
    }
    public void kick(Kicker kicker) {
        if (queue.size() >= 3) return;
        queue.add(kicker);
    }
    public void kickAll() {kick(Kicker.L); kick(Kicker.M); kick(Kicker.R);}
    public void kickSequenced(String[] sequence) {
        for (String i: sequence) {
            if (i.toLowerCase() == "l") kick(Kicker.L);
            if (i.toLowerCase() == "m") kick(Kicker.M);
            if (i.toLowerCase() == "r") kick(Kicker.R);
        }
    }
    public void periodic() {
        if (currentUp == null && !queue.isEmpty()) {
            kickerTimer.reset();
            currentUp = queue.poll();
        }
        if (currentUp != null)
            switch (currentUp) {
            case L: updateKicker(lKicker,LKICKER_UP,LKICKER_DOWN); break;
            case M: updateKicker(mKicker,MKICKER_UP,MKICKER_DOWN); break;
            case R: updateKicker(rKicker,RKICKER_UP,RKICKER_DOWN); break;
        }
    }
    public Kicker getUp() {return currentUp;}
    public Kicker getQueued() {return queue.peek();}
    public double kickerTimer() {return kickerTimer.milliseconds();}
    public boolean kickerGoingUp() {return kickerTimer() < KUP;}
    public boolean kickerGoingDown() {return kickerTimer() < KDOWN+KUP;}
    private void updateKicker(Servo servo, double upPos, double downPos) {
        if (kickerGoingUp())           servo.setPosition(upPos);
        else if (kickerGoingDown())    servo.setPosition(downPos);
        else                           currentUp = null;
    }
    public boolean kickersActive() {return currentUp != null || queue.peek() != null;}
}
