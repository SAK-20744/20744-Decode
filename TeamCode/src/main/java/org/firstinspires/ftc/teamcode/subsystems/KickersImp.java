package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

@Configurable
public class KickersImp {
    Servo[] kickers;
    public double[] kickerUpPos = new double[] {
            ApolloConstants.LKICKER_UP,
            ApolloConstants.MKICKER_UP,
            ApolloConstants.RKICKER_UP
    };
    public double[] kickerDownPos = new double[] {
            ApolloConstants.LKICKER_DOWN,
            ApolloConstants.MKICKER_DOWN,
            ApolloConstants.RKICKER_DOWN
    };
    public static double kickerUpTime = 0.5;
    public static double kickerDownTime = 0.5;

    public static double kickerUpTimeSlow = 0.75;
    public static double kickerDownTimeSlow = 0.75;
    public boolean slowed = false;

    public KickersImp(HardwareMap hardwareMap) {
        kickers = new Servo[] {
                hardwareMap.get(Servo.class, ApolloHardwareNames.lKicker),
                hardwareMap.get(Servo.class, ApolloHardwareNames.mKicker),
                hardwareMap.get(Servo.class, ApolloHardwareNames.rKicker)
        };
    }

    public SequentialGroup kick(Kicker kicker) {
        try {
            if (!slowed) return new SequentialGroup(
                    new InstantCommand(() -> kickers[kicker.id()].setPosition(kickerUpPos[kicker.id()])),
                    new Delay(kickerUpTime),
                    new InstantCommand(() -> kickers[kicker.id()].setPosition(kickerDownPos[kicker.id()])),
                    new Delay(kickerDownTime)
            );
            else return new SequentialGroup(
                    new InstantCommand(() -> kickers[kicker.id()].setPosition(kickerUpPos[kicker.id()])),
                    new Delay(kickerUpTimeSlow),
                    new InstantCommand(() -> kickers[kicker.id()].setPosition(kickerDownPos[kicker.id()])),
                    new Delay(kickerDownTimeSlow)
            );
        } catch (IndexOutOfBoundsException e) {
            throw new IndexOutOfBoundsException("Null kicker found");
        }
    }

    public SequentialGroup kick(int kicker) {
        try {
            if (!slowed) return new SequentialGroup(
                    new InstantCommand(() -> kickers[kicker].setPosition(kickerUpPos[kicker])),
                    new Delay(kickerUpTime),
                    new InstantCommand(() -> kickers[kicker].setPosition(kickerDownPos[kicker])),
                    new Delay(kickerDownTime)
            );
            else return new SequentialGroup(
                    new InstantCommand(() -> kickers[kicker].setPosition(kickerUpPos[kicker])),
                    new Delay(kickerUpTimeSlow),
                    new InstantCommand(() -> kickers[kicker].setPosition(kickerDownPos[kicker])),
                    new Delay(kickerDownTimeSlow)
            );
        } catch (IndexOutOfBoundsException e) {
            throw new IndexOutOfBoundsException("Null kicker found");
        }
    }

    public SequentialGroup kickAll() {
        return new SequentialGroup(
                kick(0),
                kick(1),
                kick(2)

        );
    }

    public SequentialGroup kickSequenced(Kicker[] order) {
        return new SequentialGroup(
                kick(order[0]),
                kick(order[1]),
                kick(order[2])

        );
    }
}
