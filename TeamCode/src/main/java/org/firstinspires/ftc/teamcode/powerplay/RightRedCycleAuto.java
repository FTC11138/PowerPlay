package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Red Cycle", group = "Linear Opmode", preselectTeleOp = "TeleOp")
@Config
public class RightRedCycleAuto extends RightCycleAuto {
    public static double autoSlideFirstTall = 0.53;
    public static double autoSlideTall = 0.36;
    public static int autoTurnFirstTall = 1650;
    public static int autoTurnTall = 1500;

    @Override
    public boolean isRed() {
        return true;
    }

    @Override
    public double getAutoSlideFirstTall() {
        return autoSlideFirstTall;
    }

    @Override
    public double getAutoSlideTall() {
        return autoSlideTall;
    }

    @Override
    public int getAutoTurnFirstTall() {
        return autoTurnFirstTall;
    }

    @Override
    public int getAutoTurnTall() {
        return autoTurnTall;
    }
}
