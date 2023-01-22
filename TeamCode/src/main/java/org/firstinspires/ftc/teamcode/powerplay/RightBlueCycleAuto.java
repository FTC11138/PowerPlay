package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Blue Cycle", group = "Linear Opmode", preselectTeleOp = "TeleOp")
@Config
public class RightBlueCycleAuto extends RightCycleAuto {
    public static double autoSlideFirstTall = 0.68;
    public static double autoSlideTall = 0.85;
    public static int autoTurnFirstTall = 1580;
    public static int autoTurnTall = 1465;

    @Override
    public boolean isRed() {
        return false;
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
