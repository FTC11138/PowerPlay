package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Blue Cycle", group = "Linear Opmode", preselectTeleOp = "TeleOp")
@Config
public class LeftBlueCycleAuto extends LeftCycleAuto {
    public static double autoSlideFirstTall = 0.58;
    public static double autoSlideTall = 0.4;
    public static int autoTurnFirstTall = -1675;
    public static int autoTurnTall = -1450;
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
