package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Left Blue High", group = "Linear Opmode", preselectTeleOp = "TeleOp")
@Config

public class LeftBlueHighAuto extends LeftHighAuto {
    public static double autoSlideFirstTall = 0.625;
    public static double autoSlideTall = 0.78;
    public static int autoTurnFirstTall = -1565;
    public static int autoTurnTall = -1435;
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
