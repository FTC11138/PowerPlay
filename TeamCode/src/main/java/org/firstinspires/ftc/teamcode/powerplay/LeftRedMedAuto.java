package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Red Med", group = "Linear Opmode", preselectTeleOp = "TeleOp")
@Config
public class LeftRedMedAuto extends LeftMedAuto {
    public static double autoSlideFirstMed = 0.61;
    public static double autoSlideMed = 0.82;
    public static int autoTurnFirstMed = -620;
    public static int autoTurnMed = -745;

    @Override
    public boolean isRed() {
        return true;
    }

    @Override
    public double getAutoSlideFirstMed() {
        return autoSlideFirstMed;
    }

    @Override
    public double getAutoSlideMed() {
        return autoSlideMed;
    }

    @Override
    public int getAutoTurnFirstMed() {
        return autoTurnFirstMed;
    }

    @Override
    public int getAutoTurnMed() {
        return autoTurnMed;
    }
}
