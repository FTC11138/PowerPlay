package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Blue Med", group = "Linear Opmode", preselectTeleOp = "TeleOp")
@Config
public class RightBlueMedAuto extends RightMedAuto {
    public static double autoSlideFirstMed = 0.61;
    public static double autoSlideMed = 0.78;
    public static int autoTurnFirstMed = 600;
    public static int autoTurnMed = 730;

    @Override
    public boolean isRed() {
        return false;
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
