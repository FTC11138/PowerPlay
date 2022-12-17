package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Blue Cycle", group = "Linear Opmode")
public class RightBlueCycleAuto extends RightCycleAuto {
    @Override
    public boolean isRed() {
        return false;
    }
}
