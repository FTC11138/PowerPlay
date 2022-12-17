package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Blue Cycle", group = "Linear Opmode")
public class LeftBlueCycleAuto extends RightCycleAuto {
    @Override
    public boolean isRed() {
        return false;
    }
}
