package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Red Cycle", group = "Linear Opmode")
public class LeftRedCycleAuto extends RightCycleAuto {
    @Override
    public boolean isRed() {
        return true;
    }
}
