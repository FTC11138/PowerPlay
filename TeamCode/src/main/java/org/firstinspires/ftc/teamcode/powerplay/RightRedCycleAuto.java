package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Red Cycle", group = "Linear Opmode", preselectTeleOp = "TeleOp")
public class RightRedCycleAuto extends RightCycleAuto {
    @Override
    public boolean isRed() {
        return true;
    }
}
