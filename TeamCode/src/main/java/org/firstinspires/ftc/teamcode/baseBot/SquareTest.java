package org.firstinspires.ftc.teamcode.baseBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SquareTest", group = "Linear Opmode")

public class SquareTest extends BaseAutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        waitForStart();

        runtime.reset();

        sleep(120);

        encoderStrafeDriveInchesRight(50, 0.5);
        encoderStraightDrive(-22,0.5);
        encoderStrafeDriveInchesRight(-50, 0.5);
        encoderStraightDrive(22,0.5);
    }
}