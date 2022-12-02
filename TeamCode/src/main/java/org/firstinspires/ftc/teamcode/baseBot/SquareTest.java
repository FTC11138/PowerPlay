package org.firstinspires.ftc.teamcode.baseBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;

@Autonomous(name = "SquareTest", group = "Linear Opmode")
@Disabled
public class SquareTest extends AutonomousMethods {

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