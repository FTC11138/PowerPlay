package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;

@Autonomous(name = "BlueRight", group = "Linear Opmode")
public class BlueRight extends BaseAutonomousMethods{

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        waitForStart();
        runtime.reset();

        sleep(120);

        encoderStraightDrive(48, 0.5);
    }
}
