package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ReliabilityStrafeTest", group = "Linear Opmode")
public class ReliabilityStrafeTest extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        sleep(1000);

        encoderStraightDrive(Constants.straightTestDist, Constants.straightTestPow);

//        for (int i=0; i<5; i++) {
//            encoderStraightDrive(20, 0.7);
//            sleep(500);
//            encoderStraightDrive(-20, 0.7);
//            sleep(500);
//        }

    }
}
