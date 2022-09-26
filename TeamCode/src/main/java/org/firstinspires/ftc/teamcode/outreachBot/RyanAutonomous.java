package org.firstinspires.ftc.teamcode.outreachBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;

@Autonomous(name = "Autonomous", group = "Linear Opmode")

public class RyanAutonomous extends BaseAutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeAuto(hardwareMap, telemetry);
        myRobot.setLiftServo(Constants.liftDrive);
        myRobot.setClawServo(Constants.clawOpenB);
        waitForStart();
        runtime.reset();

        sleep(120);

        //Code
        encoderStraightDrive(24, 0.5);
        encoderTurn(90, 0.5, 1);
        encoderStraightDrive(12, 0.8);
        myRobot.setLiftServo(Constants.liftBot);
        sleep(120);
        encoderStraightDrive(5, 0.3);
        myRobot.setClawServo(Constants.clawCloseB);
        sleep(500);
        myRobot.setLiftServo(Constants.liftTop);
        sleep(4000);
        encoderStrafeDriveInchesRight(24, 0.8);
        myRobot.setClawServo(Constants.clawOpenB);
        sleep(3000);

    }
}