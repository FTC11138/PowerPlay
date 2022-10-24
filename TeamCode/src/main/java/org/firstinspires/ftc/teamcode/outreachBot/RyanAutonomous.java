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
        myRobot.setLiftServo(OutreachConstants.liftDrive);
        myRobot.setClawServo(OutreachConstants.clawOpenB);
        waitForStart();
        runtime.reset();

        sleep(120);

        //Code
        encoderStraightDrive(24, 0.5);
        encoderTurn(90, 0.5, 1);
        encoderStraightDrive(12, 0.8);
        myRobot.setLiftServo(OutreachConstants.liftBot);
        sleep(120);
        encoderStraightDrive(5, 0.3);
        myRobot.setClawServo(OutreachConstants.clawCloseB);
        sleep(500);
        myRobot.setLiftServo(OutreachConstants.liftTop);
        sleep(4000);
        encoderStrafeDriveInchesRight(24, 0.8);
        myRobot.setClawServo(OutreachConstants.clawOpenB);
        sleep(3000);

    }
}