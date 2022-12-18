package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

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

//        encoderStrafeDriveInchesRight(50, 0.5);
        while (opModeIsActive()) {
            myRobot.lb.setVelocityPIDFCoefficients(Constants.drivekP, Constants.drivekI, Constants.drivekD, Constants.drivekF);
            myRobot.lb.setPositionPIDFCoefficients(Constants.drivePoskP);
            myRobot.lf.setVelocityPIDFCoefficients(Constants.drivekP, Constants.drivekI, Constants.drivekD, Constants.drivekF);
            myRobot.lf.setPositionPIDFCoefficients(Constants.drivePoskP);
            myRobot.rb.setVelocityPIDFCoefficients(Constants.drivekP, Constants.drivekI, Constants.drivekD, Constants.drivekF);
            myRobot.rb.setPositionPIDFCoefficients(Constants.drivePoskP);
            myRobot.rf.setVelocityPIDFCoefficients(Constants.drivekP, Constants.drivekI, Constants.drivekD, Constants.drivekF);
            myRobot.rf.setPositionPIDFCoefficients(Constants.drivePoskP);
            encoderStraightDrive(Constants.test1, Constants.test2);
            encoderTurn(0, 0.3, 1);
            sleep(1000);
            encoderStraightDrive(-Constants.test1, Constants.test2);
            encoderTurn(0, 0.3, 1);
            sleep(1000);
        }
        //        encoderStrafeDriveInchesRight(-50, 0.5);
//        encoderStraightDrive(22,0.5);
    }
}