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

        encoderStraightDrive(Constants.test1, Constants.test2, Constants.test3, Constants.test4);
        encoderTurn(0, 0.5, 1);
        sleep(1000);
//        encoderStrafeDriveInchesRight(50, 0.5);
//        for (int i = 0; i < 3 && opModeIsActive(); i++) {
//            encoderStraightDrive(Constants.test1, Constants.test2, Constants.test3, Constants.test4);
//            encoderTurn(0, 0.3, 1);
//            sleep(1000);
//            encoderStraightDrive(-Constants.test1, Constants.test2, Constants.test3, Constants.test4);
//            encoderTurn(0, 0.3, 1);
//            sleep(1000);
//        }
        //        encoderStrafeDriveInchesRight(-50, 0.5);
//        encoderStraightDrive(22,0.5);
    }
}