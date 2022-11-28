package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RightAutoWithColor2", group = "Linear Opmode")
public class RightAutoWithColor2 extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto(hardwareMap, telemetry);

        myRobot.setClawServo(Constants.clawClose);
        sleep(250);
        myRobot.setLiftMotor(0.25, Constants.liftFloor);
        myRobot.colorSensor.setGain(Constants.gain);

        waitForStart();
        runtime.reset();

//        encoderStraightDrive(1, 0.3);
//        encoderStrafeDriveInchesRight(-4, 0.5);

        multitaskMovement(0, Constants.rot90R, 32, 0.75);

        myRobot.setLiftMotor(0.5, -550);
        myRobot.setClawServo(Constants.clawClose);
        encoderStrafeDriveInchesRight(8, 0.3);
        myRobot.setSlideServo(Constants.slideOut);
        myRobot.setClawServo(Constants.clawOpen);
        sleep(1000);
        myRobot.setLiftMotor(0.5, -1000);
        sleep(500);
        myRobot.setSlideServo(Constants.slideIn);
        myRobot.setRotateMotor(0.5, -55 * Constants.rotMotorPosPerDegree);
        myRobot.setLiftMotor(1, Constants.liftHigh);
        sleep(2000);
        myRobot.setSlideServo(Constants.slideOut + 0.03);
        sleep(2000);
        myRobot.setLiftMotor(0.3, -3000);
        myRobot.setClawServo(Constants.clawClose);
        sleep(5000);

    }
}
