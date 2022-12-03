package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RightAuto", group = "Linear Opmode")
public class RightAuto extends AutonomousMethods{

    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    private ElapsedTime runtime = new ElapsedTime();

    OpenCvWebcam webcam;
    org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline signalDetectionPipeline;
    int signal = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {telemetry.update();
        initializeAuto(hardwareMap, telemetry);
        waitForStart();
        runtime.reset();

        myRobot.setClawServo(Constants.clawClose);
        sleep(1000);

        dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall);
        resetCycle(3 * Constants.autoLiftCone, Constants.rot90R, Constants.autoSlideCycle);
//        robot.setLiftMotor(1, Constants.liftHigh);
//
//        encoderStraightDrive(33, 0.5);
//        sleep(500);
//
//        robot.setRotateMotor(0.5, Constants.autoTurnFirstTall);
//        sleep(2000);
//        robot.setSlideServo(Constants.autoSlideFirstTall);
//        sleep(1000);
//        robot.setClawServo(Constants.clawOpen);
//        sleep(3000);
//        robot.setSlideServo(Constants.slideIn);
//        robot.setRotateMotor(0.5, 0);
//        robot.setLiftMotor(1, 0);
//        sleep(10000);
    }
}
