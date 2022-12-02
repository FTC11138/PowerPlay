package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RightAuto", group = "Linear Opmode")
public class RightAuto extends AutonomousMethods{

    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();

    OpenCvWebcam webcam;
    org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline signalDetectionPipeline;
    int signal = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {telemetry.update();
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        robot.initialize(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        robot.setClawServo(Constants.clawOpen);
        sleep(1000);

        robot.setLiftMotor(1, Constants.liftHigh);

        encoderStraightDrive(33, 0.5);
        sleep(500);

        robot.setRotateMotor(0.5, Math.round(Constants.temp1 * Constants.rotMotorPosPerDegree));
        sleep(2000);
        robot.setSlideServo(Constants.temp2);
        sleep(1000);
        robot.setClawServo(Constants.clawClose);
        sleep(3000);
        robot.setSlideServo(Constants.slideIn);
        robot.setRotateMotor(0.5, 0);
        robot.setLiftMotor(1, 0);
        sleep(10000);
    }
}
