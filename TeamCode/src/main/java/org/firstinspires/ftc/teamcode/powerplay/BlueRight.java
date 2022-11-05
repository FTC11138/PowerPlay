package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "BlueRight", group = "Linear Opmode")
public class BlueRight extends BaseAutonomousMethods{

    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    private ElapsedTime runtime = new ElapsedTime();
    private AutoAttachments robot = new AutoAttachments();

    OpenCvWebcam webcam;
    org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline signalDetectionPipeline;
    int signal = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {telemetry.update();
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        robot.initialize(hardwareMap, telemetry);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        runtime.reset();

        sleep(2000);

        signal = signalDetectionPipeline.getCounter();
        telemetry.addData("Signal", signal);
        telemetry.update();

        robot.setClawServo(Constants.clawOpen);
        sleep(500);
        robot.setLiftMotor(0.1, -100);

        encoderStraightDrive(36, 0.5);
        sleep(500);

        robot.setLiftMotor(0.5, Constants.liftHigh);
        robot.setRotateMotor(0.5, -43 * Constants.rotMotorPosPerDegree);
        sleep(2000);
        robot.setSlideServo(0.33);
        sleep(2000);
        robot.setLiftMotor(0.3, Constants.liftHigh + 200);
        sleep(1000);
        robot.setClawServo(Constants.clawClose);
        sleep(1000);
        robot.setSlideServo(Constants.slideIn);
        robot.setRotateMotor(0.5, 0);

        encoderStraightDrive(4, 0.5); // push signal cone ahead
        sleep(500);

        encoderStraightDrive(-4, 0.5); // move back
        sleep(500);

        robot.setLiftMotor(0.3, 0);
        sleep(3000);

        do {
            if (runtime.seconds() >= 5) {
                if (signal == 1) {
                    encoderStrafeDriveInchesRight(-18, 0.5); // turn left
                } else if (signal == 3) {
                    encoderStrafeDriveInchesRight(18, 0.5); // turn right
                }

                if (Constants.debugMode) {
                    sleep(2000);
                    // return to original, for testing purpose. REMOVE IT before competition!!!!
                    if (signal == 1) {
                        encoderStrafeDriveInchesRight(18, 0.5); // turn right
                    } else if (signal == 3) {
                        encoderStrafeDriveInchesRight(-18, 0.5); // turn left
                    }
                    encoderStraightDrive(-36, 0.5);
                }

                break;
            }
        } while (true);
        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }
}
