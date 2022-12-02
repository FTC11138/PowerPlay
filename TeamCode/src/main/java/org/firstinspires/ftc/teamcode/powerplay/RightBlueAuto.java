package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RightAutoWithColor2", group = "Linear Opmode")
public class RightBlueAuto extends AutonomousMethods {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    OpenCvWebcam webcam;
    SignalDetectionPipeline signalDetectionPipeline;
    int signal = 3;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all the parts of the robot
        initializeAuto(hardwareMap, telemetry);
        myRobot.setClawServo(Constants.clawClose);
        myRobot.colorSensor.setGain(Constants.gain);

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start showing the camera
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Detect the number on the cone before start
        while (!isStarted()) {
            signal = signalDetectionPipeline.getCounter();
            telemetry.addData("Signal", signal);
            telemetry.update();
        }
        runtime.reset();

        // Detect the number on the cone after start
        signal = signalDetectionPipeline.getCounter();
        telemetry.addData("Final Signal", signal);
        telemetry.update();

        multitaskMovement(0, Constants.liftHigh, Constants.autoTurnTall, Constants.autoSlideTall, 33, 0.5);
        sleep(500);
        encoderTurn(0, 0.3, 1);

        myRobot.setSlideServo(Constants.autoSlideTurn);
        myRobot.setRotateMotor(0.5, Constants.rot90R);
        setLiftMotor(0, Constants.liftTolerance);
        sleep(250);

        myRobot.setSlideServo(Constants.autoSlideCycle);
        lineAlign(0);
        encoderTurn(0, 0.3, 1);



        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }
}
