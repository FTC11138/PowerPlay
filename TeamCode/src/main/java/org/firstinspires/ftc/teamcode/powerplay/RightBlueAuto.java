package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

        multitaskMovement(0, Constants.liftHigh, Constants.autoTurnFirstTall, Constants.autoSlideFirstTall, 33, 0.5);
        sleep(500);
        encoderTurn(0, 0.3, 1);

        myRobot.setSlideServo(Constants.autoSlideTurn);
        myRobot.setRotateMotor(0.5, Constants.rot90R);
        sleep(250);
        setLiftMotor(0, Constants.liftTolerance);

        myRobot.setSlideServo(Constants.autoSlideCycle);
        lineAlign(0);
        encoderTurn(0, 0.3, 1);

        setLiftMotor(4 * Constants.autoLiftCone, 5);
        myRobot.setLiftMotor(0.75, 4 * Constants.autoLiftCone);
        toTargetDistance(Constants.autoDistCycle, true, 0.3, 5000, 25, 0.75);
        myRobot.setClawServo(Constants.clawClose);
        sleep(500);
        dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall);
        resetCycle(4 * Constants.autoLiftCone, Constants.rot90R, Constants.autoSlideCycle);

        for (int i = 3; i >= 0 /* condition to check time left using runtime based on detection*/; i--) {
            // Reset to stack
            // Drop cone
        }
        // Reset to front
        // Park

        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }
}
