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

        dropCone(Constants.liftLow, Constants.autoTurnShort, Constants.autoSlideShort);

//        multitaskMovement(0, Constants.rot90R, 32, 0.75);
//
//        myRobot.setLiftMotor(0.5, -550);
//        myRobot.setClawServo(Constants.clawClose);
//        encoderStrafeDriveInchesRight(8, 0.3);
//        myRobot.setSlideServo(Constants.slideOut);
//        myRobot.setClawServo(Constants.clawOpen);
//        sleep(1000);
//        myRobot.setLiftMotor(0.5, -1000);
//        sleep(500);
//        myRobot.setSlideServo(Constants.slideIn);
//        myRobot.setRotateMotor(0.5, -55 * Constants.rotMotorPosPerDegree);
//        myRobot.setLiftMotor(1, Constants.liftHigh);
//        sleep(2000);
//        myRobot.setSlideServo(Constants.slideOut + 0.03);
//        sleep(2000);
//        myRobot.setLiftMotor(0.3, -3000);
//        myRobot.setClawServo(Constants.clawClose);
//        sleep(5000);
//
    }
}
