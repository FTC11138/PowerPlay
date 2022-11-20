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

@Autonomous(name = "LeftAuto", group = "Linear Opmode")
public class LeftAuto extends AutonomousMethods {

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

        // Initialize all the parts of the robot
        initializeAuto(hardwareMap, telemetry);

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                // Start showing the camera
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //
        //

        // Detect the number on the cone before start
        signal = signalDetectionPipeline.getCounter();
        telemetry.addData("Signal", signal);
        telemetry.update();

        waitForStart();
        runtime.reset();

        sleep(4000);

        // Detect the number on the cone after start
        signal = signalDetectionPipeline.getCounter();
        telemetry.addData("Signal", signal);
        telemetry.update();

        myRobot.setClawServo(Constants.clawOpen);
        sleep(500);
        myRobot.setLiftMotor(0.1, -100);
        sleep(500);

        // Align robot to center of tile
        encoderStraightDrive(3, 0.2);
        encoderStrafeDriveInchesRight(3, 0.5);

        myRobot.setLiftMotor(0.3, Constants.liftLow);

        encoderStraightDrive(33, 0.5);
        sleep(500);

        myRobot.setLiftMotor(0.5, Constants.liftHigh);
        myRobot.setRotateMotor(0.5, 37 * Constants.rotMotorPosPerDegree);
        sleep(2000);
        myRobot.setSlideServo(Constants.autoSlideOut);
        sleep(2000);
        myRobot.setLiftMotor(0.3, Constants.liftHigh + 200);
        sleep(1000);
        myRobot.setClawServo(Constants.clawClose);
        sleep(1000);
        myRobot.setSlideServo(Constants.slideIn);
        myRobot.setRotateMotor(0.5, 0);

        encoderStraightDrive(4, 0.5); // push signal cone ahead
        sleep(500);

        encoderStraightDrive(-4, 0.5); // move back
        sleep(500);

        myRobot.setLiftMotor(0.3, 0);
        sleep(3000);

        // Park
        do {
            if (runtime.seconds() >= 5) {
                if (signal == 1) {
                    encoderStrafeDriveInchesRight(-18, 0.5); // turn left
                } else if (signal == 3) {
                    encoderStrafeDriveInchesRight(18, 0.5); // turn right
                }
                sleep(3000);

//                if (Constants.debugMode) {
//                    sleep(2000);
//                    encoderTurn(0, 0.5, 5);
//                    // return to original, for testing purpose. REMOVE IT before competition!!!!
//                    if (signal == 1) {
//                        encoderStrafeDriveInchesRight(18, 0.5); // turn right
//                    } else if (signal == 3) {
//                        encoderStrafeDriveInchesRight(-18, 0.5); // turn left
//                    }
//                    encoderStraightDrive(-36, 0.5);
//                }

                break;
            }
        } while (opModeIsActive());

        // Transition to Tele Op
        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }
}
