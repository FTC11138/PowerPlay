package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Right Blue Cycle", group = "Linear Opmode")
public class RightBlueCycleAuto extends AutonomousMethods {
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
        double overallStart = runtime.milliseconds();

        {
//        multitaskMovement(0, Constants.liftHigh, Constants.autoTurnFirstTall, Constants.autoSlideFirstTall, 33, 0.5);
//        sleep(500);
//        encoderTurn(0, 0.3, 1);
//        myRobot.setSlideServo(Constants.autoSlideTurn);
//        myRobot.setRotateMotor(0.5, Constants.rot90R);
//        sleep(250);
//        setLiftMotor(0, Constants.liftTolerance);
//        myRobot.setSlideServo(Constants.autoSlideCycle);
        }
        myRobot.setLiftMotor(1, Constants.liftHigh);
        myRobot.setRotateMotor(0.5, Constants.autoTurnFirstTall);

        encoderStraightDrive(33, 0.5);

        myRobot.setSlideServo(Constants.autoSlideFirstTall);
        sleep(750);
        myRobot.setLiftMotor(0.3, Constants.liftHigh + 200);
        sleep(1000);
        myRobot.setClawServo(Constants.clawOpen);
        sleep(Constants.clawOpenDelay);
        myRobot.setSlideServo(Constants.slideIn);
        sleep(360);
        myRobot.setRotateMotor(0.5, Constants.rot90R);
        myRobot.setLiftMotor(1, 0);
        myRobot.setSlideServo(Constants.autoSlideCycle);
        sleep(1000);

        lineAlign(0, false);
        encoderTurn(0, 0.3, 1);

        do {
            setLiftMotor(4 * Constants.autoLiftCone, 5);
            myRobot.setLiftMotor(0.75, 4 * Constants.autoLiftCone);
            toTargetDistance(Constants.autoDistCycle, true, 0.3, 5000, 5, 0.5);
        }
        while ((dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall) == -1)
        && (30000 - (runtime.milliseconds() - overallStart)) > Constants.parkBuffer + 5000);
        sleep(500);


        for (int i = 3; i >= 0 && ((30000 - (runtime.milliseconds() - overallStart)) > Constants.parkBuffer + 10000); i--) {
            // Reset to stack
            resetCycle(i * Constants.autoLiftCone, Constants.rot90R, Constants.autoSlideCycle + Constants.slideCycleBack);
            myRobot.setSlideServo(Constants.autoSlideCycle);
            sleep((long)(Constants.slideCycleBack * Constants.slideWaitARatio));
            // Drop cone
            while ((dropCone(Constants.liftHigh, i * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall) == -1)
                    && ((30000 - (runtime.milliseconds() - overallStart)) > Constants.parkBuffer + 5000)) {
                myRobot.setSlideServo(Constants.autoSlideCycle - Constants.slideCycleBack);
                myRobot.setClawServo(Constants.clawOpen);
                setLiftMotor(i * Constants.autoLiftCone, 5);
                myRobot.setLiftMotor(0.75, i * Constants.autoLiftCone);
                myRobot.setSlideServo(Constants.autoSlideCycle);
                sleep((long)(Constants.slideCycleBack * Constants.slideWaitARatio));
            }
        }

        // Reset to front
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetFront();
        encoderStraightDrive(5, 0.5);
        encoderStraightDrive(-5, 0.5);

        // Park
        if (signal == 1) {
            encoderStrafeDriveInchesRight(-27, 0.75);
        } else if (signal == 2) {
            encoderStrafeDriveInchesRight(-8, 0.75);
        } else {
            encoderStrafeDriveInchesRight(11, 0.75);
        }
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftMotor(0, 3);

//        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }
}
