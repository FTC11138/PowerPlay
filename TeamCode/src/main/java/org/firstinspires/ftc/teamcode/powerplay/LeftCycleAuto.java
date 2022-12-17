package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class LeftCycleAuto extends AutonomousMethods {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    OpenCvWebcam webcam;
    SignalDetectionPipeline signalDetectionPipeline;
    ConeAlignmentPipelineV2 coneAlignPipeline;
    int signal = 3;
    double cameraError = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all the parts of the robot
        initializeAuto(hardwareMap, telemetry);
        double parkBuffer = Constants.parkBuffer;
        myRobot.setClawServo(Constants.clawClose);

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start showing the camera
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
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
        if (signal == 3) {
            parkBuffer += 1500;
        }
        coneAlignPipeline = new ConeAlignmentPipelineV2(isRed());
        webcam.setPipeline(coneAlignPipeline);

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
        myRobot.setRotateMotor(0.5, Constants.rot90L);

        encoderStraightDrive(-48, 0.5);
        encoderTurn(0, 0.3, 1);
        sleep(2000);
        // todo alignment here
        cameraError = (coneAlignPipeline.getMiddle() - 960) * Constants.alignRatio;
//        while (Math.abs(cameraError) > Constants.cameraTolerance) {
//            cameraError = (coneAlignPipeline.getMiddle() - 960) * Constants.alignRatio;
        encoderStraightDrive(cameraError, 0.3);
//        }

        myRobot.setRotateMotor(0.5, -Constants.autoTurnFirstTall);
        myRobot.setSlideServo(Constants.autoSlideFirstTall);
        sleep(750);
        myRobot.setLiftMotor(0.5, Constants.liftHigh + 200);
        sleep(500);
        myRobot.setClawServo(Constants.clawOpen);
        sleep(Constants.clawOpenDelay);
        myRobot.setSlideServo(Constants.slideIn);
        sleep(360);
//        myRobot.setRotateMotor(0.5, Constants.rot90L);
//        myRobot.setLiftMotor(1, 0);
//        myRobot.setSlideServo(Constants.autoSlideCycle);
//        // todo motor.isactive()
//        sleep(3000);
//
//        lineAlign(0, false);
//        encoderTurn(0, 0.3, 1);
//
//        do {
//            setLiftMotor(4 * Constants.autoLiftCone, 5);
//            myRobot.setLiftMotor(0.75, 4 * Constants.autoLiftCone);
//            myRobot.setSlideServo(Constants.autoSlideCycle);
//            sleep(250);
//            toTargetDistance(Constants.autoDistCycle, false, 0.3, 5000, 5, 0.5);
//        }
//        while ((dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, -Constants.autoTurnTall, Constants.autoSlideTall) == -1)
//                && (30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + 5000);
////        setLiftMotor(4 * Constants.autoLiftCone, 5);
////        myRobot.setLiftMotor(0.75, 4 * Constants.autoLiftCone);
////        toTargetDistance(Constants.autoDistCycle, false, 0.3, 5000, 5, 0.5);
////        dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, -Constants.autoTurnTall, Constants.autoSlideTall);
//        sleep(500);
//
//
//        for (int i = 3; i >= 0 && ((30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + Constants.cycleTime); i--) {
//            // Reset to stack
//            resetCycle(i * Constants.autoLiftCone, Constants.rot90L, Constants.autoSlideCycle + Constants.slideCycleBack);
//            myRobot.setSlideServo(Constants.autoSlideCycle);
//            sleep((long)(Constants.slideCycleBack * Constants.slideWaitARatio));
//            // Drop cone
//            while ((dropCone(Constants.liftHigh, i * Constants.autoLiftCone + Constants.coneDodge, -Constants.autoTurnTall, Constants.autoSlideTall) == -1)
//                    && ((30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + 5000)) {
//                myRobot.setSlideServo(Constants.autoSlideCycle - Constants.slideCycleBack);
//                myRobot.setClawServo(Constants.clawOpen);
//                setLiftMotor(i * Constants.autoLiftCone, 5);
//                myRobot.setLiftMotor(0.75, i * Constants.autoLiftCone);
//                myRobot.setSlideServo(Constants.autoSlideCycle);
//                sleep((long)(Constants.slideCycleBack * Constants.slideWaitARatio));
//            }
////            dropCone(Constants.liftHigh, i * Constants.autoLiftCone + Constants.coneDodge, -Constants.autoTurnTall, Constants.autoSlideTall);
//        }
//
//        // Reset to front
//        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        resetFront();
//        encoderStraightDrive(5, 0.5);
//        encoderStraightDrive(-5, 0.5);
//
//        // Park todo make function
//        if (signal == 1) {
//            encoderStrafeDriveInchesRight(-11, 0.75);
//        } else if (signal == 2) {
//            encoderStrafeDriveInchesRight(8, 0.75);
//        } else {
//            encoderStrafeDriveInchesRight(27, 0.75);
//        }
//        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        setLiftMotor(0, 3);
//
////        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }

    public abstract boolean isRed();
}
