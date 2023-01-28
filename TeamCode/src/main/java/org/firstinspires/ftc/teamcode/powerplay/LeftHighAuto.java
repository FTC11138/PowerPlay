package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class LeftHighAuto extends AutonomousMethods {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    OpenCvWebcam webcam;
    SignalDetectionPipeline signalDetectionPipeline;
    ConeAlignmentPipelineV2 coneAlignPipeline;
    int signal = 3;
    double cameraError = 100;

    double autoSlideFirstTall = getAutoSlideFirstTall();
    double autoSlideTall = getAutoSlideTall();
    int autoTurnFirstTall = getAutoTurnFirstTall();
    int autoTurnTall = getAutoTurnTall();

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
        coneAlignPipeline = new ConeAlignmentPipelineV2(isRed());
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
        webcam.setPipeline(coneAlignPipeline);

//        {
//            multitaskMovement(0, Constants.liftHigh, Constants.rot90R, Constants.slideOpt, -48, 0.6);
//        encoderTurn(0, 0.3, 1);
//        }
        {
            myRobot.setLiftMotor(1, Constants.liftHigh);
            myRobot.setRotateMotor(0.75, Constants.rot90L);
//            encoderStraightDrive(-48, 1, 24, 0.5);
            encoderStraightDrive(-48, 0.5);
            encoderTurn(0, 0.5, 1);
        }
        while (myRobot.rotateMotor.isBusy()) {
        }
        sleep(1000);
        cameraError = (coneAlignPipeline.getMiddle() - 960) * Constants.alignRatio;
        encoderStraightDrive(-cameraError, 0.25);

        myRobot.setRotateMotor(0.5, autoTurnFirstTall);
        while (myRobot.rotateMotor.isBusy()) {
        }
        encoderTurn(0, 0.5, 1);
        myRobot.setSlideServo(autoSlideFirstTall);
        sleep(600);
        myRobot.setLiftMotor(1, Constants.liftHigh + 100);
        sleep(150);
        myRobot.setClawServo(Constants.clawFurtherOpen);
        myRobot.setSlideServo(Constants.autoSlideCycle - Constants.slideCycleBack);
        sleep(Constants.clawOpenDelay);
        sleep(100);
        myRobot.setRotateMotor(0.75, Constants.rot90RLong);
        while (Math.abs(myRobot.rotateMotor.getCurrentPosition() - autoTurnFirstTall) < Constants.junctionDodge){
        }

        myRobot.setLiftMotor(1, 4 * Constants.autoLiftCone);
        encoderTurn(0, 0.5, 1);
        myRobot.setClawServo(Constants.clawOpen);
        while (myRobot.rotateMotor.isBusy()) {
        }

        do {
            myRobot.setSlideServo(Constants.autoSlideCycle);
            myRobot.setLiftMotor(1, 4 * Constants.autoLiftCone);
            toTargetDistance(Constants.autoDistCycle, false, 0.5, 5000, 15, 0.25);
        }
        while ((dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, autoTurnTall, autoSlideTall) == -1)
                && (30000 - (runtime.milliseconds() - overallStart) > parkBuffer + 5000) && opModeIsActive());
//        myRobot.setLiftMotor(1, 4 * Constants.autoLiftCone);
//        myRobot.setSlideServo(Constants.autoSlideCycle);
//        while (myRobot.liftMotor.isBusy()) {}
//        toTargetDistance(Constants.autoDistCycle, true, 0.3, 5000, 5, 0.5);
//        dropCone(Constants.liftHigh, 4 * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall);


        for (int i = 3; i >= 0 && ((30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + Constants.cycleTimeHigh); i--) {
            // Reset to stack
            resetCycle(i * Constants.autoLiftCone, Constants.rot90RLong, Constants.autoSlideCycle - Constants.slideCycleBack);
//            myRobot.setSlideServo(Constants.autoSlideCycle);
//            sleep((long) (Constants.slideCycleBack * Constants.slideWaitARatio));
            // Drop cone
            while ((dropCone(Constants.liftHigh, i * Constants.autoLiftCone + Constants.coneDodge, autoTurnTall, autoSlideTall) == -1)
                    && ((30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + 5000)) {
                myRobot.setSlideServo(Constants.autoSlideCycle - Constants.slideCycleBack);
                myRobot.setClawServo(Constants.clawOpen);
                myRobot.setLiftMotor(1, i * Constants.autoLiftCone);
                while (myRobot.liftMotor.isBusy()) {
                }
                myRobot.setSlideServo(Constants.autoSlideCycle);
                sleep((long) (Constants.slideCycleBack * Constants.slideWaitARatio));
            }
//            dropCone(Constants.liftHigh, i * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall);
        }

        // Reset to front
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetFront();
        while (myRobot.rotateMotor.isBusy()) {
        }
//
//        // Park
        park(false, signal);
        myRobot.setSlideServo(Constants.slideIn);
        myRobot.setLiftMotor(1, 0);
        while (myRobot.liftMotor.isBusy()) {
        }
        myRobot.setSlideServo(Constants.slideIn);
    }

    public abstract boolean isRed();

    public abstract double getAutoSlideFirstTall();

    public abstract double getAutoSlideTall();

    public abstract int getAutoTurnFirstTall();

    public abstract int getAutoTurnTall();
}
