package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class RightMedAuto extends AutonomousMethods {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    OpenCvWebcam webcam;
    SignalDetectionPipeline signalDetectionPipeline;
    ConeAlignmentPipelineV2 coneAlignPipeline;
    int signal = 3;
    double cameraError = 100;

    double autoSlideFirstMed = getAutoSlideFirstMed();
    double autoSlideMed = getAutoSlideMed();
    int autoTurnFirstMed = getAutoTurnFirstMed();
    int autoTurnMed = getAutoTurnMed();

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
        if (signal == 1) {
            parkBuffer += 1500;
        }
        webcam.setPipeline(coneAlignPipeline);

//        {
//            multitaskMovement(0, Constants.liftMed, Constants.rot90R, Constants.slideOpt, -48, 0.6);
//            encoderTurn(0, 0.3, 1);
//        }
        {
            myRobot.setLiftMotor(1, Constants.liftMed);
            myRobot.setRotateMotor(0.75, Constants.rot90R);
            encoderStraightDrive(-48, Constants.autoDriveSpeed);
//            encoderStraightDrive(-48, 0.5);
            encoderTurn(0, 0.5, 1);
        }
        while (myRobot.rotateMotor.isBusy()) {
        }
        sleep(1000);
        cameraError = (coneAlignPipeline.getMiddle() - 960) * Constants.alignRatio;
        encoderStraightDrive(cameraError, 0.25);

        myRobot.setRotateMotor(0.5, autoTurnFirstMed);
        while (myRobot.rotateMotor.isBusy()) {
        }
        encoderTurn(0, 0.5, 1);
        myRobot.setSlideServo(autoSlideFirstMed);
        sleep(600);
        myRobot.setLiftMotor(1, Constants.liftMed + 100);
        sleep(150);
        myRobot.setClawServo(Constants.clawFurtherOpen);
        myRobot.setSlideServo(Constants.autoSlideCycle - Constants.slideCycleBack);
        sleep(Constants.clawOpenDelay);
        sleep(100);
        myRobot.setRotateMotor(0.75, Constants.rot90L);
        int currentRPosition, stage = 1;
        int startRPosition = myRobot.getRotationMotorPosition();
        while (opModeIsActive() && stage <= 3) {
            currentRPosition = myRobot.getRotationMotorPosition();
            switch (stage) {
                case 1:
                    if (Math.abs(currentRPosition - startRPosition) >= Constants.junctionDodge) {
                        stage = 2;
                    }
                    break;
                case 2:
                    myRobot.setLiftMotor(1, Constants.liftLow);
                    myRobot.setClawServo(Constants.clawOpen);
                    stage = 3;
                    break;
                case 3:
                    if (Math.abs(currentRPosition - startRPosition) >= Constants.lowJuncDodgeFirst) {
                        myRobot.setLiftMotor(1, 4 * Constants.autoLiftCone);
                        stage = 4;
                    }
            }
        }
        while (myRobot.rotateMotor.isBusy() || myRobot.liftMotor.isBusy()){
        }

        encoderTurn(0, 0.5, 1);

        do {
            myRobot.setSlideServo(Constants.autoSlideCycle);
            myRobot.setLiftMotor(1, 4 * Constants.autoLiftCone);
            toTargetDistance(Constants.autoDistCycle, true, 0.5, 5000, 15, 0.25);
        }
        while ((dropCone(Constants.liftMed, 4 * Constants.autoLiftCone + Constants.coneDodge, autoTurnMed, autoSlideMed) == -1)
                && (30000 - (runtime.milliseconds() - overallStart) > parkBuffer + 5000) && opModeIsActive());
//        myRobot.setLiftMotor(1, 4 * Constants.autoLiftCone);
//        myRobot.setSlideServo(Constants.autoSlideCycle);
//        while (myRobot.liftMotor.isBusy()) {}
//        toTargetDistance(Constants.autoDistCycle, true, 0.3, 5000, 5, 0.5);
//        dropCone(Constants.liftMed, 4 * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall);

        for (int i = 3; i >= 0 && ((30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + Constants.cycleTimeMed); i--) {
            // Reset to stack
            resetMedCycle(i * Constants.autoLiftCone, Constants.rot90L, Constants.autoSlideCycle - Constants.slideCycleBack);
//                myRobot.setSlideServo(Constants.autoSlideCycle);
////            while (myRobot.getClawDistance() > Constants.autoDistCycle + 0.5) {
////            }
//                sleep(100);

            // Drop cone
            while ((dropCone(Constants.liftMed, i * Constants.autoLiftCone + Constants.coneDodge, autoTurnMed, autoSlideMed) == -1)
                    && ((30000 - (runtime.milliseconds() - overallStart)) > parkBuffer + 5000)) {
                myRobot.setSlideServo(Constants.autoSlideCycle - Constants.slideCycleBack);
                myRobot.setClawServo(Constants.clawOpen);
                myRobot.setLiftMotor(1, i * Constants.autoLiftCone);
                while (myRobot.rotateMotor.isBusy() || myRobot.liftMotor.isBusy()) {
                }
                myRobot.setSlideServo(Constants.autoSlideCycle);
                sleep(250);
            }
//            dropCone(Constants.liftMed, i * Constants.autoLiftCone + Constants.coneDodge, Constants.autoTurnTall, Constants.autoSlideTall);
        }

        // Reset to front
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetFront();
        while (myRobot.rotateMotor.isBusy()) {
        }
//
//        // Park
        park(true, signal);
        myRobot.setSlideServo(Constants.slideIn);
        myRobot.setLiftMotor(1, 0);
        while (myRobot.liftMotor.isBusy()) {
        }
        myRobot.setSlideServo(Constants.slideIn);
    }

    public abstract boolean isRed();

    public abstract double getAutoSlideFirstMed();

    public abstract double getAutoSlideMed();

    public abstract int getAutoTurnFirstMed();

    public abstract int getAutoTurnMed();
}
