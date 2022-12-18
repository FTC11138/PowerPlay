//package org.firstinspires.ftc.teamcode.powerplay;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "LeftAuto", group = "Linear Opmode", preselectTeleOp = "TeleOp")
//@Disabled
//public class LeftAuto extends AutonomousMethods{
//
//    static final int STREAM_WIDTH = 1920; // modify for your camera
//    static final int STREAM_HEIGHT = 1080; // modify for your camera
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private Attachments robot = new Attachments();
//
//    OpenCvWebcam webcam;
//    org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline signalDetectionPipeline;
//    int signal = 1;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    TelemetryPacket packet = new TelemetryPacket();
//
//    @Override
//    public void runOpMode() throws InterruptedException {telemetry.update();
//        // Initialize all the parts of the robot
//        initializeAuto(hardwareMap, telemetry);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        signalDetectionPipeline = new org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline();
//        webcam.setPipeline(signalDetectionPipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                // Start showing the camera
//                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//
//        myRobot.setClawServo(Constants.clawClose);
//        while (!isStarted()) {
//            signal = signalDetectionPipeline.getCounter();
//            telemetry.addData("Signal", signal);
//            telemetry.update();
//        }
//        runtime.reset();
//
//        myRobot.setLiftMotor(1, Constants.liftHigh);
//        myRobot.setRotateMotor(0.5, -Constants.autoTurnFirstTall);
//
//        encoderStraightDrive(33, 0.5);
//
//        myRobot.setSlideServo(Constants.autoSlideFirstTall);
//        sleep(1000);
//        myRobot.setLiftMotor(0.3, Constants.liftHigh+200);
//        sleep(1000);
//        myRobot.setClawServo(Constants.clawOpen);
//        sleep(1000);
//        myRobot.setSlideServo(Constants.slideIn);
//        sleep(500);
//        myRobot.setRotateMotor(0.5, 0);
//        myRobot.setLiftMotor(1, 0);
//        encoderStraightDrive(-15, 0.5);
//
//        if (signal == 3) {
//            encoderStrafeDriveInchesRight(20, 0.5);
//        } else if (signal == 1) {
//            encoderStrafeDriveInchesRight(-20, 0.5);
//        }
//    }
//}
