package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class BlueAlignment extends AutonomousMethods {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera

    OpenCvWebcam webcam;
    ConeAlignmentPipelineV2 coneAlignPipeline;
    double cameraError = 100;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize all the parts of the robot
        initializeAuto(hardwareMap, telemetry);

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        coneAlignPipeline = new ConeAlignmentPipelineV2(false);
        webcam.setPipeline(coneAlignPipeline);
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

        telemetry.addData("Finished initialization", "yay");
        telemetry.update();
        waitForStart();
        runtime.reset();

        cameraError = (coneAlignPipeline.getMiddle() - 960) * Constants.alignRatio;
        encoderStraightDrive(cameraError, 0.3);
        sleep(2500);
    }
}
