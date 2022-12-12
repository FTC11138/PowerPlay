package org.firstinspires.ftc.teamcode.test;


import static org.opencv.core.CvType.CV_32F;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.powerplay.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.photo.Photo;

@TeleOp(name = "Webcam Cone Alignment V2")
public class ConeAlignmentV2 extends OpMode {
    static final int STREAM_WIDTH = Constants.imgWidth; // modify for your camera
    static final int STREAM_HEIGHT = Constants.imgHeight; // modify for your camera

    OpenCvWebcam webcam;
    ConeAlignmentPipelineV2 pipeline;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ConeAlignmentPipelineV2();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
//        telemetry.addData("Slope:", pipeline.getSlope());
//        telemetry.update();
//
//        TelemetryPacket pack = new TelemetryPacket();
//        pack.put("Slope of middle line:", pipeline.getSlope());
//        pack.put("averageX1:", pipeline.getAverageX1());
//        pack.put("averageX2:", pipeline.getAverageX2());
//        pack.put("Cone middle X:", pipeline.getConeMiddleX());
//        dashboard.sendTelemetryPacket(pack);
    }
}

class ConeAlignmentPipelineV2 extends OpenCvPipeline {
    Mat yCrCb = new Mat();
    final Mat kernel = Mat.ones(5, 5, CV_32F);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Mat mask = new Mat();
//        Cr.copyTo(input);
        // Detect Red
        if (Constants.isDetectRed) {
            Core.inRange(yCrCb, new Scalar(0, 175, 0), new Scalar(255, 255, 255), mask); // low luma from 75 to 50
        } else { // Detect Blue
            Core.inRange(yCrCb, new Scalar(0, 0, 175), new Scalar(255, 255, 255), mask); // low luma from 75 to 50
        }
        Core.bitwise_and(input, input, input, mask);
        yCrCb.release(); // don't leak memory!
        mask.release();

        Imgproc.morphologyEx(input, input, Imgproc.MORPH_OPEN, kernel);
//        Imgproc.findContours(input, );
//        Imgproc.findContours();

        return input;
    }
}
