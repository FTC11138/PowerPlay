package org.firstinspires.ftc.teamcode.test;


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

@TeleOp(name = "Webcam Cone Alignment")
@Disabled
public class ConeAlignment extends OpMode {
    static final int STREAM_WIDTH = Constants.imgWidth; // modify for your camera
    static final int STREAM_HEIGHT = Constants.imgHeight; // modify for your camera
    OpenCvWebcam webcam;
    ConeAlignmentPipeline pipeline;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ConeAlignmentPipeline();
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
        telemetry.addData("Slope:", pipeline.getSlope());
        telemetry.update();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Slope of middle line:", pipeline.getSlope());
        pack.put("averageX1:", pipeline.getAverageX1());
        pack.put("averageX2:", pipeline.getAverageX2());
        pack.put("Cone middle X:", pipeline.getConeMiddleX());
        dashboard.sendTelemetryPacket(pack);
    }
}

class ConeAlignmentPipeline extends OpenCvPipeline {
    Mat HSV = new Mat();
    static final int STREAM_WIDTH = Constants.imgWidth; // modify for your camera
    static final int STREAM_HEIGHT = Constants.imgHeight; // modify for your camera
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    double slope = 0;
    int averageX1 = 0;
    int averageX2 = 0;
    int coneMiddleX = 0;

    int getAverageX1() {
        return averageX1;
    }
    int getAverageX2() {
        return averageX2;
    }
    double getSlope() {
        return slope;
    }
    int getConeMiddleX() {return coneMiddleX;}


    int calConeMiddleX(Mat input) {
        double slope = 0.0;

        // (x1, y1)   (x2, y1)
        // (x3, y2)   (x4, y2)

        int x1 = 0;
        int x2 = 0;
        int x3 = 0;
        int x4 = 0;
        int y1 = 0;
        int y2 = 0;
        for (int y = Constants.imgHeight - 1; y >= 20; y -= 20) {
            y1 = y;
            y2 = y - 20;
            for (int j = 1; j < Constants.imgWidth; j++) {
                if ((input.at(Byte.class, y1, j).getV().byteValue() - input.at(Byte.class, y1, j - 1).getV().byteValue()) > Constants.maskChangeThresh) {
                    x1 = j;
                }
                if ((input.at(Byte.class, y1, j).getV().byteValue() - input.at(Byte.class, y1, j - 1).getV().byteValue()) < Constants.negMaskChangeThresh) {
                    x2 = j;
                }
                if ((input.at(Byte.class, y2, j).getV().byteValue() - input.at(Byte.class, y2, j - 1).getV().byteValue()) > Constants.maskChangeThresh) {
                    x3 = j;
                }
                if ((input.at(Byte.class, y2, j).getV().byteValue() - input.at(Byte.class, y2, j - 1).getV().byteValue()) < Constants.negMaskChangeThresh) {
                    x4 = j;
                }
            }
            if ((x3 + x4) == (x1 + x2)) {
                slope = Double.POSITIVE_INFINITY;
            } else {
                slope = 40.0 / ((double)(x1 + x2) - (x3 + x4)); // y2 - y1 = 20
            }

            averageX1 = (x1 + x2) / 2;
            averageX2 = (x3 + x4) / 2;

            if (slope > Constants.slopeThresh || slope < Constants.negSlopeThresh) { // > 100 or < -100
                return (x1 + x2 + x3 + x4) / 4;
            }
        }
        return -1;
    }

    double lineSlope(Mat input) {
        double slope = 0.0;
        int x1 = 0;
        int x2 = 0;
        int x3 = 0;
        int x4 = 0;
        int y1 = Constants.imgHeight - 200;
        int y2 = Constants.imgHeight - 250;
        for (int j = 1; j < Constants.imgWidth; j++) {
//            //if (input.at(Byte.class, (y2 + y1) / 2, j).getV4c().get_0() - input.at(Byte.class, (y2 + y1) / 2, j - 1).getV4c().get_0() > Constants.changeThresh) {
            if ((input.at(Byte.class, y1, j).getV().byteValue() - input.at(Byte.class, y1, j - 1).getV().byteValue()) > Constants.changeThresh) {
                x1 = j;
            }
            if ((input.at(Byte.class, y1, j).getV().byteValue() - input.at(Byte.class, y1, j - 1).getV().byteValue()) < Constants.negChangeThresh) {
                x2 = j;
            }
            if ((input.at(Byte.class, y2, j).getV().byteValue() - input.at(Byte.class, y2, j - 1).getV().byteValue()) > Constants.changeThresh) {
                x3 = j;
            }
            if ((input.at(Byte.class, y2, j).getV().byteValue() - input.at(Byte.class, y2, j - 1).getV().byteValue()) < Constants.negChangeThresh) {
                x4 = j;
            }
        }
        if ((x3 + x4) == (x1 + x2)) {
            slope = Double.POSITIVE_INFINITY;
        } else {
            slope = (double) 100 / ((x1 + x2) - (x3 + x4));
        }

        averageX1 = (x1 + x2) / 2;
        averageX2 = (x3 + x4) / 2;
        return slope;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
//        Cr.copyTo(input);
        // Detect Red
        if (Constants.isDetectRed) {
            Core.inRange(HSV, new Scalar(0, 50, 50), new Scalar(5, 255, 255), mask1); // low luma from 75 to 50
            Core.inRange(HSV, new Scalar(175, 50, 50), new Scalar(180, 255, 255), mask2); // low luma from 75 to 50
            Core.bitwise_or(mask1, mask2, input);
        } else { // Detect Blue
            Core.inRange(HSV, new Scalar(100, 50, 50), new Scalar(140   , 255, 255), input); // low luma from 75 to 50
            Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
        }
        // Core.inRange(HSV, new Scalar(175, 75, 75), new Scalar(180, 255, 255), mask2);
        // Core.bitwise_or(mask1, mask2, input);
        // Imgproc.GaussianBlur(input, input, new Size(7, 5), 0);
        //Imgproc.GaussianBlur(input, input, new Size(21, 21), 0);

        Photo.edgePreservingFilter(input, input);
        // Photo.edgePreservingFilter(input, input, 0, 50);
        //Imgproc.GaussianBlur(input, input, new Size(31, 31), 0);
        // Imgproc.addWeighted()

        HSV.release(); // don't leak memory!
        mask1.release();
        mask2.release();

        // Imgproc.line(input, new Point(getAverageX1(), 200), new Point(getAverageX2(), 250), new Scalar(0), 3);

        //slope = lineSlope(input);

        coneMiddleX = calConeMiddleX(input);

        /*Imgproc.line(input, new Point(960, 0),
                new Point(960, Constants.imgHeight),
                new Scalar(255, 255, 0));*/

        Imgproc.line(input, new Point(coneMiddleX, 0),
                new Point(coneMiddleX, Constants.imgHeight),
                new Scalar(0, 0, 0)); // Black line

        Imgproc.line(input, new Point(getAverageX1(), 0),
                new Point(getAverageX1(), Constants.imgHeight),
                new Scalar(255, 255, 255)); // White line

        Imgproc.line(input, new Point(getAverageX2(), 0),
                new Point(getAverageX2(), Constants.imgHeight),
                new Scalar(255, 255, 255)); // White line

//                input, // Buffer to draw on
//                new Point(arr[2], 0), // First point which defines the rectangle
//                new Point(arr[2], STREAM_HEIGHT), // Second point which defines the rectangle
//                new Scalar(0, 0, 255), // The color the rectangle is drawn in
//                5); // Thickness of the rectangle lines

//        Imgproc.rectangle( // rings
//                input, // Buffer to draw on
//                new Point(middle, 0), // First point which defines the rectangle
//                new Point(middle, STREAM_HEIGHT), // Second point which defines the rectangle
//                new Scalar(0, 0, 255), // The color the rectangle is drawn in
//                5); // Thickness of the rectangle lines
//        Imgproc.rectangle( // rings
//                input, // Buffer to draw on
//                new Point(arr[0], 0), // First point which defines the rectangle
//                new Point(arr[0], STREAM_HEIGHT), // Second point which defines the rectangle
//                new Scalar(0, 0, 255), // The color the rectangle is drawn in
//                5); // Thickness of the rectangle lines
//        Imgproc.rectangle( // rings
//                input, // Buffer to draw on
//                new Point(arr[2], 0), // First point which defines the rectangle
//                new Point(arr[2], STREAM_HEIGHT), // Second point which defines the rectangle
//                new Scalar(0, 0, 255), // The color the rectangle is drawn in
//                5); // Thickness of the rectangle lines
        return input;
    }
}
