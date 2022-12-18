package org.firstinspires.ftc.teamcode.test;


import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.powerplay.Constants;
import org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;

@TeleOp(name = "Webcam Color Test")
@Disabled
public class OpenCVExampleOpMode extends OpMode {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    SignalDetectionPipeline pipeline;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SignalDetectionPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
//        telemetry.addData("Image Analysis:",pipeline.getAnalysis());
//        telemetry.addData("Image number:", pipeline.getRectA_Analysis());

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        telemetry.addData("Image number:", pipeline.getCounter());
        telemetry.update();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Image number:", pipeline.getCounter());
        dashboard.sendTelemetryPacket(pack);
    }


}

class SamplePipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    //    Mat RectA_Y = new Mat();
//    int avg;
//    int avgA;
    int counter;
//    static final int STREAM_WIDTH = 1920; // modify for your camera
//    static final int STREAM_HEIGHT = 1080; // modify for your camera
//
//
//    static final int WidthRectA = 130;
//    static final int HeightRectA = 110;


//    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2 + 300, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);
//    Point RectATLCorner = new Point(
//            RectATopLeftAnchor.x,
//            RectATopLeftAnchor.y);
//    Point RectABRCorner = new Point(
//            RectATopLeftAnchor.x + WidthRectA,
//            RectATopLeftAnchor.y + HeightRectA);




    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
    }

    /*
     * This function takes the RGB frame, calculate historam,
     * detect the dominate color,
     * and return the dominate color.
     */
//    int detectColor(Mat input) {
//        int[] histR = new int[256];
//        int[] histG = new int[256];
//        int[] histB = new int[256];
//
//        int x1 = 0;
//        int x2 = 400;
//        int y1 = 0;
//        int y2 = 400;
//        int width = x2 - x1;
//        int height = y2 - y1;
//        double sens = 0.95;
//        // Rect r = new Rect(10, 10, 100, 100);
//        // Mat roiImage = input.submat(r);
//
//        for (int i = y1; i < y2; i++) {
//            for (int j = x1; j < x2; j++) {
//                if (input.at(Byte.class, i, j).getV4c().get_0() < 0) {
//                    histR[0]++;
//                } else if (input.at(Byte.class, i, j).getV4c().get_0() > 255) {
//                    histR[255]++;
//                } else {
//                    histR[input.at(Byte.class, i, j).getV4c().get_0()]++;
//                }
//
//                if (input.at(Byte.class, i, j).getV4c().get_1() < 0) {
//                    histG[0]++;
//                } else if (input.at(Byte.class, i, j).getV4c().get_1() > 255) {
//                    histG[255]++;
//                } else {
//                    histG[input.at(Byte.class, i, j).getV4c().get_1()]++;
//                }
//
//                if (input.at(Byte.class, i, j).getV4c().get_2() < 0) {
//                    histB[0]++;
//                } else if (input.at(Byte.class, i, j).getV4c().get_2() > 255) {
//                    histB[255]++;
//                } else {
//                    histB[input.at(Byte.class, i, j).getV4c().get_2()]++;
//                }
//            }
//        }
//        if (((double) (histR[0] + histR[1] + histR[2] + histR[3] + histR[4] + histR[5]) >= (sens * height * width)) &&
//            ((double) (histG[255] + histG[254] + histG[253] + histG[252] + histG[251] + histG[250]) >= (sens * height * width)) &&
//                ((double) (histB[0] + histB[1] + histB[2] + histB[3] + histB[4] + histB[5]) >= (sens * height * width))) {
//            return 0; // Green
//        }
//        else if (((double) (histR[255] + histR[254] + histR[253] + histR[252] + histR[251] + histR[250]) >= (sens * height * width)) &&
//                ((double) (histG[255] + histG[254] + histG[253] + histG[252] + histG[251] + histG[250]) >= (sens * height * width)) &&
//                ((double) (histB[0] + histB[1] + histB[2] + histB[3] + histB[4] + histB[5]) >= (sens * height * width))) {
//            return 1; // Yellow
//        }
//        else if (((double) (histR[0] + histR[1] + histR[2] + histR[3] + histR[4] + histR[5]) >= (sens * height * width)) &&
//                ((double) (histG[255] + histG[254] + histG[253] + histG[252] + histG[251] + histG[250]) >= (sens * height * width)) &&
//                ((double) (histB[255] + histB[254] + histB[253] + histB[252] + histB[251] + histB[250]) >= (sens * height * width))) {
//            return 2; // Cyan
//        }
//        else {
//            return -1; // No dominant color
//        }
//    }

    int detectBlackWhite(Mat input) {
        int counter = 0;
        int x1 = Constants.leftBoundary;
        int x2 = Constants.rightBoundary;
        int midy = Constants.middleLine;
        for (int j = x1 + 1; j < x2; j++) {
//            if (input.at(Byte.class, (y2 + y1) / 2, j).getV4c().get_0() - input.at(Byte.class, (y2 + y1) / 2, j - 1).getV4c().get_0() > Constants.changeThresh) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > Constants.changeThresh) {
                counter += 1;
            }
        }
        return counter;
    }



    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
//        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
    }

    @Override
    public Mat processFrame(Mat input) {
        //int detectedColor = detectColor(input);

        inputToY(input);
        Y.copyTo(input);
//        System.out.println("processing requested");
//        avg = (int) Core.mean(Y).val[0];
//        avgA = (int) Core.mean(RectA_Y).val[0];
        counter = detectBlackWhite(input);
        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!


        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                new Point(Constants.leftBoundary,Constants.middleLine - 20), // First point which defines the rectangle
                new Point(Constants.rightBoundary,Constants.middleLine + 20), // Second point which defines the rectangle
                new Scalar(0,0,255), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.line(input, new Point(Constants.leftBoundary, Constants.middleLine), new Point(Constants.rightBoundary, Constants.middleLine), new Scalar(0, 0, 255), 3);
        return input;
    }

//    public int getAnalysis() {
//        return avg;
//    }
//    public int getRectA_Analysis() {
//        return avgA;
//    }

    public int getCounter(){
        return counter;
    }

}
