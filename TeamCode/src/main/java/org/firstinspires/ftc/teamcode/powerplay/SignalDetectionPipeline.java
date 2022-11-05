package org.firstinspires.ftc.teamcode.powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class SignalDetectionPipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    int counter;

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

    int detectBlackWhite(Mat input) {
        int edgeCounter = 0;
        int x1 = Constants.leftBoundary;
        int x2 = Constants.rightBoundary;
        int midy = Constants.middleLine;
        for (int j = x1 + 1; j < x2; j++) {
//            if (input.at(Byte.class, (y2 + y1) / 2, j).getV4c().get_0() - input.at(Byte.class, (y2 + y1) / 2, j - 1).getV4c().get_0() > Constants.changeThresh) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > Constants.changeThresh) {
                edgeCounter += 1;
            }
        }
        if (edgeCounter == 0) { // clip
            edgeCounter = 1;
        } else if (edgeCounter > 3) {
            edgeCounter = 3;
        }
        return edgeCounter;
    }

    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToY(input);
        Y.copyTo(input);
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

    public int getCounter() {
        return counter;
    }

}
