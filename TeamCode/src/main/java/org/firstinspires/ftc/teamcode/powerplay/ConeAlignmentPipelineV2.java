package org.firstinspires.ftc.teamcode.powerplay;

import static org.opencv.core.CvType.CV_32F;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

class ConeAlignmentPipelineV2 extends OpenCvPipeline {
    Mat yCrCb = new Mat();
    final Mat kernel = Mat.ones(Constants.kernelSize, Constants.kernelSize, CV_32F);
    int middle;
    boolean isDetectRed;

    public ConeAlignmentPipelineV2(boolean isDetectRed) {
        this.isDetectRed = isDetectRed;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Mat mask = new Mat();
        // Detect Red
        if (isDetectRed) {
            Core.inRange(yCrCb, new Scalar(0, Constants.colorThresh, 0), new Scalar(255, 255, 255), mask); // low luma from 75 to 50
        } else { // Detect Blue
            Core.inRange(yCrCb, new Scalar(0, 0, Constants.colorThresh), new Scalar(255, 255, 255), mask); // low luma from 75 to 50
        }
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        if (!Constants.debugMode) {
            mask.copyTo(input);
        }
//        else {
//            Core.bitwise_or(input, input, input, mask);
//        }
        detectCone(mask);
        yCrCb.release(); // don't leak memory!
        mask.release();
        Imgproc.rectangle(input, new Point(Constants.leftAlignStart, Constants.topAlignLine),
                new Point(Constants.rightAlignEnd, Constants.bottomAlignLine),
                new Scalar(255, 0, 0), 2);
        Imgproc.line(input, new Point(middle, 0), new Point(middle, 1080),
                new Scalar(0, 0, 0), 3);
//        Imgproc.rectangle(input, new Point(Constants.test1, Constants.test2),
//                new Point(Constants.test1, Constants.test2),
//                new Scalar(0, 255, 0), 5);

        return input;
    }

    void detectCone(Mat input) {
        int topStart = Constants.leftAlignStart, midStart = Constants.leftAlignStart, botStart = Constants.leftAlignStart;
        int topEnd = Constants.rightAlignEnd, midEnd = Constants.rightAlignEnd, botEnd = Constants.rightAlignEnd;
        int opp;

        for (int i = Constants.leftAlignStart; i < Constants.rightAlignEnd; i++) {
            // Top left bound
            if ((topStart == Constants.leftAlignStart) && (input.get(Constants.topAlignLine, i)[0]) > 250) {
                topStart = i;
            }
            // Middle left bound
            if ((midStart == Constants.leftAlignStart) && (input.get(Constants.midAlignLine, i)[0]) > 250) {
                midStart = i;
            }
            // Bottom left bound
            if ((botStart == Constants.leftAlignStart) && (input.get(Constants.bottomAlignLine, i)[0]) > 250) {
                botStart = i;
            }

            opp = Constants.rightAlignEnd - (i - Constants.leftAlignStart) - 1;
            // Top right bound
            if ((topEnd == Constants.rightAlignEnd) && (input.get(Constants.topAlignLine, opp)[0]) > 250) {
                topEnd = opp;
            }
            // Mid right bound
            if ((midEnd == Constants.rightAlignEnd) && (input.get(Constants.midAlignLine, opp)[0]) > 250) {
                midEnd = opp;
            }
            // Top right bound
            if ((botEnd == Constants.rightAlignEnd) && (input.get(Constants.bottomAlignLine, opp)[0]) > 250) {
                botEnd = opp;
            }

            if ((botStart != Constants.leftAlignStart) && (midStart != Constants.leftAlignStart) && (topStart != Constants.leftAlignStart) &&
                    (botEnd != Constants.rightAlignEnd) && (midEnd != Constants.rightAlignEnd) && (topEnd != Constants.rightAlignEnd)) {
                break;
            }
        }
        int[] diffs = new int[]{topStart + (topEnd - topStart) / 2, midStart + (midEnd - midStart) / 2, botStart + (botEnd - botStart) / 2};
//        Log.d("AHHHHHHH topEnd", Constants.topAlignLine + ", " + topEnd);
//        Log.d("AHHHHHHH topStart", Constants.topAlignLine + ", " + topStart);
//        Log.d("AHHHHHHH midEnd", Constants.midAlignLine + ", " + midEnd);
//        Log.d("AHHHHHHH midStart", Constants.midAlignLine + ", " + midStart);
//        Log.d("AHHHHHHH botEnd", Constants.bottomAlignLine + ", " + botEnd);
//        Log.d("AHHHHHHH botStart", Constants.bottomAlignLine + ", " + botStart);
        Log.d("AHHHHHHH middle", "" + middle);
        Arrays.sort(diffs);
        middle = diffs[1];
    }

    public int getMiddle() {
        return middle;
    }
}
