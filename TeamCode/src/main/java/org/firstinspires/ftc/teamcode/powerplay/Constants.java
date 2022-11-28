package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Constants {
    // PowerPlay Stuff
    public static double slideIn = 0.975;
//    public static double slideSpin = 0.395;
    public static double slideOut = 0.5;
    public static double slideMed = 0.7;
    public static double autoSlideTurn = 0.6;
    public static double autoSlideOut = 0.5;
    public static double clawOpen = 0.8;
    public static double clawClose = 0.975;
    public static double slideSpeed = -0.0125;

    public static double liftUpRatio = 1;
    public static double liftDownRatio = 0.8;
    public static double setRotateMultiplier = 0.7;
    public static int rotMotorPosPerDegree = 13;

    // Lift Positions
    public static int liftHigh = -3050;
    public static int liftMed = -2200;
    public static int liftLow = -1400;
    public static int liftFloor = -300;
    public static int liftSpin = -400;
    public static int liftMax = -3100;
    public static int liftMin = 0;
    public static double liftMinPow = 0.1;
    public static double liftkP = 10;
    public static int liftTolerance = 9;

    // Rotation Positions
    public static int rotRLimit = 4270;
    public static int rotMax = 8750;
    public static double rotMin = 0.05;
    public static double rotkP = 25;
    public static int rot180L = -2125;
    public static int rot180R = 2125;
    public static int rot90L = -1075;
    public static int rot90R = 1075;
    public static int rot45L = -300;
    public static int rot45R = 300;
    public static int rotDiagBackR = 1875;
    public static int rotDiagBackL = -1875;
    public static int rotTolerance = 25;
    public static int autoRotTolerance = 125;

    // Drive motor
    public static final double TICKS_PER_REV = 751.8;
    public static final double CIRCUMFERENCE_IN_INCHES = 100 / 25.4 * Math.PI;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / CIRCUMFERENCE_IN_INCHES;
    public static double moveSpeed = 1;
    public static double rotSpeed = 1;

    // Autonomous turn PID
    public static double kR = 0.084; // PID turn kR
    public static double kD = 0.0072; // PID turn kD

    public static double tkR = 0.03;
    public static double tskR = 0.03;

    public static double ColorThresh = 0.6;
    public static double ColorStripAlignmentSpeed = 0.4;
    public static int ColorStripAlignmentDelay = 5000;
    public static int gain = 50;

    public static int imgWidth = 1920;
    public static int imgHeight = 1080;
    public static int changeThresh = 128;
    public static int negChangeThresh = -128;
    public static int colorThresh = 200;
    public static int tlx = 0; // Top left x for rectangle
    public static int tly = 0; // Top left y for rectangle
    public static int brx = 100; // Bottom right x for rectangle
    public static int bry = 100; // Bottom right y for rectangle

    // Camera stuff
    public static int leftBoundary = 890; // left side of detection zone
    public static int rightBoundary = 990; // right side of detection zone
    public static int middleLine = 735; // detection line y coordinate

    public static Scalar lowRed1 = new Scalar(0, 70, 50);
    public static Scalar highRed1 = new Scalar(10, 255, 255);
    public static Scalar lowRed2 = new Scalar(170, 70, 50);
    public static Scalar highRed2 = new Scalar(180, 255, 255);

    public static Scalar lowBlue = new Scalar(110, 70, 50);
    public static Scalar highBlue = new Scalar(130, 255, 255);

    public static int maskChangeThresh = 1;
    public static int negMaskChangeThresh = -1;
    public static double slopeThresh = 100.0; // 100
    public static double negSlopeThresh = -100.0; // 100
    public static boolean isDetectRed = true; // False: detect Blue
    public static boolean isDetectLines = true; // False: detect QR code

    public static final int automationDelay = 48;
    public static final int buttonDelay = 36;

    public static double straightTestDist = 75;
    public static double straightTestPow = 0.7;
    public static double liftSpeed = 0.1;

    public static boolean debugMode = true; // Change it to FALSE before the competition!!!!

    //tensorflow constants
    public static double magnitude = 1;
    public static double aspectRatio = 16.0/9.0;
    public static float minResultConfidence = 0.50f;
    public static float moveForward = 7.0f;
}
