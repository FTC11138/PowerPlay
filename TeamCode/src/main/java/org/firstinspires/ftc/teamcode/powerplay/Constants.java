package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Constants {
    // PowerPlay Stuff
    public static double slideIn = 1;
    public static double slideResting = 0.575;
    public static double slideSpin = 0.395;
    public static double slideOut = 0.1;
    public static double autoSlideOut = 0.35;
    public static double clawOpen = 0.73;
    public static double clawClose = 0.55;
    public static double slideSpeed = 0.0125;

    public static double liftUpRatio = 1;
    public static double liftDownRatio = 0.5;
    public static double setRotateMultiplier = 1;
    public static int rotMotorPosPerDegree = 13;

    public static int liftOff = 108;
    public static int liftTop = -3306;

    // Lift Positions
    public static int liftHigh = -3050;
    public static int liftMed = -2200;
    public static int liftLow = -1400;
    public static int liftFloor = -300;
    public static int liftSpin = -325;
    public static int liftMax = -3100;
    public static double liftMin = 0;
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
    public static int rotDiagBackR = 1875;
    public static int rotDiagBackL = -1875;
    public static int rotTolerance = 25;

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
    public static double setVerticalMinSpeed = 0.18;
    public static double setVerticalMaxSpeed = 1;
    public static double setHorizontalMinSpeed = 0.3;
    public static double setHorizontalMaxSpeed = 1;
    public static double setDiagonalMinSpeed = 0.3;
    public static double setDiagonalMaxSpeed = 1;
    public static double setReadjustMinSpeed = 0.12;
    public static double setReadjustMaxSpeed = 0.;


    public static double setHorizontalDisCap = 24;
    public static double setVerticalDisCap = 36;
    public static double setReadjustDisCap = 24;
    public static double vertHorRatio = 1.22348112;

    public static double ColorThresh = 0.9;
    public static double ColorStripAlignmentSpeed = 0.4;
    public static int ColorStripAlignmentDelay = 5000;
    public static int gain = 10;

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
    public static int leftBoundary = 740; // left side of detection zone
    public static int rightBoundary = 810; // right side of detection zone
    public static int middleLine = 750; // detection line y coordinate

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

    public static final int automationDelay = 96;
    public static final int buttonDelay = 18;

    public static boolean debugMode = true; // Change it to FALSE before the competition!!!!
}
