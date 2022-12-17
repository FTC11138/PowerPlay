package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Constants {
    // PowerPlay Stuff
    public static double parkBuffer = 2500;

    public static double slideIn = 0.83;
    public static double slideOut = 0.15;
    public static double slideOpt = 0.64; // Optimal slide position for rotation
    public static double slideMed = 0.35;
    public static double autoSlideTurn = 0.45;
    public static double autoSlideFirstTall = 0.567;
    public static double autoSlideTall = 0.35;
    public static double autoSlideOut = 0.15;
    public static double autoSlideCycle = 0.28;
    public static double slideCycleBack = 0.1;
    public static double autoDistCycle = 1.5; // inches to be away from cone
    public static double clawOpen = 0.4;
    public static double clawClose = 0.55;
    public static double autoClawClose = 2;
    public static double autoClawReset = 5;
    public static double slideSpeed = -0.05;
    public static int slideWaitRatio = 2500; // Adjusted value for slide movement to millseconds ratio (does not work well)
    public static int slideWaitARatio = 715; // Actual slide movement to milliseconds ratio
    public static int slideRetractWait = 200;
    public static int slideDelay = 1000;
    public static double slideCycleShorten = 0.05;
    public static int clawCloseDelay = 400;
    public static int clawOpenDelay = 250;
    public static double cycleTime = 7500;

    public static int autoLiftCone = -123; // encoder value to increase by per cone in stack
    public static double liftUpRatio = 1;
    public static double liftDownRatio = 0.8;
    public static double liftSlow = -500;
    public static double liftSlowRatio = 0.5;
    public static double setRotateMultiplier = 0.7;
    public static int rotMotorPosPerDegree = 12;
    public static int autoTurnFirstTall = 1628;
    public static int autoTurnTall = 1392;

    // Lift Positions
    public static int liftHigh = -3050;
    public static int liftMed = -2200;
    public static int liftLow = -1400;
    public static int liftFloor = -75;
    public static int liftSpin = -400;
    public static int liftMax = -3100;
    public static int liftMin = 0;
    public static int liftDrive = -500;
    public static double liftMinPow = 0.1;
    public static int liftkP = 10;
    public static int liftTolerance = 15;
    public static int coneDodge = -375;
    public static int junctionDodge = 222;

    // Rotation Positions
    public static int rotRLimit = 4270;
    public static int rotMax = 8750;
    public static double rotMin = 0.05;
    public static double rotkP = 25;
    public static int rotExtendScale = 5;
    public static int rot180L = -2125;
    public static int rot180R = 2125;
    public static int rot90L = -1081;
    public static int rot90R = 1081;
    public static int rot45L = -300;
    public static int rot45R = 300;
    public static int rotDiagBackR = 1875;
    public static int rotDiagBackL = -1875;
    public static int rotTolerance = 25;
    public static int rotFrontBuffer = 75;
    public static int autoRotTolerance = 125;

    // Drive motor (5203 312 rpm)
    public static final double TICKS_PER_REV = 537.7;
    public static final double CIRCUMFERENCE_IN_INCHES = 96 / 25.4 * Math.PI;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / CIRCUMFERENCE_IN_INCHES;
    public static double moveSpeed = 1;
    public static double rotSpeed = 1;

    // Autonomous turn PID
    public static double kR = 0.084; // PID turn kR
    public static double kD = 0.0072; // PID turn kD

    public static double tkR = 0.03;
    public static double tskR = 0.03;

    public static double BlueThresh = 0.7;
    public static double RedThresh = 0.5;
    public static double ColorStripAlignmentSpeed = 0.4;
    public static int ColorStripAlignmentDelay = 5000;
    public static int gain = 150;

    public static int imgWidth = 1920;
    public static int imgHeight = 1080;
    public static int changeThresh = 128;
    public static int negChangeThresh = -128;

    // Camera stuff
    public static int topAlignLine = 500;
    public static int midAlignLine = 575;
    public static int bottomAlignLine = 650;
    public static int leftAlignStart = 550;
    public static int rightAlignEnd = 1250;
    public static double alignRatio = 0.0170824;
    public static int kernelSize = 7;
    public static double cameraTolerance = 0.5;

    public static int leftBoundary = 940; // left side of detection zone
    public static int rightBoundary = 1010; // right side of detection zone
    public static int middleLine = 735; // detection line y coordinate


    public static int HVLeftBoundary = 910; // left side of detection zone
    public static int HVRightBoundary = 1010; // right side of detection zone
    public static int HVTopBoundary = 685; // top side of detection zone
    public static int HVBottomBoundary = 785; // bottom side of detection zone
    public static int colorThresh = 160;

    public static int maskChangeThresh = 1;
    public static int negMaskChangeThresh = -1;
    public static double slopeThresh = 100.0; // 100
    public static double negSlopeThresh = -100.0; // 100
    public static boolean isDetectRed = false; // False: detect Blue
    public static int signalDetectionMethod = 3; // 1: detect QR code
                                                 // 2: detect vertical 1, 2, 3 lines: require rigid alignment
                                                 // 3: detect H vs V vs Empty: best solution, require less alignment
                                                 // 4: detect H vs V vs Diagonal
                                                 // 5: detect H vs V vs #

    public static final int automationDelay = 3;
    public static final int autonomousAutomationDelay = 50;
    public static final int buttonDelay = 36;

    public static double straightTestDist = 75;
    public static double straightTestPow = 0.7;

    public static boolean debugMode = true; // Change it to FALSE before the competition!!!!

    //tensorflow constants
    public static double magnitude = 1;
    public static double aspectRatio = 16.0/9.0;
    public static float minResultConfidence = 0.50f;
    public static float moveForward = 7.0f;

    // distance sensor movement constants
    public static double setVerticalMinSpeed = 0.18;
    public static double setVerticalMaxSpeed = 1;
    public static double setHorizontalMinSpeed = 0.3;
    public static double setHorizontalMaxSpeed = 1;
    public static double setDiagonalMinSpeed = 0.3;
    public static double setDiagonalMaxSpeed = 1;
    public static double setHorizontalDisCap = 24;
    public static double setVerticalDisCap = 36;
    public static double test1 = 0;
    public static double test2 = 0;

    // PID stuff, max velocity of drive motors: 2640
    public static double drivekP = 1.24;
    public static double drivekI = 0.124;
    public static double drivekD = 0;
    public static double drivekF = 12.41;
    public static double drivePoskP = 5;
}
