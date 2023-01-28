package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // PowerPlay Stuff
    public static double parkBuffer = 2500;

    public static double slideIn = 0.305;
    public static double slideOut = 1;
    public static double slideOpt = 0.35; // Optimal slide position for rotation
    public static double slideMed = 0.75;
//    public static double autoSlideTurn = 0.45;
//    public static double autoSlideOut = 0.15;
    public static double autoSlideCycle = 0.85;
    public static double slideCycleBack = 0.15;
    public static double autoDistCycle = 2.5; // inches to be away from cone
    public static double clawOpen = 0.4;
    public static double clawFurtherOpen = 0.25;
    public static double clawClose = 0.56; // old claw was 0.53
    public static double autoClawClose = 0.52;
//    public static double autoClawReset = 5;
    public static double slideSpeed = 0.05;
    public static int slideWaitRatio = 2500; // Adjusted value for slide movement to milliseconds ratio (does not work well)
    public static int slideWaitARatio = 715; // Actual slide movement to milliseconds ratio
    public static int slideRetractWait = 200;
    public static int slideDelay = 1000;
    public static double slideCycleShorten = 0.05;
    public static int clawCloseDelay = 400;
    public static int clawOpenDelay = 200;
    public static double cycleTimeHigh = 5000;
    public static double cycleTimeMed = 4000;

    public static int autoLiftCone = -45; // encoder value to increase by per cone in stack
    public static double liftUpRatio = 1;
    public static double liftDownRatio = 0.8;
    public static double liftSlow = -500;
    public static double liftSlowRatio = 0.4;
    public static double setRotateMultiplier = 0.35;
    public static int rotMotorPosPerDegree = 12;

    // Lift Positions
    public static double newLiftRatio = 145.1 / 384.5;
    public static int liftHigh = -1125;
    public static int liftMed = -830;
    public static int liftLow = -528;
    public static int liftFloor = -75;
    public static int liftSpin = -175;
    public static int liftMax = -1175;
    public static int liftMin = 0;
    public static int liftDrive = -250;
    public static double liftMinPow = 0.1;
    public static int liftkPTele = 10;
    public static int liftTolerance = 15;
    public static int coneDodge = -142;
    public static int junctionDodge = 275; // 236 without furtheropen claw
    public static int lowJuncDodgeFirst = 1325;
    public static int lowJuncDodge = 1050;

    // Rotation Positions
    public static int rotRLimit = 4270;
    public static int rotMax = 8750;
    public static double rotMin = 0.05;
    public static double rotkP = 25;
    public static int rotExtendScale = 5;
    public static int rot180L = -2125;
    public static int rot180R = 2125;
    public static int rot90L = -1081;
    public static int rot90LLong = 4270-1081;
    public static int rot90R = 1081;
    public static int rot90RLong = -4270+1081;
    public static int earlyRotDrop = 25;
    public static int rot45L = -300;
    public static int rot45R = 300;
    public static int rotDiagBackR = 1875;
    public static int rotDiagBackL = -1875;
    public static int rotTolerance = 25;
    public static int rotFrontBuffer = 75;
    public static int autoRotTolerance = 125;

    public static double autoRotSpeed = 0.8;

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


    public static int HVLeftBoundary = 930; // left side of detection zone
    public static int HVRightBoundary = 1030; // right side of detection zone
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

    public static boolean debugMode = false; // Change it to FALSE before the competition!!!!

    //tensorflow constants
    public static double magnitude = 1;
    public static double aspectRatio = 16.0 / 9.0;
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
    public static double test1 = -48;
    public static double test2 = 1;
    public static double test3 = 0;
    public static double test4 = 1;

    // Lift PID
    public static double liftkP = 0.005; // 10
    public static double liftkI = 0;
    public static double liftkD = 0.001;
    public static double liftkF = 0.25;

    // PID stuff, max velocity of drive motors: 2640
    public static double drivekP = 5;
    public static double drivekI = 5;
    public static double drivekD = 0;
    public static double drivekF = 20;
    public static double drivePoskP = 7.5;
    public static double autoDriveSpeed = 0.7;
}
