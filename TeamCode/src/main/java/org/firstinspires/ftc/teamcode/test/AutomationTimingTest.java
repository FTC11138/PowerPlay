//package org.firstinspires.ftc.teamcode.test;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;
//import org.firstinspires.ftc.teamcode.powerplay.Constants;
//
//@Autonomous(name = "Automation Timing", group = "Linear Opmode")
//@Disabled
//public class AutomationTimingTest extends AutonomousMethods {
//
//    private ElapsedTime runtime = new ElapsedTime();
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    TelemetryPacket packet = new TelemetryPacket();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initializeAuto(hardwareMap, telemetry);
//        myRobot.setClawServo(Constants.clawClose);
//        telemetry.addData("Status", "Initialization Finished");
//        telemetry.update();
//        waitForStart();
//        runtime.reset();
//
//        int liftTarget = Constants.liftLow;
//        int rotTarget = Constants.autoTurnTall;
//        double slideTarget = Constants.autoSlideTall;
//        double currentLiftPosition, currentRPosition;
//        int delay = (int) Math.ceil(Constants.slideLoopRatio * Math.abs(myRobot.getSlidePosition() - slideTarget));
//        double liftPower, liftError, rPower, rError;
//        int stage = 1;
//        while (stage != -1 && opModeIsActive()) {
//            switch (stage) {
//                case 2: // rotating
//                    myRobot.setSlideServo(Constants.slideIn);
//                    currentRPosition = myRobot.getRotationMotorPosition();
//                    rError = (rotTarget - currentRPosition) / Constants.rotMax;
//                    telemetry.addData("2 2", "rotation error: " + rError);
//                    if (Math.abs(rError) > (Constants.autoRotTolerance / Constants.rotMax)) {
//                        //Setting p action
//                        rPower = Math.max(Math.min(rError * Constants.rotkP, 1), -1);
//                        rPower = Math.max(Math.abs(rPower), Constants.rotMin) * Math.signum(rPower);
//                        myRobot.runRotateMotor(rPower);
//                    } else {
//                        myRobot.setRotateMotor(0.3, rotTarget);
//                        stage = 3;
//                    }
//                case 1: // lifting up
//                    currentLiftPosition = myRobot.getLiftMotorPosition();
//                    if (stage == 1 && currentLiftPosition < Constants.liftSpin) {
//                        stage = 2;
//                        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    }
//                    liftError = -((liftTarget) - currentLiftPosition) / Constants.liftMax;
//                    telemetry.addData("2 1", "lift error: " + liftError);
//                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
//                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
//                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower);
//                        myRobot.runLiftMotor(liftPower);
//                    } else {
//                        myRobot.runLiftMotor(0);
//                    }
//                    break;
//                case 3: // extending
//                    myRobot.setSlideServo(slideTarget);
//                    stage = 4;
//                    break;
//                default: // dropping :)
//                    currentLiftPosition = myRobot.getLiftMotorPosition();
//                    if ((currentLiftPosition - liftTarget) <= Constants.liftTolerance &&
//                            stage >= delay + 3) {
//                        myRobot.setClawServo(Constants.clawOpen);
//                        stage = -1;
//                        break;
//                    } else {
//                        stage++;
//                    }
//                    liftError = -(liftTarget - currentLiftPosition) / Constants.liftMax;
//                    telemetry.addData("1 1", "lift error: " + liftError);
//                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
//                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
//                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower) * Constants.liftDownRatio;
//                        myRobot.runLiftMotor(liftPower);
//                    } else {
//                        myRobot.runLiftMotor(0);
//                    }
//                    Log.d("Current Time Powerplay", "" + System.currentTimeMillis());
//                    telemetry.addData("Current Time", "" + System.currentTimeMillis());
//                    break;
//            }
//            telemetry.addData("stage", stage);
//            telemetry.update();
//        }
//    }
//}
