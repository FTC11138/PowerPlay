package org.firstinspires.ftc.teamcode.powerplay;

import android.graphics.Color;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class AutonomousMethods extends LinearOpMode {
    public Attachments myRobot = new Attachments();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    private Orientation angles;
    public ElapsedTime runtime = new ElapsedTime();


    public boolean opModeStatus() {
        return opModeIsActive();
    }


    // Initializations
    public void initializeAutonomousDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
    }

    public void initializeAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initialize(hardwareMap, telemetry);
    }

//    public double autonomousGetAngle() {
//        return myRobot.getAngle();
//    }

    // Drive stuff
    public void setModeAllDrive(DcMotor.RunMode mode) {
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }

    public void runMotors(double leftPower, double rightPower) {
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }

    private void multiSetTargetPosition(double ticks, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }

    private boolean notCloseEnough(int tolerance, DcMotor... motors) {
        for (DcMotor motor : motors) {
            if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > tolerance) {
                return true;
            }
        }
        return false;
    }

    public void encoderStraightDrive(double inches, double power) {

        double originalAngle = getHorizontalAngle();
        double currentAngle;
        double angleError;

        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Log.d("test test", "test2 " + (inches * Constants.TICKS_PER_INCH));
        ElapsedTime time = new ElapsedTime();
        multiSetTargetPosition(inches * Constants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
//        Log.d("test test", "test");
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && /*time.milliseconds()<4000 &&*/ opModeIsActive()) {
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition() + "beep");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition() + "beep");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition() + "beep");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition() + "beep");

            currentAngle = getHorizontalAngle();
            angleError = loopAround(currentAngle - originalAngle);
            runMotors(power + angleError * Constants.tskR, power - angleError * Constants.tskR);
        }
//        Log.d("test test", "test3");
        runMotors(0, 0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void setLiftMotor(int position, double currentPosition, double tolerance) {
        //Undefined constants
        double newPower;
        double error = -(position - currentPosition) / Constants.liftMax;

        //Initial Time
        telemetry.addData("1", "error: " + error);
        telemetry.update();
        if (Math.abs(error) > (tolerance / -Constants.liftMax)) {
            //Setting p action
            newPower = Math.max(Math.min(error * Constants.liftkP, 1), -1);

            //Set real power
            newPower = Math.max(Math.abs(newPower), Constants.liftMinPow) * Math.signum(newPower);
            if (Math.signum(newPower) == 1) {
                newPower = newPower * Constants.liftDownRatio;
            }
            myRobot.runLiftMotor(newPower);
        } else {
            myRobot.runLiftMotor(0);
        }
    }


    // Drop cone
    public void dropCone(int liftTarget, int rotTarget, double slideTarget) {
        double currentLiftPosition, currentRPosition, currentSlidePosition;
        double liftPower, liftError, rPower, rError;
        int stage = 1;
        while (stage <= 4 && opModeIsActive()) {
            switch (stage) {
                case 2: // rotating
                    myRobot.setSlideServo(Constants.slideIn);
                    currentRPosition = myRobot.getRotationMotorPosition();
                    rError = (rotTarget - currentRPosition) / Constants.rotMax;
                    telemetry.addData("2 2", "rotation error: " + rError);
                    if (Math.abs(rError) > (Constants.autoRotTolerance / Constants.rotMax)) {
                        //Setting p action
                        rPower = Math.max(Math.min(rError * Constants.rotkP, 1), -1);
                        rPower = Math.max(Math.abs(rPower), Constants.rotMin) * Math.signum(rPower);
                        myRobot.runRotateMotor(rPower);
                    } else {
                        myRobot.setRotateMotor(0.3, rotTarget);
                        stage = 3;
                    }
                case 1: // lifting up
                    currentLiftPosition = myRobot.getLiftMotorPosition();
                    if (stage == 1 && currentLiftPosition < Constants.liftSpin) {
                        stage = 2;
                        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    liftError = -((liftTarget) - currentLiftPosition) / Constants.liftMax;
                    telemetry.addData("2 1", "lift error: " + liftError);
                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower);
                        myRobot.runLiftMotor(liftPower);
                    } else {
                        myRobot.runLiftMotor(0);
                    }
                    break;
                case 3: // extending
                    myRobot.setSlideServo(slideTarget);
                    stage = 4;
                    break;
                case 4: // dropping :)
                    currentLiftPosition = myRobot.getLiftMotorPosition();
                    currentSlidePosition = myRobot.getSlidePosition();
                    if ((currentLiftPosition - liftTarget) <= Constants.liftTolerance &&
                            // TODO: next line doesn't work like I want it to, probably have to
                            //  switch this from case 4 to case default and use if statement inside
                            //  to check the stage. Figure out a delay ratio with the amount to
                            //  extend
                            (currentSlidePosition - slideTarget <= Constants.slideTolerance)) {
                        myRobot.setClawServo(Constants.clawOpen);
                        sleep(250);
                        stage = Constants.autonomousAutomationDelay + 1;
                        break;
                    }
                    liftError = -(liftTarget - currentLiftPosition) / Constants.liftMax;
                    telemetry.addData("1 1", "lift error: " + liftError);
                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower) * Constants.liftDownRatio;
                        myRobot.runLiftMotor(liftPower);
                    } else {
                        myRobot.runLiftMotor(0);
                    }
                    break;
            }
        }
    }


    // Target Angle for rotation, inches to drive before using color sensor, power for drive
    public void multitaskMovement(double targetAngle, int rotTarget, double inches, double power) {
        double currentAngle, angleError, currentLiftPosition, currentRPosition;
        double liftPower, liftError, rPower, rError;
        int stage = 1;
        myRobot.setLiftMotor(1, Constants.liftFloor);
//        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Log.d("test test", "test2 " + (inches * Constants.TICKS_PER_INCH));
        sleep(500);
        multiSetTargetPosition(inches * Constants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Log.d("test test", "test");
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && opModeIsActive()) {
            // Angle adjustment
            currentAngle = getHorizontalAngle();
            angleError = loopAround(currentAngle - targetAngle);
            runMotors(power + angleError * Constants.tskR, power - angleError * Constants.tskR);

            // Everything else
            switch (stage) {
                case 2: // rotating
                    myRobot.setSlideServo(Constants.slideIn);
                    currentRPosition = myRobot.getRotationMotorPosition();
                    rError = (rotTarget - currentRPosition) / Constants.rotMax;
                    telemetry.addData("2", "rotation error: " + rError);
                    if (Math.abs(rError) > (Constants.autoRotTolerance / Constants.rotMax)) {
                        //Setting p action
                        rPower = Math.max(Math.min(rError * Constants.rotkP, 1), -1);
                        rPower = Math.max(Math.abs(rPower), Constants.rotMin) * Math.signum(rPower);
                        myRobot.runRotateMotor(rPower);
                    } else {
                        myRobot.setRotateMotor(0.3, rotTarget);
                        stage = 3;
                    }
                case 1: // lifting up
                    currentLiftPosition = myRobot.getLiftMotorPosition();
                    if (stage == 1 && currentLiftPosition < Constants.liftSpin) {
                        stage = 2;
                        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    liftError = -((Constants.liftSpin - 100) - currentLiftPosition) / Constants.liftMax;
                    telemetry.addData("1", "lift error: " + liftError);
                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower);
                        myRobot.runLiftMotor(liftPower);
                    } else {
                        myRobot.runLiftMotor(0);
                    }
                    break;
                case 3: // extending
                    myRobot.setSlideServo(Constants.autoSlideTurn);
                    stage = 4;
                    break;
                case Constants.autonomousAutomationDelay: // lowering
                    currentLiftPosition = myRobot.getLiftMotorPosition();
                    liftError = currentLiftPosition / Constants.liftMax;
                    telemetry.addData("1", "lift error: " + liftError);
                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower) * Constants.liftDownRatio;
                        myRobot.runLiftMotor(liftPower);
                    } else {
                        myRobot.runLiftMotor(0);
                        stage = Constants.autonomousAutomationDelay + 1;
                        break;
                    }
                    break;
                default:
                    if (stage <= Constants.autonomousAutomationDelay) {
                        stage++;
                        currentLiftPosition = myRobot.getLiftMotorPosition();
                        liftError = -((Constants.liftSpin - 100) - currentLiftPosition) / Constants.liftMax;
                        telemetry.addData("1", "lift error: " + liftError);
                        if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                            liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                            liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower);
                            myRobot.runLiftMotor(liftPower);
                        } else {
                            myRobot.runLiftMotor(0);
                        }
                    }
            }
            telemetry.addData("stage", "current stage: " + stage);
            telemetry.update();
        }
        encoderTurn(0, 0.3, 1);
        sleep(250);

        // Make sure the slides are spun out and down
        while (stage <= Constants.autonomousAutomationDelay && opModeIsActive()) {
            switch (stage) {
                case 2: // rotating
                    myRobot.setSlideServo(Constants.slideIn);
                    currentRPosition = myRobot.getRotationMotorPosition();
                    rError = (rotTarget - currentRPosition) / Constants.rotMax;
                    telemetry.addData("2 2", "rotation error: " + rError);
                    if (Math.abs(rError) > (Constants.autoRotTolerance / Constants.rotMax)) {
                        //Setting p action
                        rPower = Math.max(Math.min(rError * Constants.rotkP, 1), -1);
                        rPower = Math.max(Math.abs(rPower), Constants.rotMin) * Math.signum(rPower);
                        myRobot.runRotateMotor(rPower);
                    } else {
                        myRobot.setRotateMotor(0.3, rotTarget);
                        stage = 3;
                    }
                case 1: // lifting up
                    currentLiftPosition = myRobot.getLiftMotorPosition();
                    if (stage == 1 && currentLiftPosition < Constants.liftSpin) {
                        stage = 2;
                        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    liftError = -((Constants.liftSpin - 100) - currentLiftPosition) / Constants.liftMax;
                    telemetry.addData("2 1", "lift error: " + liftError);
                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower);
                        myRobot.runLiftMotor(liftPower);
                    } else {
                        myRobot.runLiftMotor(0);
                    }
                    break;
                case 3: // extending
                    myRobot.setSlideServo(Constants.autoSlideTurn);
                    stage = 4;
                    break;
                case Constants.autonomousAutomationDelay: // lowering
                    currentLiftPosition = myRobot.getLiftMotorPosition();
                    liftError = currentLiftPosition / Constants.liftMax;
                    telemetry.addData("2 1", "lift error: " + liftError);
                    if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                        liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                        liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower) * Constants.liftDownRatio;
                        myRobot.runLiftMotor(liftPower);
                    } else {
                        myRobot.runLiftMotor(0);
                        stage = Constants.autonomousAutomationDelay + 1;
                        break;
                    }
                    break;
                default:
                    if (stage <= Constants.autonomousAutomationDelay) {
                        stage++;
                        currentLiftPosition = myRobot.getLiftMotorPosition();
                        liftError = -((Constants.liftSpin - 100) - currentLiftPosition) / Constants.liftMax;
                        telemetry.addData("2 1", "lift error: " + liftError);
                        if (Math.abs(liftError) > (Constants.liftTolerance / -Constants.liftMax)) {
                            liftPower = Math.max(Math.min(liftError * Constants.liftkP, 1), -1);
                            liftPower = Math.max(Math.abs(liftPower), Constants.liftMinPow) * Math.signum(liftPower);
                            myRobot.runLiftMotor(liftPower);
                        } else {
                            myRobot.runLiftMotor(0);
                        }
                    }
            }
            telemetry.addData("2 stage", "current stage: " + stage);
            telemetry.update();
        }

        // while color sensor doesn't detect
//        while (opModeIsActive()) {
//            currentAngle = getHorizontalAngle();
//            angleError = loopAround(currentAngle - targetAngle);
//            runMotors(0.1 + angleError * Constants.tkR, 0.1 - angleError * Constants.tkR);
//            if (calculateBlue()) {
//                runMotors(0, 0);
//                break;
//            }
//        }
        encoderTurn(0, 0.3, 1);

        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    boolean calculateBlue() {
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = myRobot.colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red 1", "%.1f", colors.red)
                .addData("Green 1", "%.1f", colors.green)
                .addData("Blue 1", "%.1f", colors.blue);
        telemetry.update();

        if (colors.blue > Constants.ColorThresh) {
            return true;
        } else {
            return false;
        }
    }

    //Negative = Left, Positive = Right
    public void encoderStrafeDriveInchesRight(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int) Math.round(inches * Constants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
//        ElapsedTime killTimer = new ElapsedTime();
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.lb, myRobot.rf, myRobot.rb) && opModeIsActive() /*&& killTimer.seconds()<2*/) {
            Log.d("SkyStone Left Front: ", myRobot.lf.getCurrentPosition() + "");
            Log.d("SkyStone Left Back: ", myRobot.lb.getCurrentPosition() + "");
            Log.d("SkyStone Right Front: ", myRobot.rf.getCurrentPosition() + "");
            Log.d("SkyStone Right Back: ", myRobot.rb.getCurrentPosition() + "");
        }
        runMotors(0, 0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // IMU Stuff
    public double getHorizontalAngle() {
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.firstAngle;
        output = loopAround(output);
        return output;
    }

    protected double loopAround(double output) {
        if (output > 180) {
            output -= 360;
        }
        if (output < -180) {
            output += 360;
        }
        return output;
    }

    public double getRoll() {
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        output = loopAround(output);
        return output;
    }

    public double getVerticalAngle() {
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        output = loopAround(output);
        return output;
    }

    //Positive = Clockwise, Negative = Counterclockwise
    public void encoderTurn(double targetAngle, double power, double tolerance) {
        encoderTurnNoStop(targetAngle, power, tolerance);
        runMotors(0, 0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderTurnNoStop(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, -power, power, tolerance, true);
    }

    void encoderTurnNoStopPowers(double targetAngle, double leftPower, double rightPower, double tolerance, boolean usePID) {
        double kR = Constants.kR;
        double kD = Constants.kD;

        //Undefined constants
        double d;
        double dt;
        double leftProportionalPower;
        double rightProportionalPower;
        //Initial error
        double currentAngle = getHorizontalAngle();
        double error = targetAngle - currentAngle;
        error = loopAround(error);
        double previousError = error;
        //Initial Time
        ElapsedTime clock = new ElapsedTime();
        double t1 = clock.nanoseconds();
        double t2 = t1;
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(error) > tolerance && opModeIsActive()) {
            //Getting Error
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle - currentAngle);
            if (usePID) {
                //Getting time difference
                t2 = clock.nanoseconds();
                dt = t2 - t1;

                //Setting d action
                d = (error - previousError) / dt * Math.pow(10, 9);
                //Setting p action
                leftProportionalPower = Math.max(Math.min(error * kR + d * kD, 1), -1) * leftPower;
                rightProportionalPower = Math.max(Math.min(error * kR + d * kD, 1), -1) * rightPower;
                Log.d("Skystone: ", "leftProportionalPower: " + leftProportionalPower + " rightProportionalPower: " + rightProportionalPower);
                Log.d("Skystone: ", "dt: " + dt + "DerivativeAction: " + d * kD);
            } else {
                leftProportionalPower = leftPower * Math.signum(error);
                rightProportionalPower = rightPower * Math.signum(error);
            }

            //Set real power
            double realLeftPower = Math.max(Math.abs(leftPower / 2), Math.abs(leftProportionalPower)) * Math.signum(leftProportionalPower);
            double realRightPower = Math.max(Math.abs(rightPower / 2), Math.abs(rightProportionalPower)) * Math.signum(rightProportionalPower);
            runMotors(realLeftPower, realRightPower);

            //Store old values
            previousError = error;
            if (usePID) {
                t1 = t2;
            }


            //Logging
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + realLeftPower + "rightPower: " + realRightPower + "CurrentAngle: " + currentAngle);
        }
    }
}
