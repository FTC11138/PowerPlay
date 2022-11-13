package org.firstinspires.ftc.teamcode.baseBot;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.outreachBot.ClawBot;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

public abstract class BaseAutonomousMethods extends LinearOpMode {
    public ClawBot myRobot = new ClawBot();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    private Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();


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

    public void runMotors(WheelPowers wps) {
        myRobot.lb.setPower(wps.lbPower);
        myRobot.lf.setPower(wps.lfPower);
        myRobot.rb.setPower(wps.rbPower);
        myRobot.rf.setPower(wps.rfPower);
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
            Log.d("test test", "test");
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

    //Negative = Left, Positive = Right
    public void encoderStrafeDriveInchesRight(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int) Math.round(inches * Constants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime killTimer = new ElapsedTime();
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

    double loopAround(double output) {
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

    public class WheelPowers {
        public double lfPower;
        public double lbPower;
        public double rfPower;
        public double rbPower;

        public WheelPowers(double power) {
            lfPower = power;
            lbPower = power;
            rfPower = power;
            rbPower = power;
        }

        public WheelPowers(double leftPower, double rightPower) {
            lfPower = leftPower;
            lbPower = leftPower;
            rfPower = rightPower;
            rbPower = rightPower;
        }

        public WheelPowers(double lfp, double lbp, double rfp, double rbp) {
            lfPower = lfp;
            lbPower = lbp;
            rfPower = rfp;
            rbPower = rbp;
        }

        public void adjustPowers(double leftAdjust, double rightAdjust) {
            lfPower += leftAdjust;
            lbPower += leftAdjust;
            rfPower += rightAdjust;
            rbPower += rightAdjust;
        }

        public String toString() {
            return "lf: " + lfPower + ", lb: " + lbPower + ", rf: " + rfPower + ", rb: " + rbPower;
        }
    }
}
