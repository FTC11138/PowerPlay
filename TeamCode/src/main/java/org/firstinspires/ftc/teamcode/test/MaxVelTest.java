package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.Attachments;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@TeleOp(name="Max Velocity", group="Iterative Opmode")
//@Disabled
public class MaxVelTest extends LinearOpMode {
    Attachments myRobot = new Attachments();
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        myRobot.initialize(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (Constants.debugMode) {
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        waitForStart();
        myRobot.runLiftMotor(Constants.test2);
        while (opModeIsActive()) {
            currentVelocity = myRobot.liftMotor.getVelocity();

            if (myRobot.getLiftMotorPosition() < (Constants.liftHigh / 2)) {
                myRobot.runLiftMotor(0);
            }

            if (Math.abs(currentVelocity) > maxVelocity) {
                maxVelocity = Math.abs(currentVelocity);
            }
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
            Log.d("cur velah", "" + currentVelocity);
            Log.d("max velah", "" + maxVelocity);
        }
    }
}
