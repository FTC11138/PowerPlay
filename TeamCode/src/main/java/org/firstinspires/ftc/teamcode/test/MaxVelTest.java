package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.Attachments;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@TeleOp(name="Max Velocity", group="Iterative Opmode")
@Disabled
public class MaxVelTest extends LinearOpMode {
    Attachments myRobot = new Attachments();
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        myRobot.initialize(hardwareMap, telemetry);
        myRobot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myRobot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myRobot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myRobot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        myRobot.runMotors(-1, -1);
        while (opModeIsActive()) {
            currentVelocity = myRobot.lb.getVelocity();

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
