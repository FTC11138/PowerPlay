package org.firstinspires.ftc.teamcode.powerplay;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RightAutoWithColor2", group = "Linear Opmode")
public class RightAutoWithColor2 extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto(hardwareMap, telemetry);

        myRobot.colorSensor.setGain(Constants.gain);
        
        waitForStart();
        runtime.reset();

        multitaskMovement(0, Constants.rot90R, 34, 0.75);
    }
}
