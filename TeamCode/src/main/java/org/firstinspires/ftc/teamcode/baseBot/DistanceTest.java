package org.firstinspires.ftc.teamcode.baseBot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.powerplay.Attachments;
import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@Autonomous(name = "DistanceTest", group = "Linear Opmode")
@Disabled
public class DistanceTest extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeAuto(hardwareMap, telemetry);

        waitForStart();

        runtime.reset();

        sleep(120);

        while (opModeIsActive()) {
            telemetry.addData("Distance to stack: %f", myRobot.clawDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }


    }
}