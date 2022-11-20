package org.firstinspires.ftc.teamcode.baseBot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@Autonomous(name = "StrafeTest_Color", group = "Linear Opmode")
@Disabled
public class StrafeTest_Color extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    View relativeLayout;

    @Override
    public void runOpMode() {
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        waitForStart();

        runtime.reset();

        sleep(120);

        double originalAngle = getHorizontalAngle();
        double currentAngle;
        double angleError;

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
        colorSensor1.setGain(Constants.gain);
        colorSensor2.setGain(Constants.gain);

        myRobot.lb.setPower(-Constants.ColorStripAlignmentSpeed);
        myRobot.lf.setPower(Constants.ColorStripAlignmentSpeed);
        myRobot.rb.setPower(Constants.ColorStripAlignmentSpeed);
        myRobot.rf.setPower(-Constants.ColorStripAlignmentSpeed);
        while (true) {
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            currentAngle = getHorizontalAngle();
            angleError = loopAround(currentAngle - originalAngle);
            myRobot.lb.setPower(-Constants.ColorStripAlignmentSpeed + angleError * Constants.tkR);
            myRobot.lf.setPower(Constants.ColorStripAlignmentSpeed + angleError * Constants.tkR);
            myRobot.rb.setPower(Constants.ColorStripAlignmentSpeed - angleError * Constants.tkR);
            myRobot.rf.setPower(-Constants.ColorStripAlignmentSpeed - angleError * Constants.tkR);

            try {
                if (calculateCurrentColor()) {
                    myRobot.lb.setPower(0);
                    myRobot.lf.setPower(0);
                    myRobot.rb.setPower(0);
                    myRobot.rf.setPower(0);
                    break;
                }
            } finally {
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.WHITE);
                    }
                });
            }
        }

        sleep(Constants.ColorStripAlignmentDelay);

        myRobot.lb.setPower(0.2);
        myRobot.lf.setPower(-0.2);
        myRobot.rb.setPower(-0.2);
        myRobot.rf.setPower(0.2);
        while (true) {
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            try {
                if (calculateCurrentColor()) {
                    myRobot.lb.setPower(0);
                    myRobot.lf.setPower(0);
                    myRobot.rb.setPower(0);
                    myRobot.rf.setPower(0);
                    break;
                }
            } finally {
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.WHITE);
                    }
                });
            }
        }
    }

    private boolean calculateCurrentColor() {

        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];

        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
        Color.colorToHSV(colors1.toColor(), hsvValues1);
        Color.colorToHSV(colors2.toColor(), hsvValues2);


        telemetry.addLine()
                .addData("Red 1", "%.1f", colors1.red)
                .addData("Green 1", "%.1f", colors1.green)
                .addData("Blue 1", "%.1f", colors1.blue);
        telemetry.addLine()
                .addData("Red 2", "%.1f", colors2.red)
                .addData("Green 2", "%.1f", colors2.green)
                .addData("Blue 2", "%.1f", colors2.blue);

        telemetry.update();

        if (colors1.blue > Constants.ColorThresh && colors2.blue > Constants.ColorThresh) {
            return true;
        } else {
            return false;
        }

    }
}