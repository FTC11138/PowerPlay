package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

@Autonomous(name = "RightAutoWithColorSens", group = "Linear Opmode")
public class RightAutoWithColorSens extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();

    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto(hardwareMap, telemetry);

        myRobot.colorSensor.setGain(Constants.gain);

        double originalAngle = 0;
        double currentAngle;
        double angleError;
        
        waitForStart();
        runtime.reset();

        encoderStraightDrive(3, 0.2);
        encoderStrafeDriveInchesRight(18, 0.5);
        encoderStraightDrive(25, 0.7);

        runMotors(0.2, 0.2);

        while (true) {
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            currentAngle = getHorizontalAngle();
            angleError = loopAround(currentAngle - originalAngle);
            runMotors(0.2 + angleError * Constants.tkR, 0.2 - angleError * Constants.tkR);

            try {
                if (calculateBlue()) {
                    runMotors(0, 0);
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
}
