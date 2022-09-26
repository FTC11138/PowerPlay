package org.firstinspires.ftc.teamcode.outreachBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Claw Bot TeleOp", group = "Iterative Opmode")
//@Disabled
public class ClawBotTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ClawBot myRobot = new ClawBot();

    /* ------------------------------------ CONSTANTS ------------------------------------ */
    // Servos
    private double clawPos = Constants.clawOpenB;
    private double liftPos = Constants.liftDrive;
    private boolean isClawBotA = false;
    private int aPause = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        myRobot.setClawServo(clawPos);
        myRobot.setLiftServo(liftPos);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */
        //Drive motor controls
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = Constants.joyStraight;
        double rotationMultiplier = Constants.joyTurn;

        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = Constants.dpadStraight;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = Constants.dpadStraight;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = Constants.dpadSide;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = Constants.dpadSide;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);

        /* ------------------------------------ Change ------------------------------------ */
        if (gamepad1.x) {
            liftPos = Constants.liftDrive;
        } else if (gamepad1.y) {
            liftPos = Constants.liftTop;
        } else if (gamepad1.a) {
            liftPos = Constants.liftBot;
        }

        if (gamepad1.left_bumper) {
            if (isClawBotA) {
                clawPos = Constants.clawCloseA;
            } else {
                clawPos = Constants.clawCloseB;
            }
        } else if (gamepad1.right_bumper) {
            if (isClawBotA) {
                clawPos = Constants.clawOpenA;
            } else {
                clawPos = Constants.clawOpenB;
            }
        }

        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        if (rt > 0.3) {
            liftPos += rt * Constants.liftRatio;
        } else if (lt > 0.3) {
            liftPos -= lt * Constants.liftRatio;
        }

        if (liftPos > Constants.liftBot) {
            liftPos = Constants.liftBot;
        } else if (liftPos < Constants.liftTop) {
            liftPos = Constants.liftTop;
        }

        if (aPause == 0) {
            if (gamepad1.b) {
                isClawBotA = !isClawBotA;
            }
        } else {
          aPause++;
          aPause %= Constants.buttonDelay;
        }



        /* ------------------------------------ Action ------------------------------------ */
        myRobot.setLiftServo(liftPos);
        myRobot.setClawServo(clawPos);



        /* ------------------------------------ Logging ------------------------------------ */
        telemetry.addData("lift position", liftPos);
        telemetry.addData("claw position", clawPos);
        telemetry.addData("Claw Bot A?", isClawBotA);
        telemetry.update();

        Log.d("AHHHHHH lift", String.valueOf(liftPos));
        Log.d("AHHHHHH claw", String.valueOf(clawPos));
    }

    @Override
    public void stop() {
    }
}

