package org.firstinspires.ftc.teamcode.outreachBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Claw Bot TeleOp", group = "Iterative Opmode")
@Disabled
public class ClawBotTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ClawBot myRobot = new ClawBot();

    /* ------------------------------------ OutreachCONSTANTS ------------------------------------ */
    // Servos
    private double clawPos = OutreachConstants.clawOpenB;
    private double liftPos = OutreachConstants.liftDrive;
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
        double speedMultiplier = OutreachConstants.joyStraight;
        double rotationMultiplier = OutreachConstants.joyTurn;

        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = OutreachConstants.dpadStraight;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = OutreachConstants.dpadStraight;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = OutreachConstants.dpadSide;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = OutreachConstants.dpadSide;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);

        /* ------------------------------------ Change ------------------------------------ */
        if (gamepad1.x) {
            liftPos = OutreachConstants.liftDrive;
        } else if (gamepad1.y) {
            liftPos = OutreachConstants.liftTop;
        } else if (gamepad1.a) {
            liftPos = OutreachConstants.liftBot;
        }

        if (gamepad1.left_bumper) {
            if (isClawBotA) {
                clawPos = OutreachConstants.clawCloseA;
            } else {
                clawPos = OutreachConstants.clawCloseB;
            }
        } else if (gamepad1.right_bumper) {
            if (isClawBotA) {
                clawPos = OutreachConstants.clawOpenA;
            } else {
                clawPos = OutreachConstants.clawOpenB;
            }
        }

        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        if (rt > 0.3) {
            liftPos += rt * OutreachConstants.liftRatio;
        } else if (lt > 0.3) {
            liftPos -= lt * OutreachConstants.liftRatio;
        }

        if (liftPos > OutreachConstants.liftBot) {
            liftPos = OutreachConstants.liftBot;
        } else if (liftPos < OutreachConstants.liftTop) {
            liftPos = OutreachConstants.liftTop;
        }

        if (aPause == 0) {
            if (gamepad1.b) {
                isClawBotA = !isClawBotA;
            }
        } else {
          aPause++;
          aPause %= OutreachConstants.buttonDelay;
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

