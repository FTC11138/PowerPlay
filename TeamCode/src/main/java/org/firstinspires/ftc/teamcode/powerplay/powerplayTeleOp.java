package org.firstinspires.ftc.teamcode.powerplay;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class powerplayTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments myRobot = new Attachments();

    /* ------------------------------------- CONSTANTS ------------------------------------------ */
    // Motors
    private double liftPower = 0;
    private int liftTarget = 0;
    private boolean useLiftPower = true;
    private double rotatePower = 0;
    private int rotateTarget = 0;
    private boolean useRotatePower = true;


    private boolean limits = true;
    private boolean shorten = false;

    private double currentLiftPosition = 0;
    private double currentSlidePosition = Constants.slideResting;
    private double currentClawPosition = Constants.clawOpen;
    private double currentRPosition = 0;

    // Servos
    private double slidePosition = Constants.slideResting;
    private double clawPosition = Constants.clawOpen;

    private int stage = -1;
    private int goalAngle = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        setRotationPosition(0.3, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */

        // Position constants
        currentLiftPosition = myRobot.getLiftMotorPosition();
        currentSlidePosition = myRobot.getSlidePosition();
        currentRPosition = myRobot.getRotationMotorPosition();
        currentClawPosition = myRobot.getClawPosition();

        // Motors
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = Constants.moveSpeed;
        double rotationMultiplier = Constants.rotSpeed;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.3;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 0.3;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);





        /* ------------------------------------ Change ------------------------------------ */

        if (gamepad2.right_bumper) {
            clawPosition = Constants.clawClose;
        } else if (gamepad2.left_bumper) {
            clawPosition = Constants.clawOpen;
        }

        if (gamepad2.a) {
            slidePosition = Constants.slideOut;
        } else if (gamepad2.b) {
            slidePosition = Constants.slideResting;
        }
        double extJoystick = gamepad2.right_stick_y;
        if (extJoystick < -0.2) {
            // User trying to slide out by pushing the joystick up
            if (slidePosition > Constants.slideOut) {
                slidePosition += Constants.slideSpeed * extJoystick;
            }
        } else if (extJoystick > 0.2) {
            // User trying to slide in by pushing the joystick down
            if (slidePosition < Constants.slideIn) {
                slidePosition += Constants.slideSpeed * extJoystick;
            }
        }


        // Lifting by Position
        if (gamepad2.dpad_up) {
            useLiftPower = false;
            liftTarget = Constants.liftHigh;
        } else if (gamepad2.dpad_right) {
//            useLiftPower = false;
//            liftTarget = Constants.liftMed;
        } else if (gamepad2.dpad_left) {
//            useLiftPower = false;
//            liftTarget = Constants.liftLow;
        } else if (gamepad2.dpad_down) {
            useLiftPower = false;
            liftTarget = 0;
        }


        // Raising lift by power
        double liftJoystick = gamepad2.left_stick_y;
        if (liftJoystick < -0.12) {
            // user trying to lift up
            if (currentLiftPosition > Constants.liftMax || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftJoystick > 0.12) {
            // user trying to lift down
            if (currentLiftPosition < Constants.liftMin || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftDownRatio;
            } else {
                liftPower = 0;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }


        // Rotating turntable by position
        if (gamepad2.x) {
            useRotatePower = false;
            if (Math.abs(currentRPosition - Constants.rot180L) < (Math.abs(currentRPosition - Constants.rot180R))) {
                rotateTarget = Constants.rot180L;
            } else {
                rotateTarget = Constants.rot180R;
            }
        } else if (gamepad2.y) {
            useRotatePower = false;
            rotateTarget = 0;
        } else if (gamepad2.left_trigger > 0.75) {
//            useRotatePower = false;
//            rotateTarget = Constants.rotDiagBackL;
        } else if (gamepad2.right_trigger > 0.75) {
//            useRotatePower = false;
//            rotateTarget = Constants.rotDiagBackR;
        }

        // Rotating stuff by power

        /*
        The starting rotate position is 0
        The max rotate position is 4270

        We will limit it from -4270 to 4270
        This is to prevent the wires that are going through
        the turntable from twisting infinitely
         */

        double armRJoystick = gamepad2.right_stick_x;
        if (armRJoystick > 0.25 && currentRPosition < Constants.rotRLimit) {
            useRotatePower = true;
//            armDirection = rotateD.RANDOM;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatePower = armRJoystick * Constants.setRotateMultiplier;
        } else if (armRJoystick < -0.25 && currentRPosition > -Constants.rotRLimit) {
            useRotatePower = true;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatePower = armRJoystick * Constants.setRotateMultiplier;
        } else {
            rotatePower = 0;
        }


        // Stop powers
        if (gamepad1.left_bumper) {
            useRotatePower = true;
            rotatePower = 0;
            useLiftPower = true;
            liftPower = 0;
        }

        if (gamepad1.right_bumper) {
            limits = !limits;
        }

        /* ------------------------------------ Action ------------------------------------ */

        myRobot.setClawServo(clawPosition);
        myRobot.setSlideServo(slidePosition);

        if (useLiftPower) {
            myRobot.runLiftMotor(liftPower);
        } else {
            if (liftTarget < Constants.liftSpin && Math.abs(currentRPosition) > 125) {
                useLiftPower = true;
                liftPower = 0;
            } else {
                setLiftMotor(liftTarget, Constants.liftTolerance, true);
            }
        }

        if (useRotatePower) {
            myRobot.runRotateMotor(rotatePower);
        } else {
            if (currentLiftPosition > Constants.liftSpin ||
                    currentSlidePosition > Constants.slideSpin ||
                    Math.abs(currentRPosition - rotateTarget) <= 125) {
                setRotationPositionPID(rotateTarget, Constants.rotTolerance);
            }
        }



        /* ------------------------------------ Telemetry ------------------------------------ */

        // Telemetry is for debugging

        telemetry.addData("extension position", currentSlidePosition);
        telemetry.addData("lift position", currentLiftPosition);
        telemetry.addData("rotate position", currentRPosition);
        telemetry.addData("claw position", currentClawPosition);
        telemetry.addData("limits", limits);
        telemetry.update();

        Log.d("AHHHHHH extender", String.valueOf(slidePosition));
        Log.d("AHHHHHH rotate", String.valueOf(currentRPosition));
        Log.d("AHHHHHH lift", String.valueOf(currentLiftPosition));
        Log.d("AHHHHH imu: ", "" + myRobot.getAngle());
    }

    @Override
    public void stop() {
    }

    void setLiftMotor(int position, double tolerance, boolean usePID) {
        if (usePID) {
            //Undefined constants
            double newPower;
            //Initial error
            double error = -(position - currentLiftPosition) / Constants.liftMax;
            //Initial Time
            telemetry.addData("1", "error: " + error);
            if (Math.abs(error) > (tolerance / -Constants.liftMax)) {
                //Setting p action
                newPower = Math.max(Math.min(error * Constants.liftkP, 1), -1);
//                Log.d("AHHHHHH liftMotor", "PID newPower: " + newPower);
//                telemetry.addData("liftMotor PID newPower", newPower);

                //Set real power
                newPower = Math.max(Math.abs(newPower), Constants.liftMin) * Math.signum(newPower);
                if (Math.signum(newPower) == 1) {
                    newPower = newPower * Constants.liftDownRatio;
                }
                myRobot.runLiftMotor(newPower);

                //Logging
//                Log.d("AHHHHHH liftMotor", "error: " + (error * Constants.liftMax) + ", power: " + newPower + ", current position: " + currentPosition);
//                return false;
            } else {
                useLiftPower = true;
                myRobot.runLiftMotor(0);
//                return true;
            }
        } else {
            myRobot.setLiftMotor(1, position);
//            return true;
        }
    }

    public void setRotationPosition(double speed, int position) {
        // TODO: safety checks, make sure slide is high enough or extension is out enough

        myRobot.rotateMotor.setPower(speed);
        myRobot.rotateMotor.setTargetPosition(position);
        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRotationPositionPID(int position, double tolerance) {
        //Undefined constants
        double newPower;
        //Initial error
        double error = (position - currentRPosition) / Constants.rotMax;
        //Initial Time
        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("2", "error: " + error);
        if (Math.abs(error) > (tolerance / Constants.rotMax)) {
            //Setting p action
            newPower = Math.max(Math.min(error * Constants.rotkP, 1), -1);
//                Log.d("AHHHHHH liftMotor", "PID newPower: " + newPower);
//                telemetry.addData("liftMotor PID newPower", newPower);

            //Set real power
            newPower = Math.max(Math.abs(newPower), Constants.rotMin) * Math.signum(newPower);
            if (Math.signum(rotatePower) == 1) {
                if (currentRPosition > Constants.rotRLimit) {
                    newPower = 0;
                    useRotatePower = true;
                    rotatePower = 0;
                }
            } else {
                if (currentRPosition < -Constants.rotRLimit) {
                    newPower = 0;
                    useRotatePower = true;
                    rotatePower = 0;
                }
            }
            myRobot.runRotateMotor(newPower);

            //Logging
//                Log.d("AHHHHHH liftMotor", "error: " + (error * Constants.liftMax) + ", power: " + newPower + ", current position: " + currentPosition);
//                return false;
        } else {
//            rotateTarget = -1;
//            setRotationPosition(0.25, position);
            useRotatePower = true;
            myRobot.runRotateMotor(0);
//                return true;
        }
    }
}

//we need a higher success rate on