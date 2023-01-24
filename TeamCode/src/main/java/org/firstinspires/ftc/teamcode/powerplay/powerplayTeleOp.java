package org.firstinspires.ftc.teamcode.powerplay;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
    private boolean liftModeUpdate = false;
    private boolean liftUseEnc = true;
    private double rotatePower = 0;
    private int rotateTarget = 0;
    private boolean useRotatePower = true;
    private int cycleLiftPos = Constants.liftHigh;
    private double cycleSlidePos = Constants.slideMed;
    private int cycleRPos = Constants.rot180R;
    private double cycleSlideGrabPos = Constants.slideMed;
    private ElapsedTime posSave = new ElapsedTime();
    private double retractWait = Double.MAX_VALUE;

    private boolean autoGrab = true;
    private boolean limits = true;

    private double currentLiftPosition = 0;
    private double currentSlidePosition = Constants.slideIn;
    private double currentRPosition = 0;

    // Servos
    private double slidePosition = Constants.slideIn;
    private double clawPosition = Constants.clawOpen;

    private int ltrigchill = Constants.buttonDelay;
    private int rtrigchill = Constants.buttonDelay;
    private int dpadrchill = Constants.buttonDelay;
    private int dpadlchill = Constants.buttonDelay;
    private int limitbumpchill = Constants.buttonDelay;
    private int stage = -1;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        currentSlidePosition = myRobot.slideServo.getPosition();
        slidePosition = myRobot.slideServo.getPosition();
        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
//        if ((myRobot.getClawDistance() <= Constants.autoClawClose) && (currentLiftPosition > Constants.liftFloor) && autoGrab) {
//            clawPosition = Constants.clawClose;
//            autoGrab = false;
//        }
//        if (!autoGrab && myRobot.getClawDistance() >= Constants.autoClawReset) {
//            autoGrab = true;
//        }

        if (gamepad2.right_bumper) {
            clawPosition = Constants.clawOpen;
        } else if (gamepad2.left_bumper) {
            clawPosition = Constants.clawClose;
        }

        if (gamepad2.dpad_right && dpadrchill == Constants.buttonDelay) {
            useRotatePower = true;
            slidePosition = Math.min(Constants.slideIn, cycleSlidePos);
            myRobot.setSlideServo(slidePosition);
            if (cycleRPos == Constants.rot180R) {
                if (Math.abs(currentRPosition - Constants.rot180L) < (Math.abs(currentRPosition - Constants.rot180R))) {
                    rotateTarget = Constants.rot180L;
                } else {
                    rotateTarget = Constants.rot180R;
                }
            } else {
                rotateTarget = cycleRPos;
            }
            stage = 1;
            dpadrchill = 0;
        } else if (gamepad2.dpad_left && dpadlchill == Constants.buttonDelay) {
            slidePosition = Constants.slideIn;
            myRobot.setSlideServo(slidePosition);
            retractWait = runtime.milliseconds();
            stage = -2;
            dpadlchill = 0;
        }
        if (dpadrchill < Constants.buttonDelay) {
            dpadrchill++;
        }
        if (dpadlchill < Constants.buttonDelay) {
            dpadlchill++;
        }
        if ((retractWait != Double.MAX_VALUE) && (runtime.milliseconds() - retractWait >= Constants.slideRetractWait)) {
            useRotatePower = false;
            rotateTarget = 0;
            useLiftPower = false;
            liftTarget = Constants.liftSpin;
            liftUseEnc = true;
            retractWait = Double.MAX_VALUE;
        }
        if ((stage == -2) && (Math.abs(currentRPosition) < Constants.rotFrontBuffer)){
            liftTarget = 0;
            liftUseEnc = true;
            stage = -1;
        }

        if (stage > 0) {
            switch (stage) {
                case 1:
                    // Raise up
                    if (currentLiftPosition < Constants.liftSpin) {
                        stage = 2;
                    } else {
                        liftTarget = cycleLiftPos;
                        useLiftPower = false;
                        liftUseEnc = true;
                    }
                    break;
                case 2:
                    // Turn arm
                    if (Math.abs(currentRPosition - rotateTarget) <= Constants.rotExtendScale * Constants.rotTolerance) {
                        stage = 3;
                    } else {
                        useRotatePower = false;
                    }
                    break;
                case 3:
                    slidePosition = cycleSlidePos;
                    stage = -1;
            }
        }


        if (gamepad2.a) {
            slidePosition = cycleSlideGrabPos;
        } else if (gamepad2.b) {
            slidePosition = Constants.slideIn;
        }
        double extJoystick = -gamepad2.right_stick_y;
        if (extJoystick > 0.2) {
            // User trying to slide out by pushing the joystick up
            if ((slidePosition + Constants.slideSpeed) < Constants.slideOut) {
                slidePosition += Constants.slideSpeed * extJoystick;
            } else {
                slidePosition = Constants.slideOut;
            }
        } else if (extJoystick < -0.2) {
            // User trying to slide in by pushing the joystick down
            if ((slidePosition - Constants.slideSpeed) > Constants.slideIn) {
                slidePosition += Constants.slideSpeed * extJoystick;
            } else {
                slidePosition = Constants.slideIn;
            }
        }


        // Lifting by Position
        if (gamepad2.dpad_up) {
            useLiftPower = false;
            liftTarget = Constants.liftHigh;
            liftUseEnc = true;
        } else if (gamepad2.dpad_down) {
            useLiftPower = false;
            liftTarget = 0;
            liftUseEnc = true;
        }

        if (gamepad2.left_stick_button) {
            useLiftPower = false;
            liftTarget = Constants.liftDrive;
            liftUseEnc = true;
        }


        // Raising lift by power
        double liftJoystick = gamepad2.left_stick_y;
        if (liftJoystick < -0.12) {
            liftUseEnc = true;
            // user trying to lift up
            if (currentLiftPosition > Constants.liftMax || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftJoystick > 0.12) {
            liftUseEnc = true;
            // user trying to lift down
            if (currentLiftPosition < Constants.liftMin || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftDownRatio;
                if (currentLiftPosition > Constants.liftSlow) {
                    liftPower *= Constants.liftSlowRatio;
                }
            } else {
                liftPower = 0;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }


        // Reset lift encoder or hold lift in place
        if (gamepad2.x) {
            liftUseEnc = true;
            useLiftPower = true;
            myRobot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            useRotatePower = false;
//            if (Math.abs(currentRPosition - Constants.rot180L) < (Math.abs(currentRPosition - Constants.rot180R))) {
//                rotateTarget = Constants.rot180L;
//            } else {
//                rotateTarget = Constants.rot180R;
//            }
        } else if (gamepad2.y) {
            useLiftPower = false;
            liftUseEnc = true;
            liftTarget = (int) currentLiftPosition;
//            useRotatePower = false;
//            rotateTarget = 0;
        }

        if (gamepad2.left_trigger > 0.75 && ltrigchill == Constants.buttonDelay) {
            cycleLiftPos = (int)currentLiftPosition;
            cycleRPos = (int)currentRPosition;
            cycleSlidePos = currentSlidePosition;
            posSave.reset();
            ltrigchill = 0;
        } else if (gamepad2.right_trigger > 0.75 && rtrigchill == Constants.buttonDelay) {
            cycleSlideGrabPos = currentSlidePosition;
            rtrigchill = 0;
        }
        if (ltrigchill < Constants.buttonDelay) {
            ltrigchill++;
        }
        if (rtrigchill < Constants.buttonDelay) {
            rtrigchill++;
        }

        double armRJoystick = gamepad2.right_stick_x;
        if (armRJoystick > 0.5 && currentRPosition < Constants.rotRLimit) {
            useRotatePower = true;
//            armDirection = rotateD.RANDOM;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatePower = armRJoystick * Constants.setRotateMultiplier;
        } else if (armRJoystick < -0.5 && currentRPosition > -Constants.rotRLimit) {
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
            liftUseEnc = true;
            liftPower = 0;
            stage = -1;
            slidePosition = currentSlidePosition;
        }

        // Commented so the robot doesnt EXPLODE mid game
        if (gamepad1.right_bumper && limitbumpchill == Constants.buttonDelay) {
            limits = !limits;
            limitbumpchill = 0;
        }
        if (limitbumpchill < Constants.buttonDelay) {
            limitbumpchill++;
        }

        /* ------------------------------------ Action ------------------------------------ */

        // Actually moves the claw and extension
        myRobot.setClawServo(clawPosition);
        myRobot.setSlideServo(slidePosition);

        if (liftModeUpdate && liftUseEnc) {
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftModeUpdate = false;
        }

        if (useLiftPower) {
            myRobot.runLiftMotor(liftPower);
        } else {
            // Limit so the lift doesnt go down ontop of the base
//            if (liftTarget < Constants.liftSpin && Math.abs(currentRPosition) > 125) {
//                useLiftPower = true;
//                liftPower = 0;
//            } else {
            setLiftMotor(liftTarget);
//            }
        }

        if (useRotatePower) {
            myRobot.runRotateMotor(rotatePower);
        } else {
            // Limit so the turntable doesnt spin unless the slide is extended
//            if (currentLiftPosition > Constants.liftSpin ||
//                    currentSlidePosition > Constants.slideSpin ||
//                    Math.abs(currentRPosition - rotateTarget) <= 125) {
            setRotationPositionPID(rotateTarget, Constants.rotTolerance);
//            }
        }



        /* ------------------------------------ Telemetry ------------------------------------ */

        // Telemetry is for debugging
        telemetry.addData("extension position", currentSlidePosition);
        telemetry.addData("lift position", currentLiftPosition);
        telemetry.addData("lift speed", myRobot.liftMotor.getVelocity());
        telemetry.addData("rotate position", currentRPosition);
        telemetry.addData("distance sensor", myRobot.getClawDistance());
        telemetry.addData("position last saved", posSave.seconds());
        telemetry.addData("stage", stage);
        telemetry.addData("limits", limits);
        telemetry.update();

        Log.d("lift speed", "" + myRobot.liftMotor.getVelocity());
//        Log.d("AHHHHHH extender", String.valueOf(slidePosition));
//        Log.d("AHHHHHH rotate", String.valueOf(currentRPosition));
//        Log.d("AHHHHHH lift", String.valueOf(currentLiftPosition));
//        Log.d("AHHHHH imu: ", "" + myRobot.getAngle());
    }

    @Override
    public void stop() {
    }

    void setLiftMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentLiftPosition) / Constants.liftMax;
        //Initial Time
        telemetry.addData("1", "error: " + error);
        if (Math.abs(error) > (Constants.liftTolerance / -Constants.liftMax)) {
            //Setting p action
            newPower = error * Constants.liftkPTele;
            powDir = Math.signum(error);
            newPower = Math.min(Math.abs(newPower), 1);

            // Going down
            if (powDir == 1) {
                newPower = Math.max(newPower * Constants.liftDownRatio, Constants.liftMinPow);
                if (currentLiftPosition > Constants.liftSlow) {
                    newPower *= Constants.liftSlowRatio;
                }
            }
            // Going up
            else {
                newPower = Math.min(-newPower, -Constants.liftMinPow - Constants.liftkF * currentLiftPosition / Constants.liftMax);
            }
            telemetry.addData("Lift Motor", newPower);
            myRobot.runLiftMotor(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            myRobot.setLiftMotor(0.5, position);
            liftModeUpdate = true;
            liftUseEnc = false;
        } else if (!liftModeUpdate){
            myRobot.runLiftMotor(0);
        }
    }

    public void setRotationPosition(double speed, int position) {
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
        } else {
            useRotatePower = true;
            myRobot.runRotateMotor(0);
        }
    }
}

//we need a higher success rate on