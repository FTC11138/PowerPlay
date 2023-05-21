package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;

import java.util.HashSet;
import java.util.List;

@Autonomous(name = "RightAutoWithTensorFlow", group = "Linear Opmode")
public class RightAutoWithTensorFlow extends AutonomousMethods{
    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/lite-model_efficientdet_lite0_detection_metadata_1.tflite";
    String label = "clock";
    HashSet<String> labelsDetected = new HashSet<>();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private static final String[] LABELS = {
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "airplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "street sign",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "hat",
            "backpack",
            "umbrella",
            "shoe",
            "eye glasses",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "plate",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "mirror",
            "dining table",
            "window",
            "desk",
            "toilet",
            "door",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "blender",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
            "hair brush"
    };

    private static final String VUFORIA_KEY =
            "AUpeoKP/////AAABmXarRDwQxEWbjqbezmXHfkwA5aEeXNOsiGD31T/itLYty7SNnzDsX5pZm29e0d1DBEKl8c83JY6KumPJUWuHV+pfQArB21d2OVNaAZVw+wozaEQqTkFp1iPNuwDF3pk38unJDYfdG/EIpJrMQS48F3+xs80hhfhRs+6GoTxb3VxuLPxWKn8ZqRLzjsvjDjNo6hLKmqxRPUoKRK81Em/vzL9Wc3Q4MhFHRwNF+lMfu1CbxCOS0NYLjo0cIPTeICgdZ5XPy0gBfSgrmsSi21gcKCiIvnLbKQjDYiBkdqE14kZFRsfQD4QhmRIFt8kGznQ78e4ufC/B7ywJ1HlRystIVjy2l6OhLh4dQbfcND3jNjmL";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {telemetry.update();
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        robot.initialize(hardwareMap, telemetry);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(Constants.magnitude, Constants.aspectRatio);
        }

        waitForStart();

        encoderStrafeDriveInchesRight(3, 0.5);
        encoderStraightDrive(10.0f, 0.5);
        runtime.reset();
        if (opModeIsActive()) {
            telemetry.addData("Before detection: ", label);
            telemetry.update();
            labelsDetected.clear();
            while (getRuntime() <= 20 && !labelsDetected.contains("stop sign") && !labelsDetected.contains("clock") && !labelsDetected.contains("traffic light")) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        //telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            //telemetry.addData(""," ");
                            //telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            //telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            //telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            labelsDetected.add(recognition.getLabel());
                        }
                        //telemetry.update();
                    }
                }
            }
        }

        if (labelsDetected.contains("stop sign") && !labelsDetected.contains("clock") && !labelsDetected.contains("traffic light")) {
            label = "stop sign";
        }
        else if (labelsDetected.contains("traffic light") && !labelsDetected.contains("stop sign") && !labelsDetected.contains("clock")) {
            label = "traffic light";
        }

        //sleep(2000);

        telemetry.addData("Signal after detection: ", label);
        telemetry.update();

        //robot.setClawServo(Constants.clawOpen);
        //sleep(500);
        //robot.setLiftMotor(0.1, -100);
        //sleep(500);

        //encoderStraightDrive(3, 0.2);
        //encoderStrafeDriveInchesRight(3, 0.5);

        //robot.setLiftMotor(0.3, Constants.liftLow);

        encoderStraightDrive(-10.0f, 0.5);
        encoderStraightDrive(33, 0.5);
        //sleep(500);

        //robot.setLiftMotor(0.5, Constants.liftHigh);
        //robot.setRotateMotor(0.5, -40 * Constants.rotMotorPosPerDegree);
        //sleep(2000);
        //robot.setSlideServo(0.33);
        //sleep(2000);
        //robot.setLiftMotor(0.3, Constants.liftHigh + 200);
        //sleep(1000);
        //robot.setClawServo(Constants.clawClose);
        //sleep(1000);
        //robot.setSlideServo(Constants.slideIn);
        //robot.setRotateMotor(0.5, 0);

        //encoderStraightDrive(4, 0.5); // push signal cone ahead
        //sleep(500);

        //encoderStraightDrive(-4, 0.5); // move back
        //sleep(500);

        //robot.setLiftMotor(0.3, 0);
        //sleep(3000);

        if (label == "stop sign") {
            encoderStrafeDriveInchesRight(-27, 0.75);
        } else if (label == "clock") {
            encoderStrafeDriveInchesRight(-8, 0.75);
        } else {
            encoderStrafeDriveInchesRight(11, 0.75);
        }
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLiftMotor(0, 3);
        }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = Constants.minResultConfidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        /** if (labels != null) {
         tfod.loadModelFromFile(TFOD_MODEL_FILE, labels);
         }
         */
    }
}
