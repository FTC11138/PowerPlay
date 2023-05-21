/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashSet;
import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class TensorFlowDetection extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    // private static final String TFOD_MODEL_ASSET = "detect.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/detect.tflite";
    //private static final String TFOD_MODEL_LABELS = "/sdcard/FIRST/tflitemodels/labelmap.txt";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/lite-model_efficientdet_lite0_detection_metadata_1.tflite";
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/lite-model_efficientdet_lite1_detection_metadata_1.tflite";
    // private static final String TFOD_MODEL_LABELS = "/sdcard/FIRST/tflitemodels/labelmapCustom.txt";
    // private String[] labels;
    String finalLabel = "clock";
    HashSet<String> labelsDetected = new HashSet<>();

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



    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUpeoKP/////AAABmXarRDwQxEWbjqbezmXHfkwA5aEeXNOsiGD31T/itLYty7SNnzDsX5pZm29e0d1DBEKl8c83JY6KumPJUWuHV+pfQArB21d2OVNaAZVw+wozaEQqTkFp1iPNuwDF3pk38unJDYfdG/EIpJrMQS48F3+xs80hhfhRs+6GoTxb3VxuLPxWKn8ZqRLzjsvjDjNo6hLKmqxRPUoKRK81Em/vzL9Wc3Q4MhFHRwNF+lMfu1CbxCOS0NYLjo0cIPTeICgdZ5XPy0gBfSgrmsSi21gcKCiIvnLbKQjDYiBkdqE14kZFRsfQD4QhmRIFt8kGznQ78e4ufC/B7ywJ1HlRystIVjy2l6OhLh4dQbfcND3jNjmL";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // read the label map text files.
        // readLabels();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(Constants.magnitude, Constants.aspectRatio);
        }

        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start op mode");
        //telemetry.update();
        //waitForStart();

        resetRuntime();
        if (opModeIsActive()) {
            labelsDetected.clear();
            while (getRuntime() <= 5 && !labelsDetected.contains("stop sign") && !labelsDetected.contains("clock") && !labelsDetected.contains("car")) {
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
    }

    /**
     * Initialize the Vuforia localization engine.
     */
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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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
    public String getLabel() {
        runOpMode();
        if (labelsDetected.contains("stop sign") && !labelsDetected.contains("clock") && !labelsDetected.contains("car")) {
            finalLabel = "stop sign";
        }
        else if (labelsDetected.contains("clock") && !labelsDetected.contains("stop sign") && !labelsDetected.contains("car")) {
            finalLabel = "clock";
        }
        return finalLabel;
    }

    /**
     * Read the labels for the object detection model from a file.
     */

    /**
    private void readLabels() {
        ArrayList<String> labelList = new ArrayList<>();

        // try to read in the the labels.
        try (BufferedReader br = new BufferedReader(new FileReader(TFOD_MODEL_LABELS))) {
            int index = 0;
            while (br.ready()) {
                // skip the first row of the labelmap.txt file.
                // if you look at the TFOD Android example project (https://github.com/tensorflow/examples/tree/master/lite/examples/object_detection/android)
                // you will see that the labels for the inference model are actually extracted (as metadata) from the .tflite model file
                // instead of from the labelmap.txt file. if you build and run that example project, you'll see that
                // the label list begins with the label "person" and does not include the first line of the labelmap.txt file ("???").
                // i suspect that the first line of the labelmap.txt file might be reserved for some future metadata schema
                // (or that the generated label map file is incorrect).
                // for now, skip the first line of the label map text file so that your label list is in sync with the embedded label list in the .tflite model.
                if(index == 0) {
                    // skip first line.
                    br.readLine();
                } else {
                    labelList.add(br.readLine());
                }
                index++;
            }
        } catch (Exception e) {
            telemetry.addData("Exception", e.getLocalizedMessage());
            telemetry.update();
        }

        if (labelList.size() > 0) {
            labels = getStringArray(labelList);
            RobotLog.vv("readLabels()", "%d labels read.", labels.length);
            for (String label : labels) {
                RobotLog.vv("readLabels()", " " + label);
            }
        } else {
            RobotLog.vv("readLabels()", "No labels read!");
        }
    }

    // Function to convert ArrayList<String> to String[]
    private String[] getStringArray(ArrayList<String> arr)
    {
        // declaration and initialize String Array
        String str[] = new String[arr.size()];

        // Convert ArrayList to object array
        Object[] objArr = arr.toArray();

        // Iterating and converting to String
        int i = 0;
        for (Object obj : objArr) {
            str[i++] = (String)obj;
        }

        return str;
    }
     */
}
