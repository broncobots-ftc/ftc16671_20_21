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

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Timer;



/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Autonomus Mode", group = "ftc16671")

public class AutonomusMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    /** This is for encoder **/
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    //
    static final int oneRingMinHeight = 180;
    static final int oneRingMaxHeight = 240;

    static final int fourRingsMinHeight = 240;
    static final int fourRingsMaxHeight = 340;

    static final int SECONDS_TO_RUN_RING_DETECTION_FOR = 10;

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     */
    private static final String VUFORIA_KEY =
            "AWrb413/////AAABmQT6xsY2eEeEuRH7ulHkqXaAxt2nbyCB1ZDQfx1F+X80Nz5JjPzStB+GpmAByIBVfrjDCkRdsHsurFZvZruc+Rr8KeaKixYFNtpkbmk9DxNPtR3Tq67CVKTZYC46SR+zghr8zn5nP9NLOHWcVYFcNuTR8rx7R9QzAPlKYX60OHC6OLc5ngylJH/zvESjkSMq/84O68lIfkKVycJ7a8085IQBGfVh/yYEJQg3txuehOK97yTSltcJ8CYiM0qZBVRtGIbS2N6D8IZc8BpyjqTaZ8YZhE2gjCYVtlBKk6pveRidtkb0UA1uVmaVR0B9FeSlzwx8h38nbnIJlJF/WOuXNSApPALRl5wn8FZuY01VnV0s";

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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //This is for mobile phone
        parameters.cameraDirection = CameraDirection.BACK;
        //This is for external camera - MAKE SURE DEVICE NAME IS CORRECT
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     *
     * let's detect rings based on height of the object.
     * @param totalHeight
     * @return int - number of rings
     */
    private int detectRings(float totalHeight){
        int totalRings = 0;
        if(totalHeight > oneRingMinHeight && totalHeight <= oneRingMaxHeight){
            telemetry.addLine("1 Ring Found (%d)");
            totalRings = 1;
        }else if(totalHeight > fourRingsMinHeight && totalHeight < fourRingsMaxHeight){
            telemetry.addLine("4 Rings Found (%d)");
            totalRings = 4;
        }else if(totalHeight > fourRingsMaxHeight){
            telemetry.addLine("Ignore this.. (%d)");
        }else {
            telemetry.addLine("0 Rings Found (%d)");
        }
        return totalRings;
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = mecanumDrive.frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            //newLeftTarget = mecanumDrive.backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = mecanumDrive.frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            //newRightTarget = mecanumDrive.backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            mecanumDrive.frontLeft.setTargetPosition(newLeftTarget);
            mecanumDrive.backLeft.setTargetPosition(newLeftTarget);
            mecanumDrive.frontRight.setTargetPosition(newRightTarget);
            mecanumDrive.backRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            mecanumDrive.setAllMotorsToRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            mecanumDrive.setSpeeds(speed, speed, speed, speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecanumDrive.frontLeft.isBusy() && mecanumDrive.frontRight.isBusy())) {

            }

            // Stop all motion;
            mecanumDrive.frontLeft.setPower(0);
            mecanumDrive.frontRight.setPower(0);
            mecanumDrive.backLeft.setPower(0);
            mecanumDrive.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            mecanumDrive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumDrive.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumDrive.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumDrive.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        mecanumDrive.init(hardwareMap);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        long start = System.currentTimeMillis();
        long end = System.currentTimeMillis();
        int totalRings = 0;
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    end = System.currentTimeMillis();
                    float seconds = (end - start) / 1000F;
                    if (updatedRecognitions != null && seconds <= SECONDS_TO_RUN_RING_DETECTION_FOR) {

                          telemetry.addData("# Object Detected", updatedRecognitions.size());
                          telemetry.addData("# seconds passed : ", seconds);
                          // step through the list of recognitions and display boundary info.
                          int i = 0;
                          for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("Total rings height (%d)", i),"%.03f", recognition.getHeight());
                            float totalHeight=recognition.getHeight();
                            totalRings = detectRings(totalHeight);
                          }
                          telemetry.update();


                    }else{
                        tfod.shutdown();
                    }
                    //
                    moveBasedOnTotalRings(totalRings);
                    break;
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void moveBasedOnTotalRings(int totalRings) {
        //if(totalRings == 0){
            //Strafe right
            mecanumDrive.strafeRight(12, true, 5);
            //Move forward to A
            mecanumDrive.moveForward(12, true, 5);
            //Release wobble
            mecanumDrive.releaseWabble();
        /*}else if(totalRings == 1){
            //Strafe right
            mecanumDrive.strafeRight(10, true, 5);
            //Move forward to A
            mecanumDrive.moveForward(18, true, 5);
            //Strafe left to B
            mecanumDrive.strafeLeft(10, true, 5);
            //Release wobble
            mecanumDrive.releaseWabble();
        }else if(totalRings == 4){
            //Strafe right
            mecanumDrive.strafeRight(12, true, 5);
            //Move forward to A
            mecanumDrive.moveForward(24, true, 5);
            //Release wobble
            mecanumDrive.releaseWabble();
        }*/
    }


}
