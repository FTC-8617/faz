package org.firstinspires.ftc.teamcode;/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@Autonomous(name = "AutoCameraLeft", preselectTeleOp = "MECHANUM")
public class AutoCameraLeft extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DcMotor BackrightAsDcMotor = null;
    private DcMotor FrontrightAsDcMotor = null;
    private DcMotor FrontleftAsDcMotor = null;
    private DcMotor BackleftAsDcMotor = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int left =1;
    int middle = 2;
    int right =3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
        BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
        FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
        FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
        leftServo = hardwareMap.crservo.get("left");
        rightServo = hardwareMap.crservo.get("right");
        elevator = hardwareMap.dcMotor.get("elevator");

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        /* Actually do something useful */
        in(0.1);
        if (tagOfInterest == null) {
            telemetry.addLine("ain't nothing here");
            telemetry.update();
        } else if (tagOfInterest.id == left) {
            crabRight(2000, 0.3);
            sleep(4000);
            forward(1200, 0.3);
            sleep(2000);
            elevatorUp(4000, 0.75);
            sleep(3000);
            forward(100, 0.25);
            forward(175, 0.15);
            sleep(650);
            drop(1.0);
            sleep(2000);
            backward(100,0.25);
            sleep(2000);
            elevatorDown(3000, 0.50);
            sleep(2000);
            crabLeft(3125, 0.5);
            sleep(3000);
            telemetry.update();
        } else if (tagOfInterest.id == middle) {
            crabRight(2000, 0.3);
            sleep(4000);
            forward(1200, 0.3);
            sleep(2000);
            elevatorUp(4000, 0.75);
            sleep(3000);
            forward(100, 0.25);
            forward(175, 0.15);
            sleep(650);
            drop(1.0);
            sleep(2000);
            backward(100,0.25);
            sleep(2000);
            elevatorDown(3000, 0.50);
            sleep(2000);
            crabLeft(1725, 0.5);
            sleep(3000);
            telemetry.update();
        } else {
            crabRight(2000, 0.3);
            sleep(4000);
            forward(1200, 0.3);
            sleep(3000);
            elevatorUp(4000, 0.75);
            sleep(3000);
            forward(100, 0.25);
            forward(175, 0.15);
            sleep(650);
            drop(1.0);
            sleep(2000);
            backward(100,0.25);
            sleep(2000);
            elevatorDown(3000, 0.5);
            sleep(2000);
            crabLeft(650, 0.5);
            sleep(3000);
            telemetry.update();
        }
    }




    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    private void elevatorUp(int distance, double power){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setTargetPosition(distance);

        elevator.setPower(power);

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void elevatorDown(int distance, double power){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        elevator.setTargetPosition(distance);

        elevator.setPower(power);

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void drop(double power){
        rightServo.setPower(power);
        leftServo.setPower(power);
    }

    private void in(double power){
        rightServo.setPower(-power);
        leftServo.setPower(-power);
    }

    private void crabLeft(int distance, double power){
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void crabRight(int distance, double power){
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void turnright(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontleftAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        BackrightAsDcMotor.setTargetPosition(distance);

        FrontrightAsDcMotor.setPower(power);
        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void turnleft(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontleftAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        BackrightAsDcMotor.setTargetPosition(distance);

        FrontrightAsDcMotor.setPower(power);
        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void forward(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void backward(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

