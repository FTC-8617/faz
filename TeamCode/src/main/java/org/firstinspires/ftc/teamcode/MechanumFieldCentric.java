package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@TeleOp(name="MechanumFieldCentric(doesn't work)", group="TeleOp")
public class MechanumFieldCentric extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;

    private final double driveAdjuster = 1;

    Orientation angles;

    public BNO055IMU imu;


    double  elevPower = 0;
    @Override
    public void runOpMode()  throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        frontLeft = hardwareMap.dcMotor.get("front left");
        backLeft = hardwareMap.dcMotor.get("back left");
        frontRight = hardwareMap.dcMotor.get("front right");
        backRight = hardwareMap.dcMotor.get("back right");
        //leftServo = hardwareMap.crservo.get("left");
        //rightServo = hardwareMap.crservo.get("right");
        //elevator = hardwareMap.dcMotor.get("elevator");

        //fixing all the directions

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        
        //elevator.setDirection(DcMotor.Direction.REVERSE);
        
        //leftServo.setDirection(CRServo.Direction.REVERSE);
        //rightServo.setDirection(CRServo.Direction.FORWARD);
        
        waitForStart();

        while (opModeIsActive()) {

            //ok srs math time

            //left stick x and y
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            //left stick hypotenuse (this range.clip thing keeps it in a positive range )
            double r = Range.clip(Math.hypot(leftX, leftY), 0, 1);
            //left stick angle
            double gamepadAngle = Math.atan2(leftY, leftX);
            double robotAngle = getAngle();
            double movementAngle = gamepadAngle - robotAngle;
            double xControl = (Math.cos(Math.toRadians(movementAngle)) * r) * Math.abs((Math.cos(Math.toRadians(movementAngle)) * r));
            double yControl = (Math.sin(Math.toRadians(movementAngle)) * r) * Math.abs((Math.cos(Math.toRadians(movementAngle)) * r));

            //oh yeah we need the right stick so we can turn
            double rightX= gamepad1.right_stick_x;

            // *ACTUAL MATH* calculating power for da wheels
            final double v1 = yControl - xControl + rightX;
            final double v2 = yControl + xControl + rightX;
            final double v3 = yControl + xControl - rightX;
            final double v4 = yControl - xControl - rightX;
            

            //OMG first driver stuff

            //setting calculated power to wheels
            if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
                frontRight.setPower(v1);
                backRight.setPower(v2);
                frontLeft.setPower(v3);
                backLeft.setPower(v4);
            }


            //Spins da robot left
            else if(gamepad1.left_bumper) {
                frontLeft.setPower(0.5);
                backLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backRight.setPower(-0.5);
            }


            //Spins da robot right
            else if(gamepad1.right_bumper) {
                frontLeft.setPower(-0.5);
                backLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backRight.setPower(0.5);
            }


            //Stops all movement when nothing is being pushed
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }


            //OMG second driver stuff

/*
            //yay servos
            if(gamepad2.right_bumper){
                rightServo.setPower(1);
                leftServo.setPower(1);
            }
            else if(gamepad2.left_bumper){
                rightServo.setPower(-1);
                leftServo.setPower(-1);
            }
            else{
                rightServo.setPower(0);
                leftServo.setPower(0);
            }
            
            //elevator time
            if(gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1){
                elevPower=(gamepad2.right_stick_y);
            }
            else{
                elevPower=0;
            }
            elevator.setPower(elevPower);
 */
            


        }
    }
    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //gravity = imu.getGravity();
            }
        });
    }
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}
