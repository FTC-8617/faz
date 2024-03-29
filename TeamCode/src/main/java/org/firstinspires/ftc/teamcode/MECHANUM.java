package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="MECHANUM", group="TeleOp")
public class MECHANUM extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;
    private final double driveAdjuster = 1;
    double  elevPower = 0;
    @Override
    public void runOpMode()  throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        frontLeft = hardwareMap.dcMotor.get("front left");
        backLeft = hardwareMap.dcMotor.get("back left");
        frontRight = hardwareMap.dcMotor.get("front right");
        backRight = hardwareMap.dcMotor.get("back right");
        leftServo = hardwareMap.crservo.get("left");
        rightServo = hardwareMap.crservo.get("right");
        elevator = hardwareMap.dcMotor.get("elevator");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);/*fr4fl3bl1br2*/
        
        elevator.setDirection(DcMotor.Direction.FORWARD);

        leftServo.setDirection(CRServo.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.FORWARD);

        
        waitForStart();

        while (opModeIsActive()) {

            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double v1 = r * Math.cos(robotAngle) - 0.3*rightX;
            double v2 = r * Math.sin(robotAngle) + 0.3*rightX;
            double v3 = r * Math.sin(robotAngle) - 0.3*rightX;
            double v4 = r * Math.cos(robotAngle) + 0.3*rightX;
            
            if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {

                frontLeft.setPower(-v3);
                frontRight.setPower(-v4);
                backLeft.setPower(-v1);
                backRight.setPower(-v2);
            }

            else if(gamepad1.left_bumper) {
                frontLeft.setPower(-0.75);
                backLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backRight.setPower(0.75);
            }
          
            else if(gamepad1.right_bumper) {
                frontLeft.setPower(0.75);
                backLeft.setPower(0.75);
                frontRight.setPower(-0.75);
                backRight.setPower(-0.75);
            }
            else if(gamepad1.right_trigger > 0.05) {
                double power = 0.5*gamepad1.right_trigger;
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
            }
            else if(gamepad1.left_trigger > 0.05) {
                double power = 0.5*gamepad1.left_trigger;
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            }
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            // Spins wheels out
            if(gamepad2.right_trigger > 0.05){
                double servoPower = 0.3*gamepad2.right_trigger;
                rightServo.setPower(servoPower);
                leftServo.setPower(servoPower);
            }
            // Spins wheels in
            else if(gamepad2.left_trigger > 0.05){
                double servoPower = 0.33*gamepad2.left_trigger;
                rightServo.setPower(-servoPower);
                leftServo.setPower(-servoPower);
            }
            else{
                rightServo.setPower(0);
                leftServo.setPower(0);
            }
            
            if(gamepad2.right_stick_y > 0.15 || gamepad2.right_stick_y < -0.15){
                elevPower=(gamepad2.right_stick_y);
            }

            else{
                elevPower=0;
            }
            elevator.setPower(elevPower);
            
            


        }
    }

}
