package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;



@Autonomous(name="Autonomous baby", group="Linear Opmode")
public class Autonomous2023 extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftDrive2 = null;
    public DcMotor rightDrive2 = null;

    public DcMotor linslide_left = null;
    public DcMotor linslide_right = null;

    public Servo servo1;
    public Servo servo2;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor sensorRange;


    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV);

    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // Go to your motor vendor website to determine your motor's WHEEL_DIAMETER
    //  Circumfrence of the wheel
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    static final double COUNTS_PER_MOTOR_REV = 500;   // Ultra Planetary Motor Encoder
    static final double WHEEL_DIAMETER_CM = 9.8;     // Mecanum Wheel circumference cm
    static final double distance_per_rev = (WHEEL_DIAMETER_CM * Math.PI); //distance in cm
    static final double DRIVE_SPEED = 1;

    int newLeftTarget;
    int newRightTarget;
    int newLeftTarget2;
    int newRightTarget2;
    double servotargetleft;
    double servotargetright;


    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");

        linslide_left = hardwareMap.get(DcMotor.class, "arm_right");
        linslide_right = hardwareMap.get(DcMotor.class, "arm_left");

        servo1 = hardwareMap.get(Servo.class, "claw_right");
        servo2 = hardwareMap.get(Servo.class, "claw_left");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        linslide_left.setDirection(DcMotor.Direction.REVERSE);
        linslide_right.setDirection(DcMotor.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linslide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linslide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        servo1.
//        servo2.setDirection(Servo.Direction.REVERSE);



        waitForStart();

        servo_up (5);
        //Drive_Foward(DRIVE_SPEED, 320, 10);
        //linslide_up(0.7,13, 10);
        //Drive_backward(DRIVE_SPEED,70 , 10);
        //Turn_right (DRIVE_SPEED, 30,10 );
        //linslide_down(0.7,13, 10);
        //Turn_left (DRIVE_SPEED, 70,10 );




        //Turn_left (DRIVE_SPEED, 900,2 );
        runtime.reset();
        telemetry.update();
    }

    public void nuetral()
    {
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void brake()
    {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Drive_Foward(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_travel = (int) ((distance / distance_per_rev) * COUNTS_PER_MOTOR_REV);
            newLeftTarget = leftDrive.getCurrentPosition();
            newRightTarget = rightDrive.getCurrentPosition();
            newLeftTarget2 = leftDrive2.getCurrentPosition();
            newRightTarget2 = rightDrive2.getCurrentPosition();

            leftDrive.setTargetPosition(newLeftTarget + distance_travel);
            rightDrive.setTargetPosition(newRightTarget + distance_travel);
            leftDrive2.setTargetPosition(newLeftTarget2 + distance_travel);
            rightDrive2.setTargetPosition(newRightTarget2 + distance_travel);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            leftDrive2.setPower(Math.abs(speed));
            rightDrive2.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS))
            {

                if(leftDrive.getCurrentPosition() >= newLeftTarget +distance_travel)
                {
                    brake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %5d :%5d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
    }

    public void Drive_backward(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_travel = (int) ((distance / distance_per_rev) * COUNTS_PER_MOTOR_REV);
            newLeftTarget = leftDrive.getCurrentPosition();
            newRightTarget = rightDrive.getCurrentPosition();
            newLeftTarget2 = leftDrive2.getCurrentPosition();
            newRightTarget2 = rightDrive2.getCurrentPosition();

            leftDrive.setTargetPosition(newLeftTarget - distance_travel);
            rightDrive.setTargetPosition(newRightTarget - distance_travel);
            leftDrive2.setTargetPosition(newLeftTarget2 - distance_travel);
            rightDrive2.setTargetPosition(newRightTarget2 - distance_travel);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            leftDrive2.setPower(Math.abs(speed));
            rightDrive2.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (leftDrive.getCurrentPosition() >= newLeftTarget + distance_travel) {
                    brake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %5d :%5d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void Turn_left ( double speed, double distance, double timeoutS){

        if (opModeIsActive()) {
            int distance_travel = (int) ((distance / distance_per_rev) * COUNTS_PER_MOTOR_REV);
            newLeftTarget = leftDrive.getCurrentPosition();
            newRightTarget = rightDrive.getCurrentPosition();
            newLeftTarget2 = leftDrive.getCurrentPosition();
            newRightTarget2 = rightDrive2.getCurrentPosition();

            leftDrive.setTargetPosition(newLeftTarget - distance_travel);
            rightDrive.setTargetPosition(newRightTarget + distance_travel);
            leftDrive2.setTargetPosition(newLeftTarget2 + distance_travel);
            rightDrive2.setTargetPosition(newRightTarget2 - distance_travel);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            leftDrive2.setPower(Math.abs(speed));
            rightDrive2.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (leftDrive.getCurrentPosition() >= newLeftTarget + distance_travel)
                {
                    brake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %5d :%5d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    public void Turn_right ( double speed, double distance, double timeoutS){

        if (opModeIsActive()) {
            int distance_travel = (int) ((distance / distance_per_rev) * COUNTS_PER_MOTOR_REV);
            newLeftTarget = leftDrive.getCurrentPosition();
            newRightTarget = rightDrive.getCurrentPosition();
            newLeftTarget2 = leftDrive.getCurrentPosition();
            newRightTarget2 = rightDrive2.getCurrentPosition();

            leftDrive.setTargetPosition(newLeftTarget + distance_travel);
            rightDrive.setTargetPosition(newRightTarget -distance_travel);
            leftDrive2.setTargetPosition(newLeftTarget2 - distance_travel);
            rightDrive2.setTargetPosition(newRightTarget2 + distance_travel);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            leftDrive2.setPower(Math.abs(speed));
            rightDrive2.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (leftDrive.getCurrentPosition() >= newLeftTarget + distance_travel)
                {
                    brake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %5d :%5d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void linslide_up ( double speed, double distance, double timeoutS){

        if (opModeIsActive()) {
            double COUNTS_PER_LIN_MOTOR_REV = 3000;   // Ultra Planetary Motor Encoder
            double SPLOON_DIAMETER_CM = 3;     // Mecanum Wheel circumference cm
            double distance_per_lin_rev = (SPLOON_DIAMETER_CM  * Math.PI); //distance in cm
            double DRIVE_SPEED = 0.5;

            int distance_travel = (int) ((distance / distance_per_lin_rev)* COUNTS_PER_LIN_MOTOR_REV);
            newLeftTarget = linslide_left.getCurrentPosition();
            newRightTarget = linslide_right.getCurrentPosition();

            linslide_left.setTargetPosition(newLeftTarget - distance_travel);
            linslide_right.setTargetPosition(newRightTarget - distance_travel);

            linslide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linslide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            linslide_left.setPower(Math.abs(speed));
            linslide_right.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (linslide_left.getCurrentPosition() >= newLeftTarget + distance_travel)
                {
                    brake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %5d :%5d", linslide_left.getCurrentPosition(), linslide_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            linslide_left.setPower(0);
            linslide_right.setPower(0);

            linslide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linslide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }

    public void linslide_down ( double speed, double distance, double timeoutS){

        if (opModeIsActive()) {
            double COUNTS_PER_LIN_MOTOR_REV = 3000;   // Ultra Planetary Motor Encoder
            double SPLOON_DIAMETER_CM = 3;     // Mecanum Wheel circumference cm
            double distance_per_lin_rev = (SPLOON_DIAMETER_CM  * Math.PI); //distance in cm
            double DRIVE_SPEED = 0.5;

            int distance_travel = (int) ((distance / distance_per_lin_rev)* COUNTS_PER_LIN_MOTOR_REV);
            newLeftTarget = linslide_left.getCurrentPosition();
            newRightTarget = linslide_right.getCurrentPosition();

            linslide_left.setTargetPosition(newLeftTarget + distance_travel);
            linslide_right.setTargetPosition(newRightTarget - distance_travel);

            linslide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linslide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            linslide_left.setPower(Math.abs(speed));
            linslide_right.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (linslide_left.getCurrentPosition() >= newLeftTarget + distance_travel)
                {
                    brake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %5d :%5d", linslide_left.getCurrentPosition(), linslide_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            linslide_left.setPower(0);
            linslide_right.setPower(0);

            linslide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linslide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    public void servo_up (double timeoutS){

            servotargetleft=  servo1.getPosition();
            servotargetright= servo2.getPosition();

            servo1.setPosition(servotargetleft - 0.1);
            servo2.setPosition(servotargetright + 0.1);
    }

    /**
    public boolean isopen(){
        servotargetleft=  servo1.getPosition();
        servotargetright= servo2.getPosition();


        if(servotargetleft && servotargetright> 0){
            return true;
        }else{
           return false;
        }
    }
     **/

}






