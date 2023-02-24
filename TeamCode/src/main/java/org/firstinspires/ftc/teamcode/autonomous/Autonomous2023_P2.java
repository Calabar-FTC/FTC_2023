package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainConfig2023;


@Autonomous(name="Autonomous P2", group="FTC-2023-Calabar")
public class Autonomous2023_P2 extends LinearOpMode {

    // Declare the main configurations for the code
    private MainConfig2023 config = new MainConfig2023();

    int signal_sleeve_position = 2;

    @Override
    public void runOpMode() {
        // Initialize all the robot configurations
        config.TotalHardwareMap(hardwareMap, telemetry);

        SetEncoderMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // reset the runtime to track the game time
        config.runtime.reset();

        /*
         * ========================
         * AUTONOMOUS
         * SEQUENCE
         * ========================
         * */
        telemetry.addData("Status", "Running");
        telemetry.update();
        // The sequence of commands to be completed in autonomous
        Drive(config.drive_speed,80,5); // move to the cone

        Drive(0.3,5,1); // get closer to cone to adjust for error

//        DetectSignalSymbol(5); // detect the beacon color
        config.delay(1); // wait for a second

        Drive(config.drive_speed,20,5); // Center the robot in position

        //move to the position base on the signal sleeve
        switch (signal_sleeve_position){
            case 1:
                MecanumTurn_left(1,30,5);
                telemetry.addData("POSITION", "DETECTED 1");
                break;
            case 2:
                telemetry.addData("POSITION", "DETECTED 2");
                break;
            case 3:
                MecanumTurn_right(1,30,5);
                telemetry.addData("POSITION", "DETECTED 3");
                break;
            default:

        }

        telemetry.addData("AUTONOMOUS", "ROBOT JOB COMPLETED");
        telemetry.update();
    }

    public void SetEncoderMode(){
        config.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.linslide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.linslide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Drive(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_to_travel = (int) (distance * config.DRIVE_DISTANCE_RATIO);

            // Set the the new target distance
            config.right_1_wheel_target = config.rightDrive.getCurrentPosition() + distance_to_travel;
            config.right_2_wheel_target = config.rightDrive2.getCurrentPosition() + distance_to_travel;
            config.left_1_wheel_target = config.leftDrive.getCurrentPosition() + distance_to_travel;
            config.left_2_wheel_target = config.leftDrive2.getCurrentPosition() + distance_to_travel;

            // Set the target positions to move to
            config.rightDrive.setTargetPosition(config.right_1_wheel_target);
            config.rightDrive2.setTargetPosition(config.right_2_wheel_target);
            config.leftDrive.setTargetPosition(config.left_1_wheel_target);
            config.leftDrive2.setTargetPosition(config.left_2_wheel_target);

            // Set the wheels to move to the position
            config.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            config.rightDrive.setPower(Math.abs(speed));
            config.rightDrive2.setPower(Math.abs(speed));
            config.leftDrive.setPower(Math.abs(speed));
            config.leftDrive2.setPower(Math.abs(speed));


            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.rightDrive.getCurrentPosition() >= config.right_1_wheel_target) {
                    config.WheelBrake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", config.right_1_wheel_target, config.left_1_wheel_target);
                telemetry.addData("Currently at", " at %5d :%5d", config.leftDrive.getCurrentPosition(), config.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            config.WheelBrake();
        }
    }

    public void MecanumTurn_left(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_to_travel = (int) (distance * config.MECANUM_TURN_DISTANCE_RATIO);


            // Set the the new target distance
            config.right_1_wheel_target = config.rightDrive.getCurrentPosition() - distance_to_travel;
            config.right_2_wheel_target = config.rightDrive2.getCurrentPosition() + distance_to_travel;
            config.left_1_wheel_target = config.leftDrive.getCurrentPosition() + distance_to_travel;
            config.left_2_wheel_target = config.leftDrive2.getCurrentPosition() - distance_to_travel;

            // Set the target positions to move to
            config.rightDrive.setTargetPosition(config.right_1_wheel_target);
            config.rightDrive2.setTargetPosition(config.right_2_wheel_target);
            config.leftDrive.setTargetPosition(config.left_1_wheel_target);
            config.leftDrive2.setTargetPosition(config.left_2_wheel_target);

            // Set the wheels to move to the position
            config.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            config.rightDrive.setPower(Math.abs(speed));
            config.rightDrive2.setPower(Math.abs(speed));
            config.leftDrive.setPower(Math.abs(speed));
            config.leftDrive2.setPower(Math.abs(speed));

            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.rightDrive.getCurrentPosition() >= config.right_1_wheel_target) {
                    config.WheelBrake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", config.right_1_wheel_target, config.left_1_wheel_target);
                telemetry.addData("Currently at", " at %5d :%5d", config.leftDrive.getCurrentPosition(), config.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            config.WheelBrake();
        }
    }

    public void MecanumTurn_right(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_to_travel = (int) (distance * config.MECANUM_TURN_DISTANCE_RATIO);

            // Calculate the the new target distance
            config.right_1_wheel_target = config.rightDrive.getCurrentPosition() - distance_to_travel;
            config.right_2_wheel_target = config.rightDrive2.getCurrentPosition() + distance_to_travel;
            config.left_1_wheel_target = config.leftDrive.getCurrentPosition() + distance_to_travel;
            config.left_2_wheel_target = config.leftDrive2.getCurrentPosition() - distance_to_travel;

            // Set the target positions to move to
            config.rightDrive.setTargetPosition(config.right_1_wheel_target);
            config.rightDrive2.setTargetPosition(config.right_2_wheel_target);
            config.leftDrive.setTargetPosition(config.left_1_wheel_target);
            config.leftDrive2.setTargetPosition(config.left_2_wheel_target);

            // Set the wheels to move to the position
            config.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the Wheel speed
            config.rightDrive.setPower(Math.abs(speed));
            config.rightDrive2.setPower(Math.abs(speed));
            config.leftDrive.setPower(Math.abs(speed));
            config.leftDrive2.setPower(Math.abs(speed));


            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.rightDrive.getCurrentPosition() <= config.right_1_wheel_target) {
                    config.WheelBrake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", config.right_1_wheel_target, config.left_1_wheel_target);
                telemetry.addData("Currently at", " at %5d :%5d", config.leftDrive.getCurrentPosition(), config.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            config.WheelBrake();
        }
    }

    public void linslide_up(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_travel = (int) (distance * config.LIFT_DISTANCE_RATIO);

            // Calculate the the new target distance
            config.slide_1_target = config.linslide_right.getCurrentPosition() + distance_travel;
            config.slide_2_target = config.linslide_left.getCurrentPosition() + distance_travel;

            // Set the target positions to move to
            config.linslide_right.setTargetPosition(config.slide_1_target);
            config.linslide_left.setTargetPosition(config.slide_2_target);

            // Set the lift to move to the position
            config.linslide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.linslide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            config.linslide_left.setPower(Math.abs(speed));
            config.linslide_right.setPower(Math.abs(speed));

            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.linslide_left.getCurrentPosition() >= config.slide_2_position) {
                    config.LiftBrake();
                    break;
                }

                telemetry.addData("Running to", " %7d :%7d", config.slide_1_target, config.slide_2_target);
                telemetry.addData("Currently at", " at %5d :%5d", config.linslide_right.getCurrentPosition(), config.linslide_left.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            config.LiftBrake();
        }
    }

    public void linslide_down(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_travel = (int) (distance * config.LIFT_DISTANCE_RATIO);

            // Calculate the the new target distance
            config.slide_1_target = config.linslide_right.getCurrentPosition() - distance_travel;
            config.slide_2_target = config.linslide_left.getCurrentPosition() - distance_travel;

            // Set the target positions to move to
            config.linslide_right.setTargetPosition(config.slide_1_target);
            config.linslide_left.setTargetPosition(config.slide_2_target);

            // Set the lift to move to the position
            config.linslide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.linslide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            config.linslide_left.setPower(Math.abs(speed));
            config.linslide_right.setPower(Math.abs(speed));

            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.linslide_left.getCurrentPosition() >= config.slide_2_position) {
                    config.LiftBrake();
                    break;
                }
                telemetry.addData("Running to", " %7d :%7d", config.slide_1_target, config.slide_2_target);
                telemetry.addData("Currently at", " at %5d :%5d", config.linslide_right.getCurrentPosition(), config.linslide_left.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            config.LiftBrake();

        }
    }

    public void servo_claw(double timeoutS) {
        double distance = 0.4;

        config.servotargetleft = (int) (config.claw_left.getPosition() + distance);
        config.servotargetright = (int) (config.claw_right.getPosition() + distance);

        config.claw_left.setPosition(config.servotargetleft);
        config.claw_right.setPosition(config.servotargetright);

        while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {
            if (config.linslide_left.getCurrentPosition() >= config.slide_2_position) {
                config.LiftBrake();
                break;
            }
        }
    }

    public void DetectSignalSymbol(int number_of_color_tests){
        // Test for the colors 5 times then get the average of the values
        int temp_col_red = 0, temp_col_green=0, temp_col_blue = 0;

        for (int a = 0; a < number_of_color_tests; a++){
            config.colors = config.colorSensor.getNormalizedColors();
            temp_col_red += config.colors.red;
            temp_col_green += config.colors.green;
            temp_col_blue += config.colors.blue;

            config.delay(0.25); // Sleep for 1/4 second before trying again
        }

        // find the average of all the colors
        int col_red = temp_col_red / number_of_color_tests;
        int col_green = temp_col_green / number_of_color_tests;
        int col_blue = temp_col_blue / number_of_color_tests;

        if((col_red > col_green) && (col_red > col_blue)){
            signal_sleeve_position = 3;
        } else if ((col_blue > col_red) && (col_blue > col_green)) {
            signal_sleeve_position = 2;
        } else if ((col_green > col_blue) && (col_green > col_red)) {
            signal_sleeve_position = 1;
        }else{
            signal_sleeve_position = 2;
        }
    }
}






