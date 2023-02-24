package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainConfig2023;


@Autonomous(name="Autonomous baby", group="FTC-2023-Calabar")
public class Autonomous2023 extends LinearOpMode {



    // Declare the main configurations for the code
    private MainConfig2023 config = new MainConfig2023();

    // Reset the encoders for all the motors to run at a constant speed

    @Override
    public void runOpMode() {
        // Initialize all the robot configurations
        config.TotalHardwareMap(hardwareMap, telemetry);

        config.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.linslide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        config.linslide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        config.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        config.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        config.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        config.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        config.linslide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        config.linslide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        Drive(1,120,5);
        MecanumTurn_left(1,120,5);

//            if((config.color.red >config.color.green)&&(config.color.red >config.color.green){
//                MecanumTurn_left(1,120,5);
//
//            }
        if (opModeIsActive()) {


        }
    }

    public void Drive(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int distance_to_travel = (int) (distance * config.DRIVE_DISTANCE_RATIO);

//            int ticks=config.rightDrive.getCurrentPosition();
//            telemetry.addData("Amount per revolution %i",ticks );

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


            config.rightDrive.setPower(Math.abs(config.drive_speed));
            config.rightDrive2.setPower(Math.abs(config.drive_speed));
            config.leftDrive.setPower(Math.abs(config.drive_speed));
            config.leftDrive2.setPower(Math.abs(config.drive_speed));


            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.leftDrive.getCurrentPosition() >= config.right_1_wheel_target) {
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
            int distance_to_travel = (int) (distance * config.DRIVE_DISTANCE_RATIO);

//            int ticks=config.rightDrive.getCurrentPosition();
//            telemetry.addData("Amount per revolution %i",ticks );

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

            config.rightDrive.setPower(Math.abs(config.drive_speed));
            config.rightDrive2.setPower(Math.abs(config.drive_speed));
            config.leftDrive.setPower(Math.abs(config.drive_speed));
            config.leftDrive2.setPower(Math.abs(config.drive_speed));

            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.leftDrive.getCurrentPosition() >= config.right_1_wheel_target) {
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
            config.rightDrive.setPower(Math.abs(config.drive_speed));
            config.rightDrive2.setPower(Math.abs(config.drive_speed));
            config.leftDrive.setPower(Math.abs(config.drive_speed));
            config.leftDrive2.setPower(Math.abs(config.drive_speed));


            while (opModeIsActive() && (config.runtime.seconds() < timeoutS)) {

                if (config.leftDrive.getCurrentPosition() <= config.right_1_wheel_target) {
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

            config.linslide_left.setPower(Math.abs(config.slide_speedy));
            config.linslide_right.setPower(Math.abs(config.slide_speedy));

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

            config.linslide_left.setPower(Math.abs(config.slide_speedy));
            config.linslide_right.setPower(Math.abs(config.slide_speedy));

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

}






