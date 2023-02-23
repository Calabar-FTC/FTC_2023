package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MainConfig2023 {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDrive = null;
    public DcMotor leftDrive2 = null;
    public DcMotor rightDrive = null;
    public DcMotor rightDrive2 = null;
    public DcMotor linslide_left = null;
    public DcMotor linslide_right = null;
    public Servo claw_right = null;
    public Servo claw_left = null;

    public NormalizedColorSensor colorSensor;
    public DistanceSensor sensorRange;
    public DigitalChannel MagneticLimitSwitch;  // Hardware Device Object


    public HardwareMap hardMap;

    public Telemetry telemetry;

    // Variables to configure the robot and to assist with functionality
    public static final double WHEELS_COUNTS_PER_MOTOR_REV = 500; // Ultra Planetary Motor Encoder
    public static double WHEEL_DIAMETER_CM = 9.8; // Mecanum Wheel circumference cm
    public static final double DISTANCE_PER_REV = (WHEEL_DIAMETER_CM * Math.PI); //distance travel in cm
    public static final double DRIVE_DISTANCE_RATIO = WHEELS_COUNTS_PER_MOTOR_REV/DISTANCE_PER_REV;
    public static final double MECANUM_TURN_DISTANCE_RATIO = WHEELS_COUNTS_PER_MOTOR_REV/DISTANCE_PER_REV;

    public static final double COUNTS_PER_LIN_MOTOR_REV = 3000;  //  Planetary Motor Encoder
    public static final double SPLOON_DIAMETER_CM = 3;     // lift spool circumference cm
    public static final double distance_per_lin_rev = (SPLOON_DIAMETER_CM  * Math.PI); //distance in cm
    public static final double LIFT_DISTANCE_RATIO = COUNTS_PER_LIN_MOTOR_REV/distance_per_lin_rev;





    // Robot hardware variables
    public double right_power_1, left_power_1, right_power_2, left_power_2 = 0;
    public double mecanum_power = 0;
    public double drive_speed = 0;
    public double slide_speedy = 0;
    public double slide_power_1, slide_power_2 = 0;


    public int right_1_wheel_position, right_2_wheel_position, left_1_wheel_position, left_2_wheel_position = 0;
    public int right_1_wheel_target, right_2_wheel_target, left_1_wheel_target, left_2_wheel_target = 0;

    public int slide_1_position, slide_2_position = 0;
    public int slide_1_target, slide_2_target = 0;


    //robot controller variables
    public boolean mecanumright, mecanumleft = false;
    public boolean bump_right, bump_left = false;
    public boolean m_switch, claw_state=false;
    public float right_trig, left_trig = 0;
    public double servotargetleft;
    public double servotargetright;

    public double drive_speed_transmission_limiter = 0;
    public double lift_speed_transmission_limiter = 0;

    public boolean override_all_automation = false;


    public void TotalHardwareMap (HardwareMap hardMap, Telemetry telemetry)
    {
        // Connects the main config hardware map with the running hardwaremap
        this.hardMap = hardMap;
        this.telemetry = telemetry;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = this.hardMap.get(DcMotor.class, "left_drive");
        leftDrive2 = this.hardMap.get(DcMotor.class, "left_drive2");
        rightDrive = this.hardMap.get(DcMotor.class, "right_drive");
        rightDrive2 = this.hardMap.get(DcMotor.class, "right_drive2");

        linslide_left = this.hardMap.get(DcMotor.class, "arm_right");
        linslide_right = this.hardMap.get(DcMotor.class, "arm_left");

        claw_right = this.hardMap.get(Servo.class, "claw_right");
        claw_left = this.hardMap.get(Servo.class, "claw_left");

        colorSensor = this.hardMap.get(NormalizedColorSensor.class, "sensor_color");
        sensorRange = hardMap.get(DistanceSensor.class, "sensor_color");
        MagneticLimitSwitch = hardMap.get(DigitalChannel.class, "Mag_Switch");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Set the direction of the motors ad the servos
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        linslide_left.setDirection(DcMotor.Direction.FORWARD);
        linslide_right.setDirection(DcMotor.Direction.REVERSE);

        claw_right.setDirection(Servo.Direction.FORWARD);
        claw_left.setDirection(Servo.Direction.REVERSE);

        // Reset the encoders for all the motors to run at a constant speed
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linslide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linslide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linslide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linslide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MagneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Initialize variables
        mecanum_power = 1;
        drive_speed = 1;
        slide_speedy = 0.7;
        drive_speed_transmission_limiter = 0.8;
        lift_speed_transmission_limiter = 1;
    }

    public void WheelNuetral()
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

    public void WheelBrake()
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

    public void LiftBrake()
    {
        linslide_left.setPower(0);
        linslide_right.setPower(0);

        linslide_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linslide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linslide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linslide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void claw_open(){
        claw_right.setPosition(0.4);
        claw_left.setPosition(0.4);
        claw_state = true;
    }

    public void claw_close(){
        claw_right.setPosition(0);
        claw_left.setPosition(0);
        claw_state = false;
    }

    public boolean isopen(){
        servotargetleft= claw_left.getPosition();
        servotargetright=claw_right.getPosition();

        if( (servotargetleft< 0.4) && (servotargetright>0.4))
        {
            return true;
        }else
        {
            return false;
        }
    }

}
