package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class Teleop extends OpMode {

    private DcMotor rB, lB, rF, lF;

    @Override
    public void init() {
        // Initialize motors
        rB = hardwareMap.get(DcMotor.class, "rB");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lF = hardwareMap.get(DcMotor.class, "lF");

        // Set motor directions
        rB.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
        rF.setDirection(DcMotor.Direction.REVERSE);
        lF.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Gamepad controls
        double drive = gamepad1.left_stick_y; // Forward and backward
        double strafe = -gamepad1.left_stick_x;  // Left and right
        double turn = gamepad1.right_stick_x;   // Turning

        // Mecanum drive calculations
        double rBPower = Range.clip(drive + strafe + turn, -0.5, 0.5);
        double lBPower = Range.clip(drive - strafe - turn, -0.5, 0.5);
        double rFPower = Range.clip(drive - strafe + turn, -0.5, 0.5);
        double lFPower = Range.clip(drive + strafe - turn, -0.5, 0.5);

        // Set motor powers
        rB.setPower(rBPower);
        lB.setPower(lBPower);
        rF.setPower(rFPower);
        lF.setPower(lFPower);

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("rB Power", rBPower);
        telemetry.addData("lB Power", lBPower);
        telemetry.addData("rF Power", rFPower);
        telemetry.addData("lF Power", lFPower);
        telemetry.update();
    }
}
