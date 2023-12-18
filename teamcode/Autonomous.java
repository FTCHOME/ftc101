package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "name")
public class Autonomous extends LinearOpMode {

    protected DcMotor lF;
    protected DcMotor rF;
    protected DcMotor lB;
    protected DcMotor rB;

    @Override
    public void runOpMode() throws InterruptedException {
        lF = hardwareMap.get(DcMotor.class, "lF");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rB = hardwareMap.get(DcMotor.class, "rB");

        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        
        moveForward(1, 40);
        sleep(500);
        moveForward(-1, 40);
        sleep(500);
        
        Strafe(1, 100);
        sleep(500);
        Strafe(-1, 100);
        sleep(500);
        
        LeftTurn(1, 1000);
        sleep(500);
        LeftTurn(-1, 1000);
        sleep(500);
        
        moveForward(1, 40);
        sleep(500);
        Strafe(-1, 100);
        sleep(500);
        LeftTurn(1, 1000);
        sleep(500);
        
        moveForward(-1, 40);
        sleep(500);
        Strafe(1, 100);
        sleep(500);
        LeftTurn(-1, 1000);
    }
    
    public void moveForward (int power, int time){
        lF.setPower(power);
        rF.setPower(power);
        lB.setPower(power);
        rB.setPower(power);
        sleep(time);
        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }
    
    public void Strafe (int power, int time){
        lF.setPower(-power);
        rF.setPower(power);
        lB.setPower(power);
        rB.setPower(-power);
        sleep(time);
        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }
    
        public void LeftTurn (int power, int time){
        lF.setPower(-power);
        rF.setPower(power);
        lB.setPower(-power);
        rB.setPower(power);
        sleep(time);
        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }
}
    
