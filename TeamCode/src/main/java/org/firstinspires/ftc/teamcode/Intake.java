package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private static final double FULL_POWER = 1.0;
    private static final double NO_POWER = 0.0;
    private CRServo intakeServo;
    private double power;

    public void init(HardwareMap hardwareMap){
      intakeServo = hardwareMap.get(CRServo.class, "CRservo0");
      intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
      stop();
    }

    public void turnOn(){
      power = FULL_POWER;
      intakeServo.setPower(power);
    }

    public void turnOff(){
      power = NO_POWER;
      intakeServo.setPower(power);
    }

    public void update(Telemetry telemetry){
      telemetry.addData("IntakeSpinner","power %.2f", power);
    }

    public void stop(){
      power = NO_POWER;
      intakeServo.setPower(NO_POWER);
    }
}
