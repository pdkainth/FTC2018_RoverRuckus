package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MineralDeliver {

  private DcMotor mineralMotor;
  double motorPosition;
  double mineralPower;

  private Servo mineralServo;
  private double servoPosition;
  private TouchSensor mineralTouch;
  //private static final double COLLECT_POSITION;
  //private static final double DUMP_POSITION;

  public void init(HardwareMap hardwareMap){
    mineralMotor = hardwareMap.get(DcMotor.class, "Motor5");
    mineralMotor.setPower(0.0);
    mineralMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    mineralServo = hardwareMap.get(Servo.class, "Servo2");
    mineralTouch = hardwareMap.get(TouchSensor.class, "Touch1");
    collect();
  }


  public void upDn(double motorPower){
    mineralPower = motorPower;
    if((motorPower < 0) && (mineralTouch.isPressed() == true)) {
      mineralPower = 0.0;
    }
    mineralMotor.setPower(mineralPower);

  }

  public void stop(){
    mineralMotor.setPower(0.0);
  }

  public void dump(){
    servoPosition = 1.0;
    mineralServo.setPosition(servoPosition);
  }

  public void collect(){
    servoPosition = 0.25;
    mineralServo.setPosition(servoPosition);
  }
  public void update(Telemetry telemetry){
    motorPosition = mineralMotor.getCurrentPosition();
    boolean touch = mineralTouch.isPressed();
    telemetry.addData("MineralMotor", "Power %.2f CurrentPosition %.2f MineralTouch %b",
            mineralPower, motorPosition, touch);
    telemetry.addData("MineralServo", "Position %.2f", servoPosition);

  }
}
