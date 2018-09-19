import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Testing teleOP", group="Linear Opmode")


public class myTeleOp extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftDrive = null;

  @Override
  public void runOpMode(){

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    leftDrive  = hardwareMap.get(DcMotor.class, "motorTest");
    leftDrive.setDirection(DcMotor.Direction.FORWARD);

    waitForStart();
    runtime.reset();

    while (opModeIsActive()){

      double leftPower;

      double drive = -gamepad1.left_stick_y;
      double turn  =  gamepad1.right_stick_x;

      leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;

      leftDrive.setPower(leftPower);

      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Motors", "motorTest (%.2f)", leftPower);
      telemetry.update();

    }

  }
}
