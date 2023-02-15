package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** */
public class ArmSubsystem extends SubsystemBase {

  private WPI_TalonSRX arm = new WPI_TalonSRX(Constants.ARM_EXTEND_PORT);

  public ArmSubsystem() {}

  /** sets the ball intake motor speed -1 to +1 */
  public void setExtendArm() {
    SmartDashboard.putNumber("arm speed in sub system ", 0.15);
    // arm.set(ControlMode.PercentOutput, 0.15);
  }

  public void setRetracrtArm() {
    SmartDashboard.putNumber("arm speed in sub system ", -0.15);
    //  arm.set(ControlMode.PercentOutput, -0.15);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}
