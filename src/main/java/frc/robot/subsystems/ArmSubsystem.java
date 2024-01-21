package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** */
public class ArmSubsystem extends SubsystemBase {

  // private static WPI_TalonSRX arm = new WPI_TalonSRX(Constants.ARM_EXTEND_PORT);
  private static WPI_TalonFX arm = new WPI_TalonFX(Constants.ARM_EXTEND_PORT);

  /** Used to create string thoughout loop */
  StringBuilder _sb = new StringBuilder();

  int _loops = 0;

  /** Save the target position to servo to */
  double targetPositionRotations;

  public ArmSubsystem() {
    // OLD FOR MOTOR
    /* Factory Default all hardware to prevent unexpected behaviour */
    //    arm.configFactoryDefault();
    /* Config the sensor used for Primary PID and sensor direction */
    //    arm.configSelectedFeedbackSensor(
    //        FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    //    arm.setSensorPhase(Constants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    // arm.setInverted(Constants.kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    //    arm.configNominalOutputForward(0, Constants.kTimeoutMs);
    //    arm.configNominalOutputReverse(0, Constants.kTimeoutMs);
    //    arm.configPeakOutputForward(1, Constants.kTimeoutMs);
    //    arm.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    //    arm.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    //    arm.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    //  arm.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    //    arm.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    //    arm.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and intitally set the
     * relative sensor to match.
     */
    //    int absolutePosition = arm.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    //    absolutePosition &= 0xFFF;
    //    if (Constants.kSensorPhase) {
    //    absolutePosition *= -1;
    //   }
    //    if (Constants.kMotorInvert) {
    //      absolutePosition *= -1;
    //    }

    /* Set the quadrature (relative) sensor to match absolute */
    //    arm.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx,
    // Constants.kTimeoutMs);
    //    END  OLD Section

    /** Save the target position to servo to */

    ///////////////// Arm  Motor  ////////////////////////////////////////////////
    /* Factory Default all hardware to prevent unexpected behaviour */
    arm.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    arm.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    arm.setSensorPhase(Constants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    arm.setInverted(Constants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // rotate1.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    arm.configNominalOutputForward(0, Constants.kTimeoutMs);
    arm.configNominalOutputReverse(0, Constants.kTimeoutMs);
    arm.configPeakOutputForward(1, Constants.kTimeoutMs);
    arm.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    arm.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    arm.config_kF(Constants.kPIDLoopIdx, Constants.kGainsR1.kF, Constants.kTimeoutMs);
    arm.config_kP(Constants.kPIDLoopIdx, Constants.kGainsR1.kP, Constants.kTimeoutMs);
    arm.config_kI(Constants.kPIDLoopIdx, Constants.kGainsR1.kI, Constants.kTimeoutMs);
    arm.config_kD(Constants.kPIDLoopIdx, Constants.kGainsR1.kD, Constants.kTimeoutMs);
  }

  public static double GetMotorOutputPercent() {
    return (arm.getMotorOutputPercent());
  }
  ;

  public static double GetSelectedSensorPosition() {

    return (arm.getSelectedSensorPosition(0));
  }
  ;

  public double GetClosedLoopError() {
    return (arm.getClosedLoopError(0));
  }
  ;

  public ControlMode GetControlMode() {
    return (arm.getControlMode());
  }
  ;

  public void SetTargetPositionRotations(double targetPositionRotations) {
    arm.set(ControlMode.Position, targetPositionRotations);
  }
  ;

  public void SetTargetArmZero() {
    arm.set(ControlMode.Position, 0);
  }
  ;

  public void SetTargetArmHigh() {
    arm.set(ControlMode.Position, Constants.HIGH_ARM_PRESET_AUTO);
  }
  ;

  public void SetPercentOutput(double percentoutput) {
    arm.set(ControlMode.PercentOutput, percentoutput);
  }
  ;

  /** Stops the motion of the robot. */
  public void stop() {
    SetPercentOutput(0.0);
  }

  /** sets the ball intake motor speed -1 to +1 */
  public void setExtendArm() {
    arm.set(ControlMode.PercentOutput, 0.15);
  }

  public void setRetracrtArm() {
    arm.set(ControlMode.PercentOutput, -0.15);
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
