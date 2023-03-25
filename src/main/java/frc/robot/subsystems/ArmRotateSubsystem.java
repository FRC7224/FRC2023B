package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalStatus;

/** */
public class ArmRotateSubsystem extends SubsystemBase {

  private static WPI_TalonFX rotate1 = new WPI_TalonFX(Constants.ARM_ROTATE_PORT_L);
  private static WPI_TalonFX rotate2 = new WPI_TalonFX(Constants.ARM_ROTATE_PORT_R);

  /** Used to create string thoughout loop */
  StringBuilder _sb = new StringBuilder();

  int _loops = 0;

  /** Save the target position to servo to */
  double targetPositionRotations;

  public ArmRotateSubsystem() {

    /////////////////  Motor One ////////////////////////////////////////////////
    /* Factory Default all hardware to prevent unexpected behaviour */
    rotate1.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    rotate1.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        Constants.krotate1PIDLoopIdx,
        Constants.krotate1TimeoutMs);

    /* Ensure sensor is positive when output is positive */
    rotate1.setSensorPhase(Constants.krotate1SensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    rotate1.setInverted(Constants.krotate1MotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // rotate1.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    rotate1.configNominalOutputForward(0, Constants.krotate1TimeoutMs);
    rotate1.configNominalOutputReverse(0, Constants.krotate1TimeoutMs);
    rotate1.configPeakOutputForward(1, Constants.krotate1TimeoutMs);
    rotate1.configPeakOutputReverse(-1, Constants.krotate1TimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    rotate1.configAllowableClosedloopError(
        0, Constants.krotate1PIDLoopIdx, Constants.krotate1TimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    rotate1.config_kF(Constants.kPIDLoopIdx, Constants.kGainsR1.kF, Constants.krotate1TimeoutMs);
    rotate1.config_kP(Constants.kPIDLoopIdx, Constants.kGainsR1.kP, Constants.krotate1TimeoutMs);
    rotate1.config_kI(Constants.kPIDLoopIdx, Constants.kGainsR1.kI, Constants.krotate1TimeoutMs);
    rotate1.config_kD(Constants.kPIDLoopIdx, Constants.kGainsR1.kD, Constants.krotate1TimeoutMs);

    /////////////////  Motor Two ////////////////////////////////////////////////
    /* Factory Default all hardware to prevent unexpected behaviour */
    rotate2.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    rotate2.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        Constants.krotate2PIDLoopIdx,
        Constants.krotate2TimeoutMs);

    /* Ensure sensor is positive when output is positive */
    rotate2.setSensorPhase(Constants.krotate2SensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    rotate2.setInverted(Constants.krotate2MotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // rotate1.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    rotate2.configNominalOutputForward(0, Constants.krotate2TimeoutMs);
    rotate2.configNominalOutputReverse(0, Constants.krotate2TimeoutMs);
    rotate2.configPeakOutputForward(1, Constants.krotate2TimeoutMs);
    rotate2.configPeakOutputReverse(-1, Constants.krotate2TimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    rotate2.configAllowableClosedloopError(
        0, Constants.krotate1PIDLoopIdx, Constants.krotate1TimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    rotate2.config_kF(Constants.kPIDLoopIdx, Constants.kGainsR2.kF, Constants.krotate2TimeoutMs);
    rotate2.config_kP(Constants.kPIDLoopIdx, Constants.kGainsR2.kP, Constants.krotate2TimeoutMs);
    rotate2.config_kI(Constants.kPIDLoopIdx, Constants.kGainsR2.kI, Constants.krotate2TimeoutMs);
    rotate2.config_kD(Constants.kPIDLoopIdx, Constants.kGainsR2.kD, Constants.krotate2TimeoutMs);
  }

  /// Motor 1 Methods ////
  public static double GetMotorOutputPercentR1() {
    return (rotate1.getMotorOutputPercent());
  }
  ;

  public static double GetSelectedSensorPositionR1() {
    return (rotate1.getSelectedSensorPosition(0));
  }
  ;

  public double GetClosedLoopErrorR1() {
    return (rotate1.getClosedLoopError(0));
  }
  ;

  public ControlMode GetControlModeR1() {
    return (rotate1.getControlMode());
  }
  ;

  public void SetTargetPositionRotationsR1(double targetPositionRotations) {
    rotate1.set(ControlMode.Position, targetPositionRotations);
    GlobalStatus.Global_Rotate1_position = targetPositionRotations;
  }
  ;

  public void SetPercentOutputR1(double percentoutput) {
    rotate1.set(ControlMode.PercentOutput, percentoutput);
  }
  ;

  /// Motor 2 Methods ////

  public static double GetMotorOutputPercentR2() {
    return (rotate2.getMotorOutputPercent());
  }
  ;

  public static double GetSelectedSensorPositionR2() {
    return (rotate2.getSelectedSensorPosition(0));
  }
  ;

  public double GetClosedLoopErrorR2() {
    return (rotate2.getClosedLoopError(0));
  }
  ;

  public ControlMode GetControlModeR2() {
    return (rotate2.getControlMode());
  }
  ;

  public void SetTargetPositionRotationsR2(double targetPositionRotations) {
    rotate2.set(ControlMode.Position, targetPositionRotations);
    GlobalStatus.Global_Rotate2_position = targetPositionRotations;
  }
  ;

  public void SetPercentOutputR2(double percentoutput) {
    rotate2.set(ControlMode.PercentOutput, percentoutput);
  }
  ;

  public void SetTargetRotStart() {
    rotate1.set(ControlMode.Position, Constants.START_ROT_PRESET);
    rotate2.set(ControlMode.Position, -Constants.START_ROT_PRESET);
    GlobalStatus.Global_Rotate1_position = Constants.START_ROT_PRESET;
    GlobalStatus.Global_Rotate2_position = -Constants.START_ROT_PRESET;
  }
  ;

  public void SetTargetRotHIGH() {
    rotate1.set(ControlMode.Position, Constants.HIGH_ROT_PRESET + Constants.OFFSET_ROT);
    rotate2.set(ControlMode.Position, -Constants.HIGH_ROT_PRESET);
    GlobalStatus.Global_Rotate1_position = Constants.HIGH_ROT_PRESET + Constants.OFFSET_ROT;
    GlobalStatus.Global_Rotate2_position = -Constants.HIGH_ROT_PRESET;
  }
  ;

  public void SetTargetRotZero() {
    rotate1.set(ControlMode.Position, 0);
    rotate2.set(ControlMode.Position, 0);
    GlobalStatus.Global_Rotate1_position = 0;
    GlobalStatus.Global_Rotate2_position = 0;
  }

  /** Stops the motion of the robot. */
  public void stop() {
    rotate1.set(ControlMode.PercentOutput, 0);
    rotate2.set(ControlMode.PercentOutput, 0);
  }
  ;

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
