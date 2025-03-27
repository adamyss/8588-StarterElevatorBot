package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SubsystemConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

  private SparkMax m_elevatorMotorRight;
  private SparkMax m_elevatorMotorLeft;
  private ProfiledPIDController m_elevatorRightController;
  private ProfiledPIDController m_elevatorLeftController;
  private RelativeEncoder m_encoder;

  public Elevator() {
    m_elevatorMotorRight = new SparkMax(Constants.ElevatorConstants.kElevatorRightId, MotorType.kBrushless);
    m_elevatorMotorLeft = new SparkMax(Constants.ElevatorConstants.kElevatorLeftId, MotorType.kBrushless);
    m_encoder = m_elevatorMotorLeft.getEncoder();
    m_elevatorRightController = new ProfiledPIDController(Constants.ElevatorConstants.kElevatorKp,
        Constants.ElevatorConstants.kElevatorKi, Constants.ElevatorConstants.kElevatorKd,
        new TrapezoidProfile.Constraints(Constants.ElevatorConstants.kElevatorMaxVelocity,
            Constants.ElevatorConstants.kElevatorMaxAcceleration));
    m_elevatorLeftController = new ProfiledPIDController(Constants.ElevatorConstants.kElevatorKp,
        Constants.ElevatorConstants.kElevatorKi, Constants.ElevatorConstants.kElevatorKd,
        new TrapezoidProfile.Constraints(Constants.ElevatorConstants.kElevatorMaxVelocity,
            Constants.ElevatorConstants.kElevatorMaxAcceleration));

  }

  public void reachGoal(double goal) {
    double feedForward;
    double voltsOut = MathUtil.clamp(
        m_elevatorLeftController.calculate(getHeightMeters(), goal) + 7/*
                                                                        * Feed forward meant to replace 7 don't know how
                                                                        * to do that
                                                                        */, -12, 12); // 7 is the max voltage to send
                                                                                      // out.
    m_elevatorMotorLeft.setVoltage(voltsOut);
    m_elevatorMotorRight.setVoltage(voltsOut);

  }

  public double getHeightMeters() {
    // m = (e / g) * (2*pi*r)
    // m/(2*pi*r) = e / g
    // m/(2*pi*r)*g = e
    return (m_encoder.getPosition() / ElevatorConstants.kElevatorGearing) *
        (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
  }

  public double getVelocityMetersPerSecond() {
    return ((m_encoder.getVelocity() / 60) / ElevatorConstants.kElevatorGearing) *
        (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
  }

  public Command setGoal(double goal) {
    return runOnce(() -> {
      m_elevatorLeftController.reset(getHeightMeters());
      m_elevatorRightController.reset(getHeightMeters());
    }).andThen(run(() -> reachGoal(goal)));
  }

  public Command setElevatorHeight(double height) {
    return setGoal(height).until(() -> aroundHeight(height, ElevatorConstants.kElevatorAllowableError));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop() {
    m_elevatorMotorLeft.set(0.0);
    m_elevatorMotorRight.set(0.0);

  }

  public boolean aroundHeight(double height, double allowableError) {
    // System.out.println("Current Height: " + getHeightMeters() + " Desired Height:
    // " + height + " Allowable Error: " +
    // (height-getHeightMeters()));
    return MathUtil.isNear(height, getHeightMeters(), allowableError);
  }

  /**
   * Gets the height of the elevator and compares it to the given height with the
   * given tolerance.
   *
   * @param height Height in meters
   * @return Within that tolerance.
   */

  public Command setPower(double d) {
    return run(() -> {
      m_elevatorMotorLeft.set(d);
      m_elevatorMotorRight.set(d);
    });
  }
}