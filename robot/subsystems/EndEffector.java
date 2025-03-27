// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  private SparkMax m_effectorDown;

  private double EffectorSpeed = SubsystemConstants.kEffectorSlow;

  public EndEffector() {
    m_effectorDown =  new SparkMax(Constants.SubsystemConstants.kEffectorID, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    m_effectorDown.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  public void runEffector(double speed) {
    m_effectorDown.set(EffectorSpeed*speed);
  }
  public Command startEffectorCommand(){
    return this.runOnce(() -> runEffector(-1));
  }
  public Command stopEffectorCommand(){
    return this.runOnce(() -> runEffector(0));
  }

  public Command lockEffectorCommand(){
    return this.runOnce(() -> runEffector(-1*SubsystemConstants.kEffectorLock));
  }

  public void fastMode(boolean isFast) {
    if (isFast) {
      EffectorSpeed = SubsystemConstants.kEffectorFast;
    } else {
      EffectorSpeed = SubsystemConstants.kEffectorSlow;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}