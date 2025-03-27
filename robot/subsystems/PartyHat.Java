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

public class PartyHat extends SubsystemBase {
  private SparkMax m_PartyHatDown;

  private double PartyHatSpeed = SubsystemConstants.kPartyHatSlow;

  public PartyHat() {
    m_PartyHatDown =  new SparkMax(Constants.SubsystemConstants.kPartyHatId, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    m_PartyHatDown.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  public void runPartyHat(double speed) {
    m_PartyHatDown.set(PartyHatSpeed*speed);
  }
  
  public Command startPartyHatCommand(){
    return this.runOnce(() -> runPartyHat(-1));
  }
  public Command stopPartyHatCommand(){
    return this.runOnce(() -> runPartyHat(0));
  }
  public void fastMode(boolean isFast) {
      PartyHatSpeed = SubsystemConstants.kPartyHatFast * -1;
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