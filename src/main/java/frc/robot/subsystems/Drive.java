// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  // Motors
  WPI_TalonFX motorFR = new WPI_TalonFX(Constants.FRmotorID);
  WPI_TalonFX motorFL = new WPI_TalonFX(Constants.FLmotorID);
  WPI_TalonFX motorRR = new WPI_TalonFX(Constants.RRmotorID);
  WPI_TalonFX motorRL = new WPI_TalonFX(Constants.RLmotorID);

  MotorControllerGroup m_rightGroup = new MotorControllerGroup(motorFR, motorRR);
  MotorControllerGroup m_leftGroup = new MotorControllerGroup(motorFL, motorRL);
  
  DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  /** Creates a new Drive. */
  public Drive() {
    motorRR.setInverted(true);
    motorFR.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double speed, double turn) {
    m_drive.arcadeDrive(speed, turn);
  }

  public void setMotorToZero() {
    motorFR.set(0);
    motorFL.set(0);
    motorRR.set(0);
    motorRL.set(0);
  }
}
