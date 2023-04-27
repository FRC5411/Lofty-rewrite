// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase {
  private Pigeon2 pigeon;

  public Pigeon() {
    pigeon = new Pigeon2(0);
  }

  public double getYaw() {
    return pigeon.getYaw();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public double getRoll() {
    return pigeon.getRoll();
  }

  public void zeroYaw() {
    setYaw(0);
  }

  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
