// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CANdle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.LimelightHelpers;

public class CANdleSubsystem extends SubsystemBase {
  /** Creates a new CANdleSubsystem. */
    CANdle CANdleONE = new CANdle(51, "CANivore");
    CANdleConfiguration m_CANdleConfigs = new CANdleConfiguration();
    RGBWColor kGreen = new RGBWColor(0, 217, 0, 0).scaleBrightness(0.6);
    RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.6);
    RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow().scaleBrightness(0.6);
    RGBWColor kyellow = new RGBWColor(255,171,0).scaleBrightness(0.6);
    RGBWColor kpurple = new RGBWColor(145,0,255).scaleBrightness(0.6);
    int LedCount = 8;
    StrobeAnimation m_toAnimate;
    FireAnimation m_FireAnimate;
    CANdleFeaturesConfigs cfg = new CANdleFeaturesConfigs();
    public void yellowCANdle(){
    //StrobeAnimation yellowStrobe = new StrobeAnimation(4, 4);
    CANdleONE.setControl(new SolidColor(0,20).withColor(kyellow));
  }

   public void purpleCANdle() 
  {  
   CANdleONE.setControl(new SolidColor(0,20).withColor(kpurple));
  }
  public void whiteCANdle() 
  {  
   CANdleONE.setControl(new SolidColor(0,20).withColor(kWhite));
  }
  public void greenCANdle() 
  {  
   CANdleONE.setControl(new SolidColor(0,20).withColor(kGreen));
  }
   public void redCANdle() 
  {  
   CANdleONE.setControl(new SingleFadeAnimation(0,20).withSlot(0).withColor(kRed));
  }
  public void FireCANdle() 
  {
    m_FireAnimate = new FireAnimation(0, 20);
  }
 
  public void CANdle_off() 
  {
    m_toAnimate = new StrobeAnimation(0, 0);
  }
 
  public CANdleSubsystem() {
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//      if (!DriverStation.isTeleop()) {
//             // Actively set LED to a "not-teleop" state
//              FireCANdle();
//         } else {
//             // Optional: Turn off or change behavior during Teleop
//              whiteCANdle();
//         }
//          boolean hasTarget = LimelightHelpers.getTV("limelight-turret");

//     if (hasTarget) {
//         // Target Locked: Green
//         greenCANdle(); } 
// else {
//         // No Target: Red
//         redCANdle();
//     }
  }
}
