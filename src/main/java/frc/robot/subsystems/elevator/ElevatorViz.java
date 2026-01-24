package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorViz {
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevator;

    public ElevatorViz() {
        this.panel = new Mechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters));
        this.root = panel.getRoot("root", Inches.of(15).in(Meters), Inches.of(0).in(Meters));
        this.elevator = root.append(
            new MechanismLigament2d(
                "Elevator", 
                Inches.of(0).in(Meters),
                90, 
                10, 
                new Color8Bit(Color.kBlue)
            )
        );
                
        // Logger.recordOutput("ElevatorViz/Mechanism2d", this.panel);
    }
} 