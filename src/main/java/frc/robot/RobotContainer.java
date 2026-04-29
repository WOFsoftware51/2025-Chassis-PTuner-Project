// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.Targets;
import frc.robot.Autons.AutoPath;
import frc.robot.Autons.Center_2;
import frc.robot.Autons.Center_3;
import frc.robot.Autons.DEPOT;
import frc.robot.Autons.Left_2_Sweeps;
import frc.robot.Autons.Left_Center;
import frc.robot.Autons.Left_Middle2Cycle;
import frc.robot.Autons.Left_StopAtMiddle;
import frc.robot.Autons.MEIOSIS;
import frc.robot.Autons.Middle_Depot;
import frc.robot.Autons.OSMOSIS;
import frc.robot.Autons.Right_2_Sweeps;
import frc.robot.Autons.Right_Center;
import frc.robot.Autons.Right_Middle2Cycle;
import frc.robot.Autons.Right_StopAtMiddle;
import frc.robot.Autons.S_C_S;
import frc.robot.Autons.S_C_S_LEFT;
import frc.robot.Autons.Shoot8;
import frc.robot.Autons.TURNAROUNDLEFT;
import frc.robot.Autons.test;
import frc.robot.Autons.doNOTHING;
import frc.robot.commands.MoveToAngle;
import frc.robot.commands.TurretCameraPoseDefaultCommand;
import frc.robot.commands.factories.Superstructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CANdle.CANdleCommand;
import frc.robot.subsystems.CANdle.CANdleSubsystem;
import frc.robot.subsystems.feeder.FeederIOHardware;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodIOHardware;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotIOHardware;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerIOHardware;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretIOHardware;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionChassisSubsystem;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionTurretSubsystem;
import frc.robot.util.LoggedTunableNumber;
// Optional: If you need wait until a specific time in the match

public class RobotContainer {
    public double Speedmodifier = 0.6;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.5 * RotationsPerSecond.of(1.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity: 0.8435211984 RPS

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric poseTuning = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    // private final CommandPS5Controller driver = new CommandPS5Controller(0);
    // private final CommandPS5Controller operator = new CommandPS5Controller(1);
    private final CommandXboxController joystick = new CommandXboxController(3);
    private final CommandXboxController test = new CommandXboxController(5);

    private final SendableChooser<Integer> a_chooser = new SendableChooser<>();


    private final VisionTurretSubsystem limelightTurret;
    private final VisionChassisSubsystem limelightChassis;

    public final Swerve swerve;
    public final RobotState robotState = RobotState.getInstance();
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;
    private final IntakePivotSubsystem intakePivot;
    private final Superstructure superstructure;
    private final CANdleSubsystem cANdle;
	
    private final Paths10 path;
    private boolean operatorRumble = false;

    LoggedTunableNumber speedLeft = new LoggedTunableNumber("Pose/speedLeft", 5);
    LoggedTunableNumber speedRight = new LoggedTunableNumber("Pose/speedRight", 5);
    LoggedTunableNumber speedForward = new LoggedTunableNumber("Pose/speedLeft", 1);
    LoggedTunableNumber speedBackward = new LoggedTunableNumber("Pose/speedRight", 1);
    
    LoggedTunableNumber pivotAngle = new LoggedTunableNumber("Shooting/Pivot", 0.0);
    LoggedTunableNumber turretAngle = new LoggedTunableNumber("Shooting/Turret", 0.0);

    //  HashMap<String, Command> eventMap = new HashMap<>();

    public RobotContainer() {
        this.intakePivot = new IntakePivotSubsystem(
            Robot.isReal() ? new IntakePivotIOHardware() : new IntakePivotIOSim()
        );
        this.limelightTurret = new VisionTurretSubsystem(
            //  new VisionIOHardware(Constants.VisionConstants.kTurretLimelight)
            Robot.isReal() ? new VisionIOHardware(Constants.VisionConstants.kTurretLimelight) : new VisionIOSim()
        );

        this.spindexer = new SpindexerSubsystem(
            Robot.isReal() ? new SpindexerIOHardware() : new SpindexerIOSim()
        );

        this.feeder = new FeederSubsystem(
            Robot.isReal() ? new FeederIOHardware() : new FeederIOSim()
        );

        this.hood = new HoodSubsystem(
            Robot.isReal() ? new HoodIOHardware() : new HoodIOSim(), 
            robotState
        );
        
        this.limelightChassis = new VisionChassisSubsystem(
            Robot.isReal() ? new VisionIOHardware(Constants.VisionConstants.kChassisLimelight) : new VisionIOSim()
        );

        this.turret = new TurretSubsystem(
            Robot.isReal() ? new TurretIOHardware() : new TurretIOSim(),
            limelightTurret, 
            robotState
        );

        this.swerve = TunerConstants.createDrivetrain(limelightTurret, limelightChassis);

        this.intake = new IntakeSubsystem(
            Robot.isReal() ? new IntakeIOHardware() : new IntakeIOSim(swerve.mapleSimSwerveDrivetrain.mapleSimDrive)
        );

        this.shooter = new ShooterSubsystem(
            Robot.isReal() ? new ShooterIOHardware() : new ShooterIOSim(this.swerve.mapleSimSwerveDrivetrain.mapleSimDrive)
        );
        this.cANdle = new CANdleSubsystem(
        );

        
        this.superstructure = new Superstructure(swerve, intake, intakePivot, spindexer, feeder, turret, shooter, hood);

        this.path = new Paths10();

        configureBindings();
        printAutons();
       
    }

    private void configureBindings() {
        /*
        Swerve Controls
        */
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            swerve.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerve.applyRequest(() ->
                    drive.withVelocityX(-driver.getLeftY() * MaxSpeed * Speedmodifier ) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * Speedmodifier ) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate ) // Drive counterclockwise with negative X (left)
                )
            );
            new Trigger(driver.rightTrigger()).onTrue(Commands.runOnce(() -> {Speedmodifier = 1.0;}));
            new Trigger(driver.rightTrigger()).onFalse(Commands.runOnce(() -> {Speedmodifier = 0.5;}));
            //new Trigger(operator.rightTrigger()).onTrue(Commands.runOnce(() -> {Speedmodifier = 0.1;}));
            //new Trigger(operator.rightTrigger()).onFalse(Commands.runOnce(() -> {Speedmodifier = 0.5;}));

            // driver.y().whileTrue(
            //     swerve.applyRequest(() ->
            //         poseTuning.withVelocityX(speedForward.get())
            //             .withVelocityY(0) 
            //             .withRotationalRate(0)
            //     )
            // );  
            // driver.a().whileTrue(
            //     swerve.applyRequest(() ->
            //         poseTuning.withVelocityX(-speedBackward.get()) // Drive forward with negative Y (forward)
            //             .withVelocityY(0) // Drive left with negative X (left)
            //             .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            //     )
            // );
            // driver.leftTrigger().whileTrue(swerve.applyRequest(() -> brake));
            // driver.y().whileTrue(swerve.applyRequest(() ->
            //     point.withModuleDirection(new Rotation2d(0))
            // ));
            // driver.rightBumper().whileTrue(
            //     new GoToPositionCommand(swerve, robotState, new Pose2d(new Translation2d(2, 4), 
            //     new Rotation2d(Units.degreesToRadians(-90))), 1));

            // driver.leftBumper().whileTrue(
            //     new GoToPositionCommand(
            //         swerve, 
            //         robotState, 
            //         () -> new Pose2d(robotState.getPose2d().getTranslation(), new Rotation2d(robotState.getRobotToRedHubDegrees())),
            //         1
            //     )
            // );

            new Trigger(() -> swerve.testConfigsChanged).onTrue(swerve.setDriveGains());
            
            driver.start().onTrue(swerve.runOnce(() -> swerve.resetPose(new Pose2d())));
            // driver.touchpad().onTrue(swerve.runOnce(() -> swerve.resetPose(new Pose2d())));

        /*CANdle */
        
            cANdle.setDefaultCommand(new CANdleCommand(cANdle));

        /*
        Turret Controls
        */
        turret.setDefaultCommand(new TurretCameraPoseDefaultCommand(turret));
            operator.rightTrigger().onTrue(Commands.runOnce(() -> robotState.currentTarget = Targets.Hub));
            operator.leftTrigger().onTrue(Commands.runOnce(() -> robotState.currentTarget = Targets.Feed));
            driver.rightBumper().whileTrue(turret.TurretRunWithVolts(Volts.of(0)));
            // operator.x().whileTrue(turret.TurretRunWithVolts(Volts.of(3))); //To the left
            // operator.a().whileTrue(turret.TurretToSetpointCommand(Degrees.of(0))); 
            // operator.povUp().whileTrue(turret.resetEncoder()); 

        /*
        Shooter Controls
        */
            new Trigger(() -> shooter.gainsChanged).whileTrue(shooter.updateGainsCommand());
            // driver.rightTrigger().whileTrue(shooter.runRPMCommand());
            operator.rightTrigger().whileTrue(shooter.treeMapRPMCommand());
            operator.leftTrigger().whileTrue(shooter.treeMapRPMCommand());
            
            // operator.R2().whileTrue(shooter.treeMapRPMCommand());
            
            new Trigger(() -> shooter.atRPM).onTrue(
                Commands.runOnce(() ->
                    operator.setRumble(RumbleType.kBothRumble, 0.5)
                )
            )
            .onFalse(
                Commands.runOnce(() ->
                    operator.setRumble(RumbleType.kBothRumble, 0.0)
                )
            );

        /*
        Pivot Controls
        */
            hood.setDefaultCommand(Commands.run(() -> hood.runToPosition(), hood));
            // hood.setDefaultCommand(hood.treeMapRPMCommand());
            operator.rightTrigger().whileTrue(hood.treeMapRPMCommand());
			operator.leftTrigger().whileTrue(hood.treeMapRPMCommand());
            // operator.y().whileTrue(hood.runToPositionCommand(15));

            test.y().whileTrue(hood.runVolts(2));
            test.a().whileTrue(hood.runVolts(-2));
            // operator.rightBumper().whileTrue(hood.runToPositionCommand(10));
            // operator.povUp().whileTrue(hood.resetEncoder());
        
        /*
        Feeder Controls
        */
            // operator.leftTrigger().whileTrue(feeder.runFeederVoltsCommand(12));
            // operator.L2().whileTrue(feeder.runFeederVoltsCommand(12));

            new Trigger(() -> shooter.atRPM).whileTrue(feeder.runFeederVoltsCommand(12));

        /*
        Spindexer Controls
        */
            // operator.leftTrigger().whileTrue(spindexer.runSpindexerVoltsCommand(12));
            // operator.L2().whileTrue(spindexer.runSpindexerVoltsCommand(12));

            new Trigger(() -> shooter.atRPM).whileTrue(spindexer.runSpindexerVoltsCommand(12));


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*
        Intake
        */
            // driver.leftBumper().whileTrue(intake.runVolts(10.8));
            driver.leftTrigger().whileTrue(intake.runVolts(10.56));
            // driver.L1().whileTrue(intake.runVolts(12));
        
        /*
        Intake Pivot
        */
            intakePivot.setDefaultCommand(intakePivot.runVoltsJoystick(() -> operator.getRightY()*0.5));
            // joystick.a().whileTrue(intakePivot.runSetpointCommand(0));
            // joystick.a().whileTrue(intakePivot.runSetpointCommand(-100));
            operator.leftBumper().whileTrue(intakePivot.runVolts(-6).alongWith(intake.runVolts(12)));
            operator.rightBumper().whileTrue(intakePivot.runVolts(6).alongWith(intake.runVolts(12)));

            
            operator.a().whileTrue(intakePivot.goDown());
            operator.x().whileTrue(intakePivot.bounce()).and(() -> intakePivot.up).whileTrue(intake.runVolts(6));
            // operator.L1().whileTrue(intakePivot.runVolts(-3));
            // operator.L2().whileTrue(intakePivot.runVolts(3));
            // operator.povUp().whileTrue(intakePivot.resetEncoder());

        /*NamedCommands*/ 
            NamedCommands.registerCommand("Intake",intake.runVolts(10.56));
            NamedCommands.registerCommand("IntakePivot",intakePivot.runVolts(6).withTimeout(0.5));
            NamedCommands.registerCommand("IntakePivotUp",intakePivot.runVolts(-6).withTimeout(0.2));
            NamedCommands.registerCommand("Score",shooter.treeMapRPMCommand().alongWith(hood.treeMapRPMCommand()));
            NamedCommands.registerCommand("ScoreEvent",shooter.treeMapRPMCommand().alongWith(hood.treeMapRPMCommand()));
            NamedCommands.registerCommand("IntakeEvent",intake.runVolts(10.56));
            NamedCommands.registerCommand("IntakePivotEvent",intakePivot.runVolts(6));
            NamedCommands.registerCommand("IntakePivotUpEvent",intakePivot.runVolts(-6));
            
        /*Instances of EventMakers*/
            // new EventMarker("IntakeEvent",1.0,1.28,NamedCommands.getCommand("Intake"));
            // new EventMarker("IntakePivotEvent", 0,NamedCommands.getCommand("IntakePivot"));
            // new EventMarker("ScoreEvent", 0.7, 1.95, NamedCommands.getCommand("Score"));
            // new EventMarker("IntakePivotUpEvent", 0, NamedCommands.getCommand("IntakePivotUp"));

        swerve.registerTelemetry(logger::telemeterize);


    NamedCommands.registerCommand("Shoot",shooter.treeMapRPMCommand().withTimeout(2.5));
    NamedCommands.registerCommand("Pivot",hood.treeMapRPMCommand().withTimeout(2.5));
    NamedCommands.registerCommand("IntakePivot",intakePivot.runVolts(6).withTimeout(0.7));
    NamedCommands.registerCommand("IntakePivotUp", intakePivot.runVolts(-6).withTimeout(0.53));
    NamedCommands.registerCommand("Intake", intake.runVolts(10.56));
    NamedCommands.registerCommand("ShootMore",shooter.treeMapRPMCommand().withTimeout(5));
    NamedCommands.registerCommand("PivotMore",hood.treeMapRPMCommand().withTimeout(5));
    
    new EventMarker("IntakePivotEvent", 0,intakePivot.runVolts(6).withTimeout(0.7));
    new EventMarker("IntakePivotUpEvent", 0,intakePivot.runVolts(-6).withTimeout(0.53));
    new EventMarker("IntakeEvent", 2,1.28,intake.runVolts(10.56));
    new EventMarker("ShootEvent", 0.70,1.95,shooter.treeMapRPMCommand());
    new EventMarker("PivotEvent", 0.70,1.95,hood.treeMapRPMCommand());

    // NamedCommands.registerCommand("Spin", spindexer.runSpindexerVoltsCommand(12.0));
    // NamedCommands.registerCommand("Feed", feeder.runFeederVoltsCommand(12));

 


    }

    public void printAutons(){
        SmartDashboard.putData("Auton", a_chooser);
        // a_chooser.setDefaultOption("test", 1);
        // a_chooser.addOption("test", 1);
        // a_chooser.addOption("Left_Center (dont run yet unless you wanna yolo)", 2);
        a_chooser.addOption("Do Nothing", 1);
        // a_chooser.addOption("Left_StopAtMiddle", 4);
        // a_chooser.addOption("Right_StopAtMiddle", 5);
        // a_chooser.addOption("Right_Center (dont run yet)", 6);
        // a_chooser.addOption("S_C_S_LEFT", 2);
        // a_chooser.addOption("S_C_S", 3);
        a_chooser.addOption("Shoot8", 2);
        // a_chooser.addOption("OSMOSIS", 5);
        // a_chooser.addOption("MEIOSIS", 6);
        a_chooser.addOption("Left_Middle2Cycle", 3);
        a_chooser.addOption("Right_Middle2Cycle", 4);
        a_chooser.addOption("Right_StopAtMiddle", 5);
        a_chooser.addOption("Left_StopAtMiddle", 6);
        // a_chooser.addOption("TURNAROUNDLEFT", 11);
        // a_chooser.addOption("DEPOT", 12);
        a_chooser.addOption("Middle_depot", 7);
        a_chooser.addOption("Center_2", 8);
        a_chooser.addOption("Center_3", 9);
        a_chooser.addOption("Left_2_Sweeps", 10);
        a_chooser.addOption("Right_2_Sweeps", 11);



    }
    


    public Command getAutonomousCommand() {
        switch (a_chooser.getSelected()) {
            // case 1:
            //     return new test(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);

            // case 2:
            //     return new Left_Center(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);

            case 1:
                return new doNOTHING(swerve);
                
            // case 6:
            //     return new Right_Center(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);
            // case 2:
            //     return new S_C_S_LEFT(spindexer, feeder, shooter, intake, hood, swerve, intakePivot);
            // case 3:
            //     return new PathPlannerAuto("S_C_S");
            case 2:
                return new Shoot8(swerve, null, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure, path);
            // case 5:
            //     return new PathPlannerAuto("OSMOSIS");
            // case 6:
            //     return new PathPlannerAuto("MEIOSIS");
             case 3:
                return new Left_Middle2Cycle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            case 4:
                return new Right_Middle2Cycle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure, path);
            case 5:
                return new Right_StopAtMiddle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure, path);
            case 6:
                return new Left_StopAtMiddle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure);
            // case 11:
            //     return new TURNAROUNDLEFT(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            // case 7:
            //     return new DEPOT(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            case 7:
                return new Middle_Depot(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            case 8:
                return new Center_2(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            case 9:
                return new Center_3(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            case 10:
                return new Left_2_Sweeps(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);
            case 11:
                return new Right_2_Sweeps(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure,path);

            default:
                return new doNOTHING(swerve);

        }
    }  
}
