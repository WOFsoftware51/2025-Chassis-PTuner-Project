package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class Paths10 {
    public PathPlannerPath Center_3;
    public PathPlannerPath Center_2;
    public PathPlannerPath Left_2_Sweeps;
    public PathPlannerPath Right_2_Sweeps;
    public PathPlannerPath RightPickUp_RightTrenchMoreCenter;
    public PathPlannerPath RightTrenchMoreCenter_Center2;
    public PathPlannerPath Middle_Depot;
    public PathPlannerPath emptycenter;
    public PathPlannerPath emptyBump;
    public PathPlannerPath _Depot;
    public PathPlannerPath LeftTrench_Center10;
    public PathPlannerPath COMEBACK10;
    public PathPlannerPath LeftTrench_Center102;
    public PathPlannerPath LeftCenter_Pickup26;
    public PathPlannerPath LeftPickUp_LeftTrench26;
    public PathPlannerPath LeftTrench_Center6;
    public PathPlannerPath emptyLeftTrench6;
    public PathPlannerPath LeftPickUp_LeftTrench6;
    public PathPlannerPath LeftTrench_Center26;
    public PathPlannerPath RightTrench_Center;
    public PathPlannerPath RightPickUp_RightTrench;
    public PathPlannerPath RightTrench_Center2;
    public PathPlannerPath RightCenter_Pickup2;
    public PathPlannerPath RightPickUp_RightTrench2;
    public PathPlannerPath emptyRightTrench;

    public Paths10() {
        try {
            RightTrench_Center = PathPlannerPath.fromPathFile("RightTrench_Center");
            // RightCenter_Pickup = PathPlannerPath.fromPathFile("RightCenter_Pickup");
            RightPickUp_RightTrench = PathPlannerPath.fromPathFile("RightPickUp_RightTrench");
            RightTrench_Center2 = PathPlannerPath.fromPathFile("RightTrench_Center2");
            RightCenter_Pickup2 = PathPlannerPath.fromPathFile("RightCenter_Pickup2");
            RightPickUp_RightTrench2 = PathPlannerPath.fromPathFile("RightPickUp_RightTrench2");
            emptyRightTrench = PathPlannerPath.fromPathFile("emptyRightTrench");
            LeftTrench_Center26 = PathPlannerPath.fromPathFile("LeftTrench_Center26");
            // RightCenter_Pickup = PathPlannerPath.fromPathFile("RightCenter_Pickup");
            LeftPickUp_LeftTrench6 = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrench6");
            emptyLeftTrench6 = PathPlannerPath.fromPathFile("emptyLeftTrench6");
            LeftTrench_Center6 = PathPlannerPath.fromPathFile("LeftTrench_Center6");
            LeftPickUp_LeftTrench26 = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrench26");
            LeftCenter_Pickup26 = PathPlannerPath.fromPathFile("LeftCenter_Pickup26");
            LeftTrench_Center102 = PathPlannerPath.fromPathFile("LeftTrench_Center102");
            COMEBACK10 = PathPlannerPath.fromPathFile("COMEBACK10");
            LeftTrench_Center10 = PathPlannerPath.fromPathFile("LeftTrench_Center10");
            _Depot = PathPlannerPath.fromPathFile("_Depot");
            emptyBump = PathPlannerPath.fromPathFile("emptyBump");
            emptycenter = PathPlannerPath.fromPathFile("emptycenter");
            Middle_Depot = PathPlannerPath.fromPathFile("Middle_Depot");
            RightPickUp_RightTrenchMoreCenter = PathPlannerPath.fromPathFile("RightPickUp_RightTrenchMoreCenter");
            RightTrenchMoreCenter_Center2 = PathPlannerPath.fromPathFile("RightTrenchMoreCenter_Center2");
            Right_2_Sweeps = PathPlannerPath.fromPathFile("Right_2_Sweeps");
            Left_2_Sweeps = PathPlannerPath.fromPathFile("Left_2_Sweeps");
            Center_2 = PathPlannerPath.fromPathFile("Center_2");
            Center_3 = PathPlannerPath.fromPathFile("Center_3");


        
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }
}
