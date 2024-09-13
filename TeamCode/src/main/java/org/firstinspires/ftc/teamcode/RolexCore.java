package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RolexCore extends Robot {
    public enum AllianceType {
        BLUE, RED
    }

    public final AllianceType ALLIANCE_TYPE;

    public RolexCore(HardwareMap hardwareMap, AllianceType allianceType){
        ALLIANCE_TYPE = allianceType;
    }

}
