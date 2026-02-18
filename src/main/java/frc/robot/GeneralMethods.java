// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class GeneralMethods {

    /**
     * @param negativeTolerance the negative tolerance 
     * @param positiveTolerance the positive tolerance
     * @param comparisonNumber the number to compare to the negative and positive tolerances
     * @param isBlacklist determines whether the function returns true if comparisonNumber is inside the tolerance (a whitelist) or false (a blacklist)
     * @return
     */
    public boolean compareToTolerance(Double negativeTolerance, Double positiveTolerance, Double comparisonNumber, boolean isBlacklist) {
        if (isBlacklist) {
            if (comparisonNumber <= negativeTolerance || comparisonNumber >= positiveTolerance) {
                return true;
            } else {
                return false;
            }
        } else {
            if (comparisonNumber >= negativeTolerance && comparisonNumber <= positiveTolerance) {
                return true;
            } else {
                return false;
            }
        }
    }

    //Presumes the secondary Translation2d is from (0,0), aka bot center
    public Rotation2d calculateAngleToPoint(Translation2d point) {
        double theta = Math.atan2(point.getY(), point.getX());
        return Rotation2d.fromRadians(theta);
    }

}
