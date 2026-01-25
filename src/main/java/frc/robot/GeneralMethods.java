// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class GeneralMethods {

    //"isBlacklist" determines whether the function returns true if comparisonNumber is inside the tolerance (a whitelist) or false (a blacklist)

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

}
