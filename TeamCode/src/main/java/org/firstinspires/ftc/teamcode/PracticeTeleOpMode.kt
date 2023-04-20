/*
    Copyright (c) 2022 Atomic Robotics (https://atomicrobotics3805.org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see https://www.gnu.org/licenses/.
*/
package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive
import com.atomicrobotics.cflib.driving.localizers.MecanumDriveWheelLocalizer
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer
import com.atomicrobotics.cflib.opmodes.TeleOpMode

/**
 * This class is an example of how you can create an TeleOp OpMode. Everything is handled by the
 * TeleOpMode parent class, so all you have to do is pass in the constructor parameters.
 */
@TeleOp(name = "Joseph's TeleOp OpMode")
class PracticeTeleOpMode : TeleOpMode(
    PracticeControls,
    Constants.Color.UNKNOWN,
    null,
    null,
    null,
    MecanumDrive(
        PracticeMecanumDriveConstants,
        TwoWheelOdometryLocalizer(PracticeOdometryConstants)
    ),
    PracticeClaw,
    PracticeLift
)