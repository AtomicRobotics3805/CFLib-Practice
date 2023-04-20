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

import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.CommandGroup
import com.atomicrobotics.cflib.Constants.drive
import com.atomicrobotics.cflib.example.mechanisms.Claw
import com.atomicrobotics.cflib.example.mechanisms.Lift
import com.atomicrobotics.cflib.example.trajectoryfactory.ExampleTrajectoryFactory
import com.atomicrobotics.cflib.parallel
import com.atomicrobotics.cflib.sequential
import com.atomicrobotics.cflib.utilCommands.Delay

/**
 * This class is an example of how to create routines. Routines are essentially just groups of
 * commands that can be run either one at a time (sequentially) or all at once (in parallel).
 */
object PracticeRoutines {
    val myDriveRoutine1: CommandGroup
        get() = sequential {
            +drive.followTrajectory(PracticeTrajectoryFactory.firstTrajectory)
        }
    val myDriveRoutine2: CommandGroup
        get() = sequential {
            +drive.followTrajectory(PracticeTrajectoryFactory.secondTrajectory)
        }
    val myDriveRoutine3: CommandGroup
        get() = sequential {
            +drive.followTrajectory(PracticeTrajectoryFactory.thirdTrajectory)
        }
    val myLowLiftRoutine: CommandGroup
        get() = sequential {
            +PracticeLift.toLow
    }
    val myBottomLiftRoutine: CommandGroup
        get() = sequential {
            +PracticeLift.toBottom
        }
    val myHighLiftRoutine: CommandGroup
        get() = sequential {
            +PracticeLift.toHigh
        }
    val openClawRoutine: CommandGroup
        get() = sequential {
            +PracticeClaw.open
            +Delay(5.0)
        }
    val closeClawRoutine: CommandGroup
        get() = sequential {
            +PracticeClaw.close
        }
    val combinedRoutine1: CommandGroup
        get() = parallel {
            +openClawRoutine
            +combinedRoutine2
        }
    val combinedRoutine2: CommandGroup
        get() = sequential {
            +myDriveRoutine1
            +myLowLiftRoutine
            +closeClawRoutine
            +myHighLiftRoutine
            +combinedRoutine3
        }
    val combinedRoutine3: CommandGroup
        get() = parallel {
            +myBottomLiftRoutine
            +combinedRoutine4
        }
    val combinedRoutine4: CommandGroup
        get() = sequential {
            +myDriveRoutine2
            +openClawRoutine
            +myDriveRoutine3
        }


}