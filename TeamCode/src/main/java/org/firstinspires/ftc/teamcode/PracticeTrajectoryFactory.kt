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

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.atomicrobotics.cflib.trajectories.*
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.Constants.drive

/**
 * This class contains all of the RoadRunner trajectories and start positions in the project. It's
 * used by the ExampleRoutines class. You can find how to use each of the possible trajectory
 * segments (like back and splineToSplineHeading) here:
 * https://learnroadrunner.com/trajectorybuilder-functions.html
 */
object PracticeTrajectoryFactory : TrajectoryFactory() {

    lateinit var startPose: Pose2d
    lateinit var firstTrajectory: ParallelTrajectory
    lateinit var secondTrajectory: ParallelTrajectory

    /**
     * Initializes the robot's start positions and trajectories. This is where the trajectories are
     * actually created.
     */
    override fun initialize() {
        super.initialize()
        // start positions
        startPose = Pose2d(0.0, 0.0, 0.0)
        // trajectories
        firstTrajectory = drive.trajectoryBuilder(startPose)
            .lineTo(Vector2d(0.0, 30.0))
            .build()
        secondTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
            .lineTo(Vector2d(0.0, 30.0))
            .build()
    }
}