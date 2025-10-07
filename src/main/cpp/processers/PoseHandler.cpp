#include "processers/PoseHandler.h"
#include "utilities/RobotLogs.h"

PoseHandler::PoseHandler() {
}

frc::Pose2d PoseHandler::GetPose() {
    return _poseEstimator.GetEstimatedPosition();
}

frc::Pose2d PoseHandler::GetSimPose() {
    return _simPoseEstimator.GetEstimatedPosition();
}

void PoseHandler::Update(frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states) {
    _poseEstimator.Update(angle, states);
    Dashboard::FieldDisplay::GetInstance().SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void PoseHandler::UpdateSim(frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states, bool resetHeading, frc::Rotation2d heading) {
    _simPoseEstimator.Update(angle, states);
    if (resetHeading) { _simPoseEstimator.ResetRotation(heading); }
    Dashboard::FieldDisplay::GetInstance().DisplayPose("Sim pose", _simPoseEstimator.GetEstimatedPosition());
}