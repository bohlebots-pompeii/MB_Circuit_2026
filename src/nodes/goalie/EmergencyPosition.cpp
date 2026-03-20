#include <nodes/goalie/EmergencyPosition.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <cmath>

EmergencyPosition::EmergencyPosition(std::shared_ptr<MotionController> motion)
    : BT::BehaviorNode("EmergencyPosition"), _motion(std::move(motion)) {}

BT::Status EmergencyPosition::tick(const WorldState& ws) {
    if (!(!ws.ballExists && ws.peerBallValid && ws.peerAlive)) {
        return BT::Status::FAILURE;
    }

    const Vector2 emergencyBall = getEmergencyBallVec(ws);
    Vector2 target;
    float rotInput = ws.awayFromOwnGoalAngle;
    bool usePID = true;

    if (const double emergencyDist = emergencyBall.getMagnitude(); emergencyDist > 1.0) {
        target = getHalfCircleTarget(ws, &emergencyBall);
    } else {
        target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
        rotInput = ws.heading;
        usePID = true;
    }

    auto [vx, vy, rot] = _motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);

    return BT::Status::RUNNING;
}

Vector2 EmergencyPosition::getEmergencyBallVec(const WorldState& ws) {
    if constexpr (!GeneralConfig::USE_COMMUNICATION) {
        return {0, 0};
    }

    // "mate sees ball" check done in condition?
    // ws.peerBallValid is flag 3.
    // ws.peerAlive.

    // In Goalie.cpp: if (!peerAlive || !flag3) return {0,0}.
    // Here condition is met.

    if (ws.peerBallDist <= 0) {
        return {0, 0};
    }

    const double pGlobalX = ws.peerGlobalX;
    const double pGlobalY = ws.peerGlobalY;

    const double ballAngleGlobal = toRad(ws.peerBallRot + ws.peerHeading);
    const double ballGlobalX = pGlobalX + cos(ballAngleGlobal) * ws.peerBallDist;
    const double ballGlobalY = pGlobalY + sin(ballAngleGlobal) * ws.peerBallDist;

    const double myGlobalX = ws.globalX;
    const double myGlobalY = ws.globalY;

    const double diffGlobalX = ballGlobalX - myGlobalX;
    const double diffGlobalY = ballGlobalY - myGlobalY;

    const double myHeadingRad = toRad(ws.heading);
    const double localX =  diffGlobalX * cos(-myHeadingRad) - diffGlobalY * sin(-myHeadingRad);
    const double localY =  diffGlobalX * sin(-myHeadingRad) + diffGlobalY * cos(-myHeadingRad);

    emergencyBallAvgX.addValue(localX);
    emergencyBallAvgY.addValue(localY);

    return {emergencyBallAvgX.getAverage(), emergencyBallAvgY.getAverage()};
}

Vector2 EmergencyPosition::getHalfCircleTarget(const WorldState& ws, const Vector2* ballVecOverride) {

    const Vector2 ownGoalVec = ws.ownGoalVec;
    const Vector2 ballVec = *ballVecOverride;

    Vector2 goalToBall = ballVec - ownGoalVec;

    if (const double gtbMag = goalToBall.getMagnitude(); gtbMag < 1e-3) {
        goalToBall = ownGoalVec * -1.0;
    }
    goalToBall.normalize();

    Vector2 awayFromGoal = ownGoalVec * -1.0;
    awayFromGoal.normalize();
    if (const double dot = goalToBall.getX() * awayFromGoal.getX() + goalToBall.getY() * awayFromGoal.getY(); dot < 0) {
        const Vector2 perp(-awayFromGoal.getY(), awayFromGoal.getX());
        const double projPerp = goalToBall.getX() * perp.getX() + goalToBall.getY() * perp.getY();
        goalToBall = perp * (projPerp >= 0 ? 1.0 : -1.0);
    }

    return ownGoalVec + goalToBall * Goalie::HALF_CIRCLE_RADIUS;
}

