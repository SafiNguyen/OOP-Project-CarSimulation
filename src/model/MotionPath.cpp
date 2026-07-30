#include "MotionPath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double MIN_SCALE = 1e-6;
constexpr double ROUNDABOUT_BLEND_ANGLE = PI / 6.0;

Pose2D poseFromDerivatives(Vec2 position,
                           Vec2 first,
                           Vec2 second,
                           double metresPerWorldUnit) {
    const double derivativeLengthSquared = lengthSquared(first);
    Pose2D pose;
    pose.position = position;
    if (derivativeLengthSquared <= 1e-18) {
        pose.headingRadians = 0.0;
        pose.curvature = 0.0;
        return pose;
    }

    pose.headingRadians = std::atan2(first.y, first.x);
    const double denominator =
        std::pow(derivativeLengthSquared, 1.5);
    const double worldCurvature =
        denominator > 1e-18 ? cross(first, second) / denominator : 0.0;
    pose.curvature =
        worldCurvature / std::max(metresPerWorldUnit, MIN_SCALE);
    return pose;
}

double clockwiseSweep(double start, double finish) {
    double sweep = finish - start;
    while (sweep >= 0.0) sweep -= 2.0 * PI;
    while (sweep < -2.0 * PI) sweep += 2.0 * PI;
    if (std::fabs(sweep) < 1e-6) sweep = -2.0 * PI;
    return sweep;
}

double counterClockwiseSweep(double start, double finish) {
    double sweep = finish - start;
    while (sweep <= 0.0) sweep += 2.0 * PI;
    while (sweep > 2.0 * PI) sweep -= 2.0 * PI;
    if (std::fabs(sweep) < 1e-6) sweep = 2.0 * PI;
    return sweep;
}

std::vector<std::shared_ptr<const MotionPath>> buildRoundaboutSegments(
    Vec2 centre,
    double radiusWorld,
    Vec2 entryPoint,
    Vec2 incomingTangent,
    Vec2 exitPoint,
    Vec2 outgoingTangent,
    bool clockwise,
    double metricScale,
    bool forceFullCircle) {
    radiusWorld = std::max(radiusWorld, 1e-3);
    const Vec2 rawEntryRadial =
        normalized(entryPoint - centre, {-1.0, 0.0});
    const Vec2 rawExitRadial =
        normalized(exitPoint - centre, {1.0, 0.0});
    const double directionSign = clockwise ? -1.0 : 1.0;
    const double entryAngle =
        std::atan2(rawEntryRadial.y, rawEntryRadial.x) +
        directionSign * ROUNDABOUT_BLEND_ANGLE;
    const double exitAngle =
        std::atan2(rawExitRadial.y, rawExitRadial.x) -
        directionSign * ROUNDABOUT_BLEND_ANGLE;
    const Vec2 entryRadial{
        std::cos(entryAngle), std::sin(entryAngle)
    };
    const Vec2 exitRadial{
        std::cos(exitAngle), std::sin(exitAngle)
    };
    const Vec2 entryMerge = centre + entryRadial * radiusWorld;
    const Vec2 exitMerge = centre + exitRadial * radiusWorld;
    const Vec2 entryCircleTangent =
        directionSign * Vec2{-std::sin(entryAngle), std::cos(entryAngle)};
    const Vec2 exitCircleTangent =
        directionSign * Vec2{-std::sin(exitAngle), std::cos(exitAngle)};

    const double entryGap = distance(entryPoint, entryMerge);
    const double exitGap = distance(exitPoint, exitMerge);
    const double entryRadialClearance = std::max(
        0.0, distance(entryPoint, centre) - radiusWorld);
    const double exitRadialClearance = std::max(
        0.0, distance(exitPoint, centre) - radiusWorld);
    const double entryRoadControl =
        std::max(1e-3, entryRadialClearance * 0.40);
    const double exitRoadControl =
        std::max(1e-3, exitRadialClearance * 0.40);
    const double entryCircleControl =
        std::max(radiusWorld * 0.28, entryGap * 0.55);
    const double exitCircleControl =
        std::max(radiusWorld * 0.28, exitGap * 0.55);

    auto entry = std::make_shared<BezierJunctionPath>(
        entryPoint,
        entryPoint + normalized(incomingTangent) * entryRoadControl,
        entryMerge - entryCircleTangent * entryCircleControl,
        entryMerge,
        metricScale);

    double sweep = clockwise
        ? clockwiseSweep(entryAngle, exitAngle)
        : counterClockwiseSweep(entryAngle, exitAngle);
    if (forceFullCircle &&
        std::fabs(sweep) < PI) {
        sweep += clockwise ? -2.0 * PI : 2.0 * PI;
    }
    auto circle = std::make_shared<CircularArcPath>(
        centre, radiusWorld, entryAngle, sweep, metricScale);

    auto exit = std::make_shared<BezierJunctionPath>(
        exitMerge,
        exitMerge + exitCircleTangent * exitCircleControl,
        exitPoint - normalized(outgoingTangent) * exitRoadControl,
        exitPoint,
        metricScale);

    return {std::move(entry), std::move(circle), std::move(exit)};
}

} // namespace

StraightLanePath::StraightLanePath(Vec2 start, Vec2 end, double lengthMetres)
    : start_(start),
      end_(end),
      lengthMetres_(lengthMetres >= 0.0 ? lengthMetres : distance(start, end)) {
}

double StraightLanePath::getLength() const {
    return lengthMetres_;
}

Pose2D StraightLanePath::sampleByDistance(double distanceMetres) const {
    const double ratio = lengthMetres_ > 1e-12
        ? std::clamp(distanceMetres / lengthMetres_, 0.0, 1.0)
        : 0.0;
    const Vec2 tangent = normalized(end_ - start_);
    return {lerp(start_, end_, ratio),
            std::atan2(tangent.y, tangent.x),
            0.0};
}

BezierJunctionPath::BezierJunctionPath(Vec2 p0,
                                       Vec2 p1,
                                       Vec2 p2,
                                       Vec2 p3,
                                       double metresPerWorldUnit,
                                       std::size_t samples)
    : p0_(p0),
      p1_(p1),
      p2_(p2),
      p3_(p3),
      metresPerWorldUnit_(std::max(metresPerWorldUnit, MIN_SCALE)) {
    samples = std::clamp<std::size_t>(samples, 8, 256);
    arcTable_.reserve(samples + 1);
    arcTable_.push_back({0.0, 0.0});
    Vec2 previous = p0_;
    for (std::size_t index = 1; index <= samples; ++index) {
        const double t = static_cast<double>(index) /
                         static_cast<double>(samples);
        const Vec2 current = positionAt(t);
        lengthMetres_ +=
            distance(previous, current) * metresPerWorldUnit_;
        arcTable_.push_back({t, lengthMetres_});
        previous = current;
    }
}

Vec2 BezierJunctionPath::positionAt(double t) const {
    t = std::clamp(t, 0.0, 1.0);
    const double u = 1.0 - t;
    return p0_ * (u * u * u)
         + p1_ * (3.0 * u * u * t)
         + p2_ * (3.0 * u * t * t)
         + p3_ * (t * t * t);
}

Vec2 BezierJunctionPath::derivativeAt(double t) const {
    t = std::clamp(t, 0.0, 1.0);
    const double u = 1.0 - t;
    return (p1_ - p0_) * (3.0 * u * u)
         + (p2_ - p1_) * (6.0 * u * t)
         + (p3_ - p2_) * (3.0 * t * t);
}

Vec2 BezierJunctionPath::secondDerivativeAt(double t) const {
    t = std::clamp(t, 0.0, 1.0);
    return (p2_ - p1_ * 2.0 + p0_) * (6.0 * (1.0 - t))
         + (p3_ - p2_ * 2.0 + p1_) * (6.0 * t);
}

double BezierJunctionPath::getLength() const {
    return lengthMetres_;
}

Pose2D BezierJunctionPath::sampleByDistance(double distanceMetres) const {
    if (arcTable_.empty() || lengthMetres_ <= 1e-12) {
        Vec2 derivative = p3_ - p0_;
        if (lengthSquared(derivative) <= 1e-18) {
            derivative = p1_ - p0_;
        }
        return poseFromDerivatives(
            p0_, derivative, {}, metresPerWorldUnit_);
    }

    const double clamped =
        std::clamp(distanceMetres, 0.0, lengthMetres_);
    auto upper = std::lower_bound(
        arcTable_.begin(), arcTable_.end(), clamped,
        [](const ArcSample& sample, double value) {
            return sample.distanceMetres < value;
        });

    double t = 0.0;
    if (upper == arcTable_.begin()) {
        t = upper->t;
    } else if (upper == arcTable_.end()) {
        t = 1.0;
    } else {
        const ArcSample& next = *upper;
        const ArcSample& previous = *(upper - 1);
        const double span = next.distanceMetres - previous.distanceMetres;
        const double fraction = span > 1e-12
            ? (clamped - previous.distanceMetres) / span
            : 0.0;
        t = previous.t + (next.t - previous.t) * fraction;
    }

    Vec2 derivative = derivativeAt(t);
    if (lengthSquared(derivative) <= 1e-18) {
        const double delta = 1e-4;
        derivative = positionAt(std::min(1.0, t + delta))
                   - positionAt(std::max(0.0, t - delta));
    }
    return poseFromDerivatives(
        positionAt(t), derivative, secondDerivativeAt(t),
        metresPerWorldUnit_);
}

CircularArcPath::CircularArcPath(Vec2 centre,
                                 double radiusWorld,
                                 double startAngle,
                                 double sweepRadians,
                                 double metresPerWorldUnit)
    : centre_(centre),
      radiusWorld_(std::max(radiusWorld, 1e-6)),
      startAngle_(startAngle),
      sweepRadians_(sweepRadians),
      metresPerWorldUnit_(std::max(metresPerWorldUnit, MIN_SCALE)) {
}

double CircularArcPath::getLength() const {
    return std::fabs(sweepRadians_) * radiusWorld_ * metresPerWorldUnit_;
}

Pose2D CircularArcPath::sampleByDistance(double distanceMetres) const {
    const double pathLength = getLength();
    const double ratio = pathLength > 1e-12
        ? std::clamp(distanceMetres / pathLength, 0.0, 1.0)
        : 0.0;
    const double angle = startAngle_ + sweepRadians_ * ratio;
    const double sign = sweepRadians_ < 0.0 ? -1.0 : 1.0;
    const Vec2 tangent =
        sign * Vec2{-std::sin(angle), std::cos(angle)};
    return {
        centre_ + Vec2{std::cos(angle), std::sin(angle)} * radiusWorld_,
        std::atan2(tangent.y, tangent.x),
        sign / (radiusWorld_ * metresPerWorldUnit_)
    };
}

CompositePath::CompositePath(
    std::vector<std::shared_ptr<const MotionPath>> segments)
    : segments_(std::move(segments)) {
    cumulativeLengths_.reserve(segments_.size());
    for (const auto& segment : segments_) {
        if (segment != nullptr) {
            lengthMetres_ += std::max(0.0, segment->getLength());
        }
        cumulativeLengths_.push_back(lengthMetres_);
    }
}

double CompositePath::getLength() const {
    return lengthMetres_;
}

Pose2D CompositePath::sampleByDistance(double distanceMetres) const {
    if (segments_.empty()) {
        return {};
    }

    const double clamped =
        std::clamp(distanceMetres, 0.0, lengthMetres_);
    auto upper = std::lower_bound(
        cumulativeLengths_.begin(), cumulativeLengths_.end(), clamped);
    std::size_t index = upper == cumulativeLengths_.end()
        ? cumulativeLengths_.size() - 1
        : static_cast<std::size_t>(
              std::distance(cumulativeLengths_.begin(), upper));
    while (index < segments_.size() && segments_[index] == nullptr) {
        ++index;
    }
    if (index >= segments_.size()) {
        return {};
    }
    const double previousLength =
        index == 0 ? 0.0 : cumulativeLengths_[index - 1];
    return segments_[index]->sampleByDistance(clamped - previousLength);
}

RoundaboutTraversalPath::RoundaboutTraversalPath(
    Vec2 centre,
    double circulatingRadiusWorld,
    Vec2 entryPoint,
    Vec2 incomingTangent,
    Vec2 exitPoint,
    Vec2 outgoingTangent,
    bool clockwise,
    double metresPerWorldUnit,
    bool forceFullCircle)
    : path_(buildRoundaboutSegments(
          centre,
          circulatingRadiusWorld,
          entryPoint,
          incomingTangent,
          exitPoint,
          outgoingTangent,
          clockwise,
          metresPerWorldUnit,
          forceFullCircle)),
      circulatingRadiusMetres_(
          circulatingRadiusWorld *
          std::max(metresPerWorldUnit, MIN_SCALE)) {
}

double RoundaboutTraversalPath::getLength() const {
    return path_.getLength();
}

Pose2D RoundaboutTraversalPath::sampleByDistance(
    double distanceMetres) const {
    return path_.sampleByDistance(distanceMetres);
}
