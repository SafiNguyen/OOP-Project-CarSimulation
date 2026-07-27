#ifndef MOTION_PATH_H
#define MOTION_PATH_H

#include <cstddef>
#include <memory>
#include <vector>

#include "Geometry.h"

class MotionPath {
public:
    virtual ~MotionPath() = default;
    virtual double getLength() const = 0;
    virtual Pose2D sampleByDistance(double distanceMetres) const = 0;
};

class StraightLanePath final : public MotionPath {
private:
    Vec2 start_;
    Vec2 end_;
    double lengthMetres_;

public:
    StraightLanePath(Vec2 start, Vec2 end, double lengthMetres = -1.0);
    double getLength() const override;
    Pose2D sampleByDistance(double distanceMetres) const override;
};

class BezierJunctionPath final : public MotionPath {
private:
    struct ArcSample {
        double t;
        double distanceMetres;
    };

    Vec2 p0_;
    Vec2 p1_;
    Vec2 p2_;
    Vec2 p3_;
    double metresPerWorldUnit_;
    double lengthMetres_ = 0.0;
    std::vector<ArcSample> arcTable_;

    Vec2 positionAt(double t) const;
    Vec2 derivativeAt(double t) const;
    Vec2 secondDerivativeAt(double t) const;

public:
    BezierJunctionPath(Vec2 p0,
                       Vec2 p1,
                       Vec2 p2,
                       Vec2 p3,
                       double metresPerWorldUnit = 1.0,
                       std::size_t samples = 48);

    double getLength() const override;
    Pose2D sampleByDistance(double distanceMetres) const override;

    Vec2 getStartPoint() const { return p0_; }
    Vec2 getEndPoint() const { return p3_; }
};

class CircularArcPath final : public MotionPath {
private:
    Vec2 centre_;
    double radiusWorld_;
    double startAngle_;
    double sweepRadians_;
    double metresPerWorldUnit_;

public:
    CircularArcPath(Vec2 centre,
                    double radiusWorld,
                    double startAngle,
                    double sweepRadians,
                    double metresPerWorldUnit = 1.0);

    double getLength() const override;
    Pose2D sampleByDistance(double distanceMetres) const override;
};

class CompositePath final : public MotionPath {
private:
    std::vector<std::shared_ptr<const MotionPath>> segments_;
    std::vector<double> cumulativeLengths_;
    double lengthMetres_ = 0.0;

public:
    explicit CompositePath(
        std::vector<std::shared_ptr<const MotionPath>> segments);

    double getLength() const override;
    Pose2D sampleByDistance(double distanceMetres) const override;
    const std::vector<std::shared_ptr<const MotionPath>>& getSegments() const {
        return segments_;
    }
};

class RoundaboutTraversalPath final : public MotionPath {
private:
    CompositePath path_;
    double circulatingRadiusMetres_;

public:
    RoundaboutTraversalPath(Vec2 centre,
                            double circulatingRadiusWorld,
                            Vec2 entryPoint,
                            Vec2 incomingTangent,
                            Vec2 exitPoint,
                            Vec2 outgoingTangent,
                            bool clockwise,
                            double metresPerWorldUnit = 1.0,
                            bool forceFullCircle = false);

    double getLength() const override;
    Pose2D sampleByDistance(double distanceMetres) const override;
    double getCirculatingRadiusMetres() const {
        return circulatingRadiusMetres_;
    }
};

#endif
