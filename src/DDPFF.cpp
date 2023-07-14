#include "DDPFF.h"

#include <algorithm>
#include <iostream>
#include <stack>
#include <type_traits>
#include <utility>
#include <memory>

#define BACKGROUND 0

DDPFF::DDPFF(Config& config)
{
    m_config = &config;
    sampleFactor_ = config.sampleFactor;
    height_ = config.IMAGE_HEIGHT;
    width_ = config.IMAGE_WIDTH;
    bufferSize_ = height_ * width_;
}

bool DDPFF::getSeed(const size_t& row, const size_t& col, Seed& seed, size_t& startRadius) {
    static Vec3                  diff, normal;
    static std::array< Vec3, 4 > normals, points;
    static double                avgNorth, avgSouth, avgEast, avgWest;

    seed.point.fill(0);
    seed.normal.fill(0);

    const Vec3& center = (*m_pointCloud)[width_ * row + col];

    if (notValid(center)) { return false; }

    diff.fill(0);
    normal.fill(0);
    for (size_t i = 0; i < 4; i++) {
        normals[i].fill(0);
        points[i].fill(0);
    }

    Vec3 &north = points[0], &west = points[1], &south = points[2], &east = points[3];
    Vec3 &westSouth = normals[0], &southEast = normals[1], &eastNorth = normals[2], &northWest = normals[3];

    size_t range = rangeFunc(center.squaredNorm());

    if (!utils::lix< size_t >(row - range, 0, height_) || !utils::lix< size_t >(row + range, 0, height_)) { return false; }

    if (!utils::lix< size_t >(col - range, 0, width_) || !utils::lix< size_t >(col + range, 0, width_)) { return false; }

    // double threshold = pointDistThreshold(center.squaredNorm());
    // threshold *= fabs(center.normalize().dot(seed.normal));

    // Find the four corners for cross product based vector calculation
    north = (*m_pointCloud)[width_ * (row - range) + col];
    south = (*m_pointCloud)[width_ * (row + range) + col];
    west  = (*m_pointCloud)[width_ * row + (col - range)];
    east  = (*m_pointCloud)[width_ * row + (col + range)];

    north -= center;
    south -= center;
    east -= center;
    west -= center;
    avgNorth = north.squaredNorm();
    avgSouth = south.squaredNorm();
    avgEast  = east.squaredNorm();
    avgWest  = west.squaredNorm();

    // constrain the distance between pixels
    // if (avgNorth > threshold * range/2 || avgSouth > threshold * range/2 || avgEast > threshold *range|| avgWest > threshold*range)
    // {
    //     return false;
    // }

    southEast = south.cross(east);
    eastNorth = east.cross(north);
    northWest = north.cross(west);
    westSouth = west.cross(south);
    southEast.normalize();
    eastNorth.normalize();
    northWest.normalize();
    westSouth.normalize();

    for (const Vec3& n : normals) {
        if (notValid(n)) { return false; }
    }

    // constrain the normal directions
    // if (fabs(southEast.dot(eastNorth)) < 0.9 || fabs(northWest.dot(westSouth)) < 0.9 || fabs(southEast.dot(northWest) < 0.9) )
    // {
    //     return false;
    // }

    normal.fill(0);
    for (const auto& n : normals) { normal += n; }
    normal /= 4;
    normal.normalize();

    seed.point = (*m_pointCloud)[width_ * row + col];
    // std::cout<<(seed.point)<<std::endl;
    seed.normal = normal;
    startRadius = range;

    return true;
}

void DDPFF::planeAverage(const DDPFF::planeSegParamsPtr_t& plane1, const DDPFF::planeSegParamsPtr_t& plane2, const DDPFF::planeSegParamsPtr_t& target) {
    static Vec3 avgPoint, avgNormal, tmp;

    const size_t totalPoints = plane1->count + plane2->count;

    // Update both of their parameters with their weighted average
    double plane1Wt = (plane1->count * 1.0) / totalPoints;
    double plane2Wt = (plane2->count * 1.0) / totalPoints;

    tmp.fill(0);
    avgNormal.fill(0);
    avgPoint.fill(0);

    tmp = plane1->point;
    tmp *= plane1Wt;
    avgPoint += tmp;
    tmp = plane2->point;
    tmp *= plane2Wt;
    avgPoint += tmp;

    tmp = plane1->normal;
    tmp *= plane1Wt;
    avgNormal += tmp;
    tmp = plane2->normal;
    tmp *= plane2Wt;
    avgNormal += tmp;
    avgNormal.normalize();

    target->point     = avgPoint;
    target->normal    = avgNormal;
    target->count     = totalPoints;
    target->pixel.x() = int(plane1->pixel.x() * plane1Wt) + int(plane2->pixel.x() * plane2Wt);
    target->pixel.y() = int(plane1->pixel.y() * plane1Wt) + int(plane2->pixel.y() * plane2Wt);

    // Update maximum depth
    target->maxDepth = std::max(plane1->maxDepth, plane2->maxDepth);
}

void DDPFF::transferParameters(const DDPFF::planeSegParamsPtr_t& target, const DDPFF::planeSegParamsPtr_t& source) {

    planeAverage(source, target, target);

    // Make the target plane the neighbour of the neighbours of the source plane
    for (const auto& neighbourIndex : source->neighbours) { mergedPlanes_[neighbourIndex]->neighbours.insert(target->planeId); }

    // Copy the source plane's neighbours into the target plane
    std::copy(source->neighbours.begin(), source->neighbours.end(), std::inserter(target->neighbours, target->neighbours.end()));

    // Copy the source plane inliers into the target plane
    std::copy(source->inliers.begin(), source->inliers.end(), std::inserter(target->inliers, target->inliers.end()));

    // Copy earlier merged planes information
    std::copy(source->mergeWith.begin(), source->mergeWith.end(), std::inserter(target->mergeWith, target->mergeWith.end()));

    // This plane is also merging
    target->mergeWith.insert(source->planeId);
}

double DDPFF::mutualDistance(const PlaneSegParams& plane1, const PlaneSegParams& plane2) {
    const Vec3 &n1 = plane1.normal, &n2 = plane2.normal;
    const Vec3 &p1 = plane1.point, &p2 = plane2.point;

    // Mutual plane distance
    const double d_plane1 = fabs(n1.dot(p2 - p1));
    const double d_plane2 = fabs(n2.dot(p2 - p1));

    const double d_plane = utils::trunc(std::min(d_plane1, d_plane2), 2);

    return d_plane;
}

double DDPFF::angleDistance(const PlaneSegParams& plane1, const PlaneSegParams& plane2) {
    const Vec3 &n1 = plane1.normal, &n2 = plane2.normal;

    // deviation from perfectly aligned
    // const double d_orientation = utils::trunc(fabs(fabs(n1.dot(n2)) - 1), 2); // alignment distance
    const double d_orientation = utils::trunc(1 - n1.dot(n2), 2);  // alignment distance

    return d_orientation;
}

bool DDPFF::isMergeable_mutual(const PlaneSegParams& plane1, const PlaneSegParams& plane2) {
    const double thresholdPlane = std::min(planeDistThreshold_merge(plane1.maxDepth), planeDistThreshold_merge(plane2.maxDepth));
    const double d_plane        = mutualDistance(plane1, plane2);

    // The planes are sufficiently close; merge them
    if (d_plane <= thresholdPlane) { return true; }

    return false;
}

bool DDPFF::isMergeable_angle(const PlaneSegParams& plane1, const PlaneSegParams& plane2) {
    const double thresholdAngle = std::min(angleDistanceThreshold(plane1.maxDepth), angleDistanceThreshold(plane2.maxDepth));

    // deviation from perfectly aligned
    const double d_orientation = angleDistance(plane1, plane2);

    // The planes are sufficiently close; merge them
    if (d_orientation <= thresholdAngle) { return true; }

    return false;
}

bool DDPFF::isMergeable(const PlaneSegParams& plane1, const PlaneSegParams& plane2) {
    // The planes are sufficiently close; merge them
    if (isMergeable_angle(plane1, plane2) && (isMergeable_mutual(plane1, plane2))) { return true; }

    return false;
}

void DDPFF::merge(bool unidir) {
    static std::vector< size_t > mergeableIndices;
    static std::vector< size_t > removableIndices_bidir, removableIndices_unidir;
    static planeSegParamsPtr_t   mergeTarget = std::make_shared< PlaneSegParams >();

    bool mergeOp = false;

    do {
        mergeOp = false;
        // The zeroeth index is for the background
        for (size_t planeIndex = 1; planeIndex < mergedPlanes_.size(); planeIndex++) {
            planeSegParamsPtr_t plane = mergedPlanes_[planeIndex];

            if (!plane) { continue; }

            if (plane->planeId == BACKGROUND) { continue; }

            if (notValid(plane->normal) || notValid(plane->point)) { continue; }

            size_t targetIndex = plane->planeId;
            mergeableIndices.clear();
            mergeableIndices.push_back(plane->planeId);  // add itself
            removableIndices_bidir.resize(0);
            removableIndices_unidir.resize(0);
            mergeTarget->clear();
            planeAverage(plane, mergeTarget, mergeTarget);

            for (size_t neighbourIndex : plane->neighbours) {
                planeSegParamsPtr_t neighbour = mergedPlanes_[neighbourIndex];

                if (!neighbour) { continue; }

                if (plane == neighbour) { continue; }

                if (!isMergeable(*mergeTarget, *neighbour)) {
                    removableIndices_bidir.push_back(neighbour->planeId);
                    continue;
                }

                if (unidir) {
                    bool mergeable = true;
                    for (const size_t m : plane->mergeWith) {
                        if (!isMergeable(*neighbour, unmergedPlanes_[m])) {
                            removableIndices_unidir.push_back(neighbour->planeId);
                            mergeable = false;
                            break;
                        }
                    }
                    if (!mergeable) { continue; }
                }

                mergeableIndices.push_back(neighbour->planeId);

                // Should we update the merge target before checking merge-ability?
                planeAverage(mergeTarget, neighbour, mergeTarget);

                // Always make the largest plane the merge target
                if (mergedPlanes_[targetIndex]->count < neighbour->count) { targetIndex = neighbour->planeId; }
            }

            // Remove the unmergeable neigbours before transfering parameters
            for (size_t removableIndex : removableIndices_bidir) {
                plane->neighbours.erase(removableIndex);
                mergedPlanes_[removableIndex]->neighbours.erase(plane->planeId);
                plane->disconnected.insert(removableIndex);
                mergedPlanes_[removableIndex]->disconnected.insert(plane->planeId);
            }

            if (unidir) {
                for (size_t removableIndex : removableIndices_unidir) {
                    plane->neighbours.erase(removableIndex);
                    plane->disconnected.insert(removableIndex);
                }
            }

            for (auto mergeableIndex : mergeableIndices) {
                if (mergedPlanes_[targetIndex] == mergedPlanes_[mergeableIndex]) {
                    // Already merged
                    continue;
                }

                transferParameters(mergedPlanes_[targetIndex], mergedPlanes_[mergeableIndex]);

                // Replace the planes that were merged with the current one with the new plane
                for (const auto mergedIndex : mergedPlanes_[mergeableIndex]->mergeWith) { mergedPlanes_[mergedIndex] = mergedPlanes_[targetIndex]; }

                mergedPlanes_[mergeableIndex] = mergedPlanes_[targetIndex];

                mergeOp = true;
            }
        }

        unidir = false;  // Disable merge-set checks after the first iteration.

        for (auto& p : mergedPlanes_) {
            p->neighbours.insert(p->disconnected.begin(), p->disconnected.end());
            p->disconnected.clear();
        }

    } while (mergeOp);
}


// // New implemented version, this version is observed to occasionally cause memory leak when used in FP-Loc.
// void DDPFF::merge(bool unidir)
// {
//     planeSegParamsPtr_t plane =  std::make_shared<PlaneSegParams>();
//     std::vector<bool> merged_index( mergedPlanes_.size(), false);
    
//     for (size_t planeIndex = 1; planeIndex < mergedPlanes_.size(); planeIndex++)
//     {
//         if (merged_index[planeIndex]) continue;
//         merged_index[planeIndex] = true;
//         plane = mergedPlanes_[planeIndex];   

//         if (!plane) continue;
//         if (plane->planeId == BACKGROUND) continue;
//         if (notValid(plane->normal) || notValid(plane->point)) continue;

//         // Keep merging for the current plane until no neighbor
//         while (plane->neighbours.size())
//         {
//             size_t neighbourIndex = *plane->neighbours.begin();
//             plane->neighbours.erase(neighbourIndex);

//             planeSegParamsPtr_t neighbour = mergedPlanes_[neighbourIndex];
//             if (!neighbour) continue;
//             if (neighbour->planeId == BACKGROUND) continue;
//             if (notValid(neighbour->normal) || notValid(neighbour->point)) continue;

//             if (plane == neighbour) continue;
//             if (!isMergeable(*plane, *neighbour)) 
//             {
//                 neighbour->neighbours.erase(planeIndex);
//                 continue;
//             }
//             planeAverage(plane, neighbour, plane);
//             transferParameters(plane, neighbour);
//             merged_index[neighbourIndex] = true;
//             mergedPlanes_[neighbourIndex] = plane;
//         }
//     }
// }

void DDPFF::reorder() {
#if DEBUG_DDPFF == 1
    for (auto& p : mergedPlanes_) { unorderedPlanes_.push_back(*p); }
#endif

    static std::unordered_map< size_t, planeSegParamsPtr_t > finalPlanes;
    finalPlanes.clear();

    for (auto& p : mergedPlanes_) {
        if (finalPlanes.count(p->planeId)) { continue; }

        // Don't add the zero plane
        if (p->planeId == BACKGROUND) { continue; }

        finalPlanes[p->planeId] = p;
    }

    static std::vector< planeSegParamsPtr_t > planeList;
    planeList.clear();

    for (auto& pair : finalPlanes) { planeList.push_back(pair.second); }

    // Sort from largest to smallest
    std::sort(planeList.begin(), planeList.end(), [](const planeSegParamsPtr_t& p1, const planeSegParamsPtr_t& p2) { return p1->count > p2->count; });

    static const planeSegParamsPtr_t zeroPlane = std::make_shared< PlaneSegParams >();

    // Renumber the planes
    mergedPlanes_.clear();
    mergedPlanes_.push_back(zeroPlane);
    for (size_t i = 0; i < planeList.size(); i++) {
        planeList[i]->planeId = i + 1;
        mergedPlanes_.push_back(planeList[i]);
    }

    for (auto& p : mergedPlanes_) {

        // Update label image
        for (auto& index : p->inliers) { mergedLabels_[index] = p->planeId; }

        // Clear neighbour and merge lists
        p->neighbours.clear();
        p->mergeWith.clear();
    }
}

void DDPFF::plugGaps() {
    static const auto& fillConnectedSegment = [this](const size_t& row, const int& dir, size_t& col) {
        const auto curIndex = row * width_ + col;

        if (mergedLabels_[curIndex] == BACKGROUND) {

            if (row - dir < 0 || row + dir >= height_) { return; }

            auto       dirIndex = (row + dir) * width_ + col;
            const auto l        = mergedLabels_[dirIndex];

            if (l == BACKGROUND) { return; }

            int runWidth = 0;
            while (true) {
                if (col + runWidth >= width_) { break; }
                auto index = row * width_ + col + runWidth;
                dirIndex   = (row + dir) * width_ + col + runWidth;
                if (mergedLabels_[index] != BACKGROUND || mergedLabels_[dirIndex] != l) { break; }
                runWidth++;
            }

            if (runWidth <= 0) { return; }

            const auto& mid = (*m_pointCloud)[row * width_ * sampleFactor_ + (col + (runWidth >> 1)) * sampleFactor_];
            if (notValid(mid)) { return; }

            const auto& plane = mergedPlanes_[l];

            if (fabs(plane->normal.dot(plane->point - mid)) < planeDistThreshold_merge(mid.squaredNorm())) {
                for (int i = 0; i < runWidth; i++) {
                    mergedLabels_[row * width_ + col + i] = l;
                    mergedPlanes_[l]->inliers.push_back(row * width_ + col + i);
                }

                mergedPlanes_[l]->count += runWidth;
            }

            col += (runWidth - 1);
        }
    };

    // First pass: top to bottom
    for (size_t row = 0; row < height_; row++) {
        for (size_t col = 0; col < width_; col++) { fillConnectedSegment(row, -1, col); }
    }

    // Second pass: bottom to top
    for (size_t row = height_ - 1; row > 0; row--) {
        for (size_t col = 0; col < width_; col++) { fillConnectedSegment(row, 1, col); }
    }
}

void DDPFF::findInliers() {
    for (size_t row = 0; row < height_; row++) {
        for (size_t col = 0; col < width_; col++) {
            const auto index         = row * width_ + col;
            const auto originalIndex = row * sampleFactor_ * width_ + col * sampleFactor_;

            const auto l = unmergedLabels_[index];

            if (l == BACKGROUND) { continue; }

            if (notValid((*m_pointCloud)[originalIndex])) {
                unmergedLabels_[index] = BACKGROUND;
                continue;
            }

            // Add inlier
            unmergedPlanes_[l].inliers.push_back(index);

            // Update maximum depth
            unmergedPlanes_[l].maxDepth = std::max(unmergedPlanes_[l].maxDepth, (*m_pointCloud)[originalIndex].squaredNorm());
        }
    }
}

void DDPFF::findPointNeighbours() {
    const static auto& checkAndAddNeighbour = [this](const int r1, const int c1, const int r2, const int c2) {
        const size_t idx1 = r1 * width_ + c1;
        const size_t idx2 = r2 * width_ + c2;
        const int    diff = abs((r2 - r1) + (c2 - c1));
        const size_t l1 = unmergedLabels_[idx1], l2 = unmergedLabels_[idx2];
        auto &       p1 = unmergedPlanes_[l1], &p2 = unmergedPlanes_[l2];

        if (l1 == BACKGROUND || l2 == BACKGROUND) { return; }

        if (p1.neighbours.count(l2) && p2.neighbours.count(l1)) { return; }

        const auto& pt1            = (*m_pointCloud)[idx1];
        const auto& pt2            = (*m_pointCloud)[idx2];
        const auto  pointThreshold = std::max(pointDistThreshold(pt1.squaredNorm()), pointDistThreshold(pt2.squaredNorm()));
        if ((pt1 - pt2).squaredNorm() < diff * pointThreshold) {
            p1.neighbours.insert(p2.planeId);
            p2.neighbours.insert(p1.planeId);
        }
    };

    // Two pass algorithm: row pass and column pass

    // Pass 1: rows
    for (size_t row = 0; row < height_; row++) {
        size_t savedLabel  = BACKGROUND;
        size_t savedChange = 0;
        for (size_t col = 1; col < height_; col++) {
            const size_t prvIdx   = row * width_ + col - 1;
            const size_t curIdx   = row * width_ + col;
            const size_t curLabel = unmergedLabels_[curIdx];
            const size_t prvLabel = unmergedLabels_[prvIdx];
            if (curLabel != prvLabel) {
                // change
                if (curLabel == BACKGROUND && prvLabel != BACKGROUND) {
                    // leaving a region into background
                    savedLabel  = prvLabel;
                    savedChange = col - 1;
                } else if (curLabel != BACKGROUND && prvLabel == BACKGROUND) {
                    // entering a region from background
                    if (savedLabel != BACKGROUND) { checkAndAddNeighbour(row, savedChange, row, col); }
                    savedLabel  = curLabel;
                    savedChange = col;
                } else if (curLabel != BACKGROUND && prvLabel != BACKGROUND) {
                    // entering a region from a different region
                    checkAndAddNeighbour(row, col - 1, row, col);
                    savedLabel  = curLabel;
                    savedChange = col;
                }
            }
            // else both are marked same: do nothing
        }
    }

    // Pass 2: cols
    for (size_t col = 0; col < width_; col++) {
        size_t savedLabel  = BACKGROUND;
        size_t savedChange = 0;
        for (size_t row = 1; row < height_; row++) {
            const size_t prvIdx   = (row - 1) * width_ + col;
            const size_t curIdx   = row * width_ + col;
            const size_t curLabel = unmergedLabels_[curIdx];
            const size_t prvLabel = unmergedLabels_[prvIdx];
            if (curLabel != prvLabel) {
                // change
                if (curLabel == BACKGROUND && prvLabel != BACKGROUND) {
                    // leaving a region into background
                    savedLabel  = prvLabel;
                    savedChange = row - 1;
                } else if (curLabel != BACKGROUND && prvLabel == BACKGROUND) {
                    // entering a region from background
                    if (savedLabel != BACKGROUND) { checkAndAddNeighbour(savedChange, col, row, col); }
                    savedLabel  = curLabel;
                    savedChange = row;
                } else if (curLabel != BACKGROUND && prvLabel != BACKGROUND) {
                    // entering a region from a different region
                    checkAndAddNeighbour(row - 1, col, row, col);
                    savedLabel  = curLabel;
                    savedChange = row;
                }
            }
            // else both are marked same: do nothing
        }
    }
}

void DDPFF::computePlaneParams() {
    for (auto& planeParam : unmergedPlanes_) {

        if (planeParam.planeId == BACKGROUND) { continue; }

        size_t validCount = 0;
        planeParam.accumulator3D.fill(0);

        // Compute the centroid of the plane
        for (const size_t index : planeParam.inliers) {
            const auto r             = index / width_;
            const auto c             = index % width_;
            const auto originalIndex = r * sampleFactor_ * width_ + c * sampleFactor_;
            if (notValid((*m_pointCloud)[originalIndex])) { continue; }
            const Vec3& point = (*m_pointCloud)[originalIndex];
            validCount++;
            planeParam.accumulator3D += point;
            planeParam.accumulator2D.x() += c;
            planeParam.accumulator2D.y() += r;
        }

        if (validCount > 0) {
            planeParam.point = planeParam.accumulator3D / validCount;
            planeParam.pixel = planeParam.accumulator2D / validCount;
            planeParam.count = validCount;
            // planeParam.color = colorGenerator_.next();
        }

        planeSegParamsPtr_t planeParamPtr = std::make_shared< PlaneSegParams >();
        *planeParamPtr                    = planeParam;
        mergedPlanes_.push_back(planeParamPtr);
    }

#if DEBUG_DDPFF == 1
    mutualPlanes_ = unmergedPlanes_;
#endif
}

void DDPFF::disconnectUnmergeableNeighbours() {
    static std::vector< size_t > p_removableIndices;

    for (size_t i = 1; i < mergedPlanes_.size(); i++) {
        auto& p = mergedPlanes_[i];

        p_removableIndices.resize(0);
        for (size_t j : p->neighbours) {
            auto&        n                  = mergedPlanes_[j];
            const double p_n_mutDist        = mutualDistance(*p, *n);
            const double p_n_angDist        = angleDistance(*p, *n);
            const double p_n_thresholdPlane = std::min(planeDistThreshold_flood(p->maxDepth), planeDistThreshold_flood(n->maxDepth));
            const double p_n_thresholdAngle = std::min(angleDistanceThreshold(p->maxDepth), angleDistanceThreshold(n->maxDepth));
            if (p_n_mutDist > p_n_thresholdPlane || p_n_angDist > p_n_thresholdAngle) {
                p_removableIndices.push_back(n->planeId);
                continue;
            }
        }

        for (size_t j : p_removableIndices) {
            p->neighbours.erase(j);
            mergedPlanes_[j]->neighbours.erase(i);
#if DEBUG_DDPFF == 1
            mutualPlanes_[i].neighbours.erase(j);
            mutualPlanes_[j].neighbours.erase(i);
            mutualPlanes_[i].disconnected.insert(j);
            mutualPlanes_[j].disconnected.insert(i);
#endif
        }
    }
}

void DDPFF::init() {
    unmergedLabels_.resize(bufferSize_, BACKGROUND);
    mergedLabels_.resize(bufferSize_, BACKGROUND);
}

void DDPFF::clear() {
    std::fill(unmergedLabels_.begin(), unmergedLabels_.end(), BACKGROUND);
    std::fill(mergedLabels_.begin(), mergedLabels_.end(), BACKGROUND);
    // colorGenerator_.reset();
    totalPlaneCount_ = 1;
    unmergedPlanes_.resize(0);
    mergedPlanes_.resize(0);
    static const planeSegParamsPtr_t zeroPlane = std::make_shared< PlaneSegParams >();
    unmergedPlanes_.push_back(*zeroPlane);
    mergedPlanes_.push_back(zeroPlane);

#if DEBUG_DDPFF == 1
    unorderedPlanes_.resize(0);
    seedIndices_.resize(0);
    seedIndices_.push_back(0);
    seedPoints_.resize(0);
    seedPoints_.push_back(Seed());
    seedRadii_.resize(0);
    seedRadii_.push_back(0);
#endif
}

void DDPFF::compute() {
    // static StopWatch sw;
    // sw.start();

    static Seed           seed;
    static PlaneSegParams planeParam;

    clear();

    // sw.start();

    size_t floodTotal = 0;
    size_t seedRadius = m_config->normalSampleDistance_min;

    FloodFill ff(*this);

    // std::cout<<"Start Scanning"<<std::endl;
    // Gather up points into segments
    for (size_t row = 0; row < height_; row++) {
        for (size_t col = 0; col < width_; col += std::max< size_t >(2 * seedRadius, 1))  // startRadius = 0.5 * seed range
        // for (size_t col = 0; col < width_; col += 32) {  // startRadius = 0.5 * seed range
        {
            const size_t idx = row * width_ + col;

            if (unmergedLabels_[idx] != BACKGROUND || notValid((*m_pointCloud)[idx])) {
                // This point already belongs to a segment
                // Or it is missing any information
                continue;
            }

            const bool goodSeed = getSeed(row, col, seed, seedRadius);

            if (!goodSeed) { continue; }

            if (notValid(seed.point) || notValid(seed.normal)) {
                // Mark this point with a label to take it out of consideration
                // for subsequent floods.
                continue;
            }

            const size_t planeId = totalPlaneCount_;

            planeParam.clear();
            planeParam.planeId = planeId;

            const size_t& floodCount = ff.flood(row, col, planeId, seed);
            // const size_t& floodCount = 0;

            if (floodCount == 0) {
                seedRadius = m_config->normalSampleDistance_min;
                continue;
            }

            totalPlaneCount_++;

            floodTotal += floodCount;

            planeParam.point     = seed.point;
            planeParam.seedIndex = idx;
            planeParam.normal    = seed.normal;
            unmergedPlanes_.push_back(planeParam);
        }
    }

    std::cout<<"Unmerged plane count: "<<totalPlaneCount_<<std::endl;

    // Find the euclidean neighbours of each plane before merging
    // std::cout<<"Start finding neighbors"<<std::endl;
    findPointNeighbours();

    // Find the inliers of the new planes.
    // std::cout<<"Start finding inliers"<<std::endl;
    findInliers();

    // Compute centroid and colors
    // std::cout<<"Start computing params"<<std::endl;
    computePlaneParams();

    // Disconnect neighbours that can't be merged
    // disconnectUnmergeableNeighbours();
    // std::cout<<"Start merging"<<std::endl;
    merge();

    // std::cout<<"Start reordering"<<std::endl;
    reorder();

    // std::cout<<"Start pluging"<<std::endl;
    plugGaps();

    std::cout<<"Merged plane count: "<<mergedPlanes_.size()<<std::endl;
}

std::vector< size_t > DDPFF::getLabels() { return mergedLabels_; }

std::vector< PlanePointNormal > DDPFF::getPlanes() {
    static std::vector< PlanePointNormal > planes;
    planes.resize(mergedPlanes_.size());

    for (size_t i = 0; i < mergedPlanes_.size(); i++) {
        auto& plane     = mergedPlanes_[i];
        planes[i].p     = plane->point;
        planes[i].n     = plane->normal;
        planes[i].count = plane->count;
        planes[i].id    = plane->planeId;
        planes[i].inliers.clear();
        planes[i].inliers.insert(plane->inliers.begin(), plane->inliers.end());
    }

    return planes;
}

double DDPFF::planeDistThreshold_flood(const double& x) {
    return 0.05;
    // return std::min(m_config->c_plane * x * x * 1e-3 + m_config->planeThresholdFloodFill_flood, m_config->planeThresholdFloodFill_flood_max);
}

double DDPFF::planeDistThreshold_merge(const double& x) {
    return 0.08;
    // return std::min(m_config->c_plane_merge * x * x * 1e-3 + m_config->planeThresholdFloodFill_merge, m_config->planeThresholdFloodFill_merge_max);
}

double DDPFF::pointDistThreshold(const double& x) {
    // Adjust paras here
    if (isnan(x)) { return m_config->pointThresholdFloodFill_min; }
    return m_config->c_point * x + m_config->pointThresholdFloodFill_min;
    // return std::min(std::pow(sampleFactor_ * m_config->c_point * x * x * 1e-2 + m_config->pointThresholdFloodFill_min, 2), m_config->pointThresholdFloodFill_max * m_config->pointThresholdFloodFill_max);
}

double DDPFF::angleDistanceThreshold(const double& x) {
    return 0.3;
    // return std::min(sampleFactor_ * m_config->c_angle * x * x * 1e-2 + m_config->angleThresholdFloodFill, m_config->angleThresholdFloodFill_max);
}

size_t DDPFF::ctWidthFunc(const double& x) { return rangeFunc(x); }

size_t DDPFF::rangeFunc(const double& x) {
    // Adjust paras here
    if (isnan(x)) {
        std::cout << "nan" << std::endl;
        return static_cast< size_t >(m_config->normalSampleDistance_min);
    }

    return static_cast< size_t >(std::max(m_config->normalSampleDistance_max / (x + 0.01) * m_config->c_range, m_config->normalSampleDistance_min));
    // return static_cast<size_t>(std::min((m_config->c_range / sampleFactor_) * (x * x * 1e-1) + (m_config->normalSampleDistance_min / sampleFactor_), m_config->normalSampleDistance_max));
}

DDPFF::pointDistValidator_t DDPFF::FloodFill::setPointDistFunc() {
    return ([this](const Vec3& p1, const Vec3& p2, const size_t& diff, double& d) -> bool {
        const double threshold = outer_.pointDistThreshold(p1.squaredNorm());
        d                      = (p1 - p2).squaredNorm();
        return d < diff * threshold;
    });
}

void DDPFF::FloodFill::setSeedDistFunc(const Seed& seed) {
    const double pnInnerProduct = seed.point.dot(seed.normal);
    const Vec3&  normal         = seed.normal;

    sdf_ = ([this, normal, pnInnerProduct](const Vec3& p, double& d) -> bool {
        const double threshold = outer_.planeDistThreshold_flood(p.squaredNorm());
        d                      = fabs(p.dot(normal) - pnInnerProduct);
        return d < threshold;
    });
}

bool DDPFF::FloodFill::validateIndex(const size_t& index, const size_t& prvIndex, const size_t& diff) {
    nOps_++;

    if (index == prvIndex) { return true; }

    const Vec3& curData = (*outer_.m_pointCloud)[index];
    const Vec3& prvData = (*outer_.m_pointCloud)[prvIndex];

    if (notValid(curData)) { return false; }

    double ptDist = 0;
    if (!pdf_(curData, prvData, diff, ptDist)) { return false; }

    double seedDist = 0;
    if (!sdf_(curData, seedDist)) { return false; }

    return true;
}

bool DDPFF::FloodFill::validateInRowDir(const int& y, const int& x, const size_t& ctWidth) {
    if (utils::lix< size_t >(y + ctWidth / 2, 0, outer_.height_) && validateIndex((y + ctWidth / 2) * outer_.width_ + x, y * outer_.width_ + x, ctWidth / 2)) {
        // if (utils::lix<size_t>(y + ctWidth, 0, outer_.height_) && validateIndex((y + ctWidth) * outer_.width_ + x, y * outer_.width_ + x,  ctWidth)) {
        return true;
    } else if (utils::lix< size_t >(y - ctWidth / 2, 0, outer_.height_) && validateIndex((y - ctWidth / 2) * outer_.width_ + x, y * outer_.width_ + x, ctWidth / 2)) {
        // } else if (utils::lix<size_t>(y - ctWidth, 0, outer_.height_) && validateIndex((y - ctWidth) * outer_.width_ + x, y * outer_.width_ + x, ctWidth)) {
        return true;
    }
    return false;
}

bool DDPFF::FloodFill::validateInColDir(const int& prevx, int& x, const int& y) {
    size_t current = y * outer_.width_ + x;

    if (!validateIndex(current, y * outer_.width_ + prevx, abs(x - prevx))) { return false; }

    return true;
}

bool DDPFF::FloodFill::validateInColDir_new(const int& y, const int& x, const size_t& ctWidth) {
    if (utils::lix< size_t >(x + ctWidth, 0, outer_.width_) && validateIndex(y * outer_.width_ + x + ctWidth, y * outer_.width_ + x, ctWidth)) {
        return true;
    } else if (utils::lix< size_t >(x - ctWidth, 0, outer_.width_) && validateIndex(y * outer_.width_ + x - ctWidth, y * outer_.width_ + x, ctWidth)) {
        return true;
    }
    return false;
}

void DDPFF::FloodFill::mark(const size_t y, const size_t lx, const size_t rx, const size_t& label) {
    const size_t rowIndex = y * outer_.width_;
    for (size_t x = lx; x <= rx; x++) {
        const size_t index            = rowIndex + x;
        outer_.unmergedLabels_[index] = label;
        floodCounter_++;
    }
}

// New implemented Flood function
size_t DDPFF::FloodFill::flood(const size_t& startRow, const size_t& startCol, const size_t& label, const Seed& seed) {
    const size_t& seedIndex = startRow * outer_.width_ + startCol;
    // Seed label already marked
    if (outer_.unmergedLabels_[seedIndex] != BACKGROUND) return 0;

    if (notValid((*outer_.m_pointCloud)[seedIndex])) return 0;

    setSeedDistFunc(seed);
    floodTracker_.resize(0);
    floodCounter_ = 0;

    if (!utils::lix< size_t >(startCol, 0, outer_.width_) || !utils::lix< size_t >(startRow, 0, outer_.height_)) return 0;

    floodTracker_.emplace_back(startRow, startCol, startCol, -1);  // scan up
    floodTracker_.emplace_back(startRow, startCol, startCol, 1);   // scan down

    int x, l_most, r_most, lastX = 0;

    while (!floodTracker_.empty()) {
        const Span& segment = floodTracker_.back();
        floodTracker_.pop_back();
        int dy = segment.dir, y = segment.y + segment.dir, x1 = segment.lx, x2 = segment.rx;

        if (!utils::lix< size_t >(x1, 0, outer_.width_) || !utils::lix< size_t >(x2, 0, outer_.width_) || !utils::lix< size_t >(y, 0, outer_.height_)) continue;

        if (outer_.unmergedLabels_[y * outer_.width_ + x1] == label || outer_.unmergedLabels_[y * outer_.width_ + x2] == label) {
            // This segment has already been claimed
            // mark(y, x1, x2, label);
            continue;
        }

        Vec3&     center_point = (*outer_.m_pointCloud)[y * outer_.width_ + ((x1 + x2) / 2)];
        const int ctWidth      = outer_.ctWidthFunc(center_point.squaredNorm());

        // ============================================LEFT====================================================
        // skip and check left (fast)
        for (x = x1, lastX = x1; x > 0; x -= ctWidth) {
            if (!validateInColDir(lastX, x, y)) break;
            if (!validateInRowDir(y, x, ctWidth)) break;
            if (outer_.unmergedLabels_[y * outer_.width_ + x] != BACKGROUND) break;
            lastX = x;
        }
        // check every pixel left (slower)
        for (x = lastX; x > 0; x -= 1) {
            if (!validateInColDir(lastX, x, y)) break;
            if (!validateInRowDir(y, x, ctWidth)) break;
            if (outer_.unmergedLabels_[y * outer_.width_ + x] != BACKGROUND) break;
            lastX = x;
        }
        l_most = lastX;

        if ((x1 - l_most) >= ctWidth) mark(y, l_most, x1, label);
        // Flood Leak on left
        if (x < x1 && (x1 - x) > ctWidth) {
            floodTracker_.emplace_back(y, l_most, x1, -dy);  // opposite row direction
        }

        // ============================================MIDDLE====================================================
        int tmp_l = l_most;
        x         = x1;
        lastX     = x1;
        while (x < x2) {
            for (; x < x2; x += ctWidth) {
                if (!validateInColDir(lastX, x, y)) break;
                if (!validateInRowDir(y, x, ctWidth)) break;
                if (outer_.unmergedLabels_[y * outer_.width_ + x] != BACKGROUND) break;
                lastX = x;
            }
            for (x = lastX; x < x2; x += 1) {
                if (!validateInColDir(lastX, x, y)) break;
                if (!validateInRowDir(y, x, ctWidth)) break;
                if (outer_.unmergedLabels_[y * outer_.width_ + x] != BACKGROUND) break;
                lastX = x;
            }
            x = lastX;

            if (lastX != x2 - 1)  // x2 not reached
            {
                if ((lastX - tmp_l) >= ctWidth) {
                    mark(y, tmp_l, lastX, label);
                    floodTracker_.emplace_back(y, tmp_l, lastX, dy);  // same row direction
                }
                tmp_l = x2;  // tmp_l don't change if x2 reached, where the span will be pushed in RIGHT part
            }

            // x++ to check next pixel
            for (x++; x < x2; x++) {
                const size_t current = y * outer_.width_ + x;
                if (outer_.unmergedLabels_[current] == BACKGROUND) {
                    tmp_l = x;
                    lastX = x;
                    break;
                }
            }
        }
        /////////////////////////////////////////////////       NOT  SURE HERE ////////////////////////////////////////////////////////
        // if (r_most - l_most > ctWidth) floodTracker_.emplace_back(y, l_most, r_most, dy); // same row direction

        // ============================================RIGHT====================================================
        // skip and check right (fast)
        for (x = x2, lastX = x2; x < int(outer_.width_); x += ctWidth) {
            if (!validateInColDir(lastX, x, y)) break;
            if (!validateInRowDir(y, x, ctWidth)) break;
            if (outer_.unmergedLabels_[y * outer_.width_ + x] != BACKGROUND) break;
            lastX = x;
        }
        // check every pixel right (slower)
        for (x = lastX; x < int(outer_.width_); x += 1) {
            if (!validateInColDir(lastX, x, y)) break;
            if (!validateInRowDir(y, x, ctWidth)) break;
            if (outer_.unmergedLabels_[y * outer_.width_ + x] != BACKGROUND) break;
            lastX = x;
        }
        r_most = lastX;

        if ((r_most - x2) >= ctWidth) mark(y, x2, r_most, label);
        // Flood Leak on right
        if (x > x2 && (x - x2) > ctWidth) {
            floodTracker_.emplace_back(y, x2 + 1, r_most, -dy);  // opposite row direction
        }
        if ((r_most - tmp_l) > ctWidth) floodTracker_.emplace_back(y, tmp_l, r_most, dy);  // same row direction
    }

    return floodCounter_;
}
