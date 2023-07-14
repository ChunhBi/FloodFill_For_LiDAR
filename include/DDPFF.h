#ifndef DDPFF_H
#define DDPFF_H

#include <functional>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include "constants.h"
#include "utilities.h"
#include "Config.h"

#define DEBUG_DDPFF 0

struct PlanePointNormal 
{
    uint id;
    size_t count;
    Vec3 p, n;
    std::set< size_t > inliers;

    PlanePointNormal() {}
    inline void clear() 
    {
        id    = 0;
        count = 0;
        p.fill(0);
        n.fill(0);
        inliers.clear();
    }
};


/* The parameters of the segmented planes. */
struct PlaneSegParams {

    size_t planeId = 0;
    size_t count = 0, seedIndex = 0;
    double     maxDepth = 0;
    Vec2i      pixel;          // The center pixel of the plane.
    Vec2i      accumulator2D;  // Keep a running sum of pixels in the plane.
    Vec3       point, normal;  // The plane is represented in point-normal form using its centroid.
    Vec3       accumulator3D;  // Keep a running sum of points in the plane.
    // QColor color; // The color assigned to the plane.
    std::unordered_set< size_t > neighbours;    // A list of immediate neighbours of this plane.
    std::unordered_set< size_t > disconnected;  // List of erstwhile neighbours.
    std::vector< size_t >        inliers;       // List of plane inlier indices.
    std::unordered_set< size_t > mergeWith;     // A list of planes which are merged with this one.

    /* Clear the segmentation parameters. */
    inline void clear() {
        count     = 0;
        planeId   = 0;
        seedIndex = 0;
        maxDepth  = 0;
        point.fill(0);
        normal.fill(0);
        accumulator3D.fill(0);
        mergeWith.clear();
        neighbours.clear();
        inliers.resize(0);
        inliers.shrink_to_fit();
    }
};

/*
 * Depth dependent planar flood fill.
 */
class DDPFF
{
private:
    CloudBuffer* m_pointCloud;
    Config* m_config;

public:
    DDPFF(Config& config);
    void setBuffer(CloudBuffer* pointCloud) { this->m_pointCloud = pointCloud; }
    void init();
    void clear();
    void compute();
    std::vector< size_t > getLabels();
    std::vector< PlanePointNormal > getPlanes();

private:
    /* Seed represented as combined point and normal. */
    struct Seed {
        Vec3 point, normal;
    };

    /*
     * Filled horizontal segment of scanline y for xl<=x<=xr.
     * Parent segment was on line y-dy.  dy=1 or -1
     */
    struct Span {
        size_t y, lx, rx;  // y denotes the row number of the root segment
        short  dir;

        Span(){};
        Span(size_t _y, size_t _xl, size_t _xr, short _dy) : y(_y), lx(_xl), rx(_xr), dir(_dy){};
    };

    using labels_t             = std::vector< size_t >;
    using floodTracker_t       = std::vector< Span >;
    using seedDistValidator_t  = std::function< bool(const Vec3&, double&) >;
    using pointDistValidator_t = std::function< bool(const Vec3&, const Vec3&, const size_t&, double&) >;
    using planeSegParamsPtr_t  = std::shared_ptr< PlaneSegParams >;

#if DEBUG_DDPFF == 1
    floodTracker_t unmarkedSegments_;
#endif

    /* Flood an organized point cloud using the SEEDFILL algorithm. */
    struct FloodFill {

        /* Reference to outer class. */
        DDPFF& outer_;

        /* The function used to adaptively validate the point-plane distance from the seed. */
        seedDistValidator_t sdf_;

        /* The function used to adaptively validate the distance between two points. */
        const pointDistValidator_t pdf_;

        /* A stack based flood tracker. */
        floodTracker_t floodTracker_;

        /* Keep flood statistics such as number of points marked and the number of comparisons performed. */
        size_t floodCounter_, nOps_;

        /* Set the point distance function. */
        pointDistValidator_t setPointDistFunc();

        /* Set the seed (point-plane) distance function. */
        void setSeedDistFunc(const Seed& seed);

        /* Validate if the point at the given index belongs to the current flood. */
        bool validateIndex(const size_t& index, const size_t& prvIndex, const size_t& diff);

        /*
         * Validate a seed fill segment in terms of the cut-through width.
         * Specifically, check if the segment can be grown in the y-direction.
         */
        bool validateInRowDir(const int& y, const int& x, const size_t& ctWidth);

        /* Check if the given index can be marked with the given label. */
        bool validateInColDir(const int& prevx, int& x, const int& y);

        bool validateInColDir_new(const int& y, const int& x, const size_t& ctWidth);

        /* Mark the seed fill segment with the given label.This indicates that the given segment is covered by the current flood. */
        void mark(const size_t y, const size_t lx, const size_t rx, const size_t& label);

        /* Constructor. */
        FloodFill(DDPFF& outer) : outer_(outer), pdf_(setPointDistFunc()), floodCounter_(0), nOps_(0){};

        /* Start flood at the given index with the given label. */
        size_t flood(const size_t& startRow, const size_t& startCol, const size_t& label, const Seed& seed);
    };

    /* A count of the number of planes found so far. Also used for assigning unique plane indices. */
    size_t totalPlaneCount_;

    /* The dowsampling factor, height and width of the depth image. */
    size_t sampleFactor_, height_, width_;

    /* buffer size = height x width. */
    size_t bufferSize_;

    /*
     * The planar labels. Due to the nature of flood fill,
     * the segments necessarily are connected.
     */
    labels_t unmergedLabels_, mergedLabels_;

    /* List of pointers to the final set of merged planes. */
    std::vector< planeSegParamsPtr_t > mergedPlanes_;

    /* Plane stats before merging. */
    std::vector< PlaneSegParams > unmergedPlanes_;

    /* List of threshold functions. */
    double planeDistThreshold_flood(const double& x);
    double planeDistThreshold_merge(const double& x);
    double pointDistThreshold(const double& x);
    double angleDistanceThreshold(const double& x);
    size_t ctWidthFunc(const double& x);
    size_t rangeFunc(const double& x);

    /* Get a plane hypothesis at the given pixel indicated by row and col. */
    bool getSeed(const size_t& row, const size_t& col, Seed& seed, size_t& startRadius);

    /* Compute the weighted average centroid and normal of two planes. */
    void planeAverage(const planeSegParamsPtr_t& plane1, const planeSegParamsPtr_t& plane2, const planeSegParamsPtr_t& target);

    /* Merge plane2 into plane1. */
    void transferParameters(const planeSegParamsPtr_t& target, const planeSegParamsPtr_t& source);

    /* Mutual plane distance between plane segments. */
    double mutualDistance(const PlaneSegParams& plane1, const PlaneSegParams& plane2);

    /* Angle distance between plane segments. */
    double angleDistance(const PlaneSegParams& plane1, const PlaneSegParams& plane2);

    /* Check if source plane can be merged into target plane (mutual plane distance). */
    bool isMergeable_mutual(const PlaneSegParams& plane1, const PlaneSegParams& plane2);

    /* Check if source plane can be merged into target plane (angle difference between normals). */
    bool isMergeable_angle(const PlaneSegParams& plane1, const PlaneSegParams& plane2);

    /* Check if source plane can be merged into target plane. */
    bool isMergeable(const PlaneSegParams& plane1, const PlaneSegParams& plane2);

    /* Merge planes using bfs. */
    void merge(bool unidir = true);

    /*
     * Re order the merged planes by size and renumber them.
     * Also build the segmentation image.
     */
    void reorder();

    /* Fill gaps in the initial segmentation using flood fill. */
    void plugGaps();

    /* Find plane inliers from the segmentation. */
    void findInliers();

    /* Find plane neighbours from the segmentation. */
    void findPointNeighbours();

    /* Compute the plane parameters from the segmentation. */
    void computePlaneParams();

    /* Remove plane indices from neighbour lists in cases where they cannot be merged. */
    void disconnectUnmergeableNeighbours();
};

#endif  // DDPFF_H
