#pragma once
#include <string>
#include <vector>
#include <iostream>
#define FLAGS_planning_upper_speed_limit 12.5
#define FLAGS_default_reference_line_width 3.75
// #define FLAGS_trajectory_time_length 10
#define FLAGS_trajectory_time_resolution 0.1
#define FLAFS_longitudinal_acceleration_lower_bound -6
#define FLAGS_longitudinal_acceleration_upper_bound 4
#define FLAGS_trajectory_time_length 8
#define FLAGS_polyomial_minimal_param 0.01
#define FLAGS_num_velocity_sample 6
#define FLAGS_min_velocity_sample_gap 1
#define FLAGS_numerical_epsilon 1e-6
#define FLAGS_time_min_density 1.0
#define FLAGS_default_lon_buffer 5.0
#define FLAGS_num_sample_follow_per_timestamp 3
#define FLAGS_polynomial_minimal_param 0.01

using namespace std;

struct PathPoint
{
    /* data */
    double x;
    double y;
    double z;

    double theta;
    double kappa;
    double s;

    double dkappa;
    double ddkappa;
    int lane_id;

    double x_derivative;
    double y_derivative;
};



struct ReferencePoint
{
    double rx;
    double ry;
    double rtheta;

    double rkappa;
    double rdkappa;

};

struct SpeedLimit
{
    double start_s = 0;
    double end_s = 0;
    double speed_limit = 10;
};



struct Vec2d
{
    double x;
    double y;
    // double length;
    // double width;
    // double heading;
};


struct SL_boundary
{
    /* data */
    double start_s;
    double end_s;
    double start_l;
    double end_l;
};

struct STPoint
{
    /* data */
    double t;
    double s;
};
struct StopPoint
{
    double s;
    enum Type
    {
        HARD = 0,
        SOFT = 1
    };
    StopPoint::Type StopPoint_type = HARD;
};

struct PlanningTarget
{
    StopPoint Stop_Point;
    double cruse_speed;
};

struct STBoundary
{
    std::string id;
    std::pair<double,double> bottom_left_point;   //[time, s]
    std::pair<double,double> bottom_right_point;
    std::pair<double,double> upper_left_point;
    std::pair<double,double> upper_right_point;
};

struct TrajectoryPoint
{
    PathPoint path_point;
    double v;
    double a;
    double da;
    double steer;
    double confidence;
    double relative_time;
};


struct Trajectory
{
    double probablity;
    std::vector<TrajectoryPoint> trajectory_points;
};

struct BBox2D
{
    double xmin;
    double ymin;
    double xmax;
    double ymax;
};

struct SensorMeasurement
{
    /* data */
    std::string sensor_id;
    std::string id;
};

struct LightStatus
{
    uint8_t brake_visible;
    uint8_t brake_switch_on;
    uint8_t left_turn_visible;
    uint8_t left_turn_switch_on;
    uint8_t right_turn_visible;
    uint8_t right_turn_switch_on;
};

struct PerceptionObstacles
{
    std::string id;
    bool isVirtual;
    std::pair<double,double> position;
    double theta;
    std::pair<double, double> velocity;
    double length;
    double width;
    double height;

    std::vector<Vec2d> corner_points;
    double tracking_time;
    enum Type
    {
        UNKONWN = 0,
        UNKNOWN_MOVABLE = 1,
        UNKNOWN_UNMOVABLE = 2,
        PEDESTRIAN = 3,
        BICYCLE = 4,
        VEHICLE = 5,
    };
    Type obstacle_type = VEHICLE;
    double timestamp;
    double confidence;
    enum ConfidenceType
    {
        CONFIDENCE_UNKNOWN = 0,
        CONFIDENCE_CNN = 1,
        CONFIDENCE_RADAR = 2,
    };
    ConfidenceType confidence_type;
    std::pair<double, double> acceleration;
    std::pair<double, double> anchor_point;

    BBox2D bbox;
    enum SubType
    {
        ST_UNKNOWN = 0,
        ST_UNKNOWN_MOVABLE = 1,
        ST_UNKNOWN_UNMOVABLE = 2,
        ST_CAR = 3,
        ST_VAN = 4,
        ST_TRUCK = 5,
        ST_BUS = 6,
        ST_CYCLIST = 7,
        ST_MOTORCYCLIST = 8,
        ST_TRICYCLIST = 9,
        ST_PEDESTRIAN = 10,
        ST_TRAFFICCONE = 11,
        ST_SMALLMOT = 12,
        ST_BIGMOT = 13,
        ST_NOMOT = 14,
    };
    SubType sub_type;
    SensorMeasurement measurement;
    double height_above_ground;
    LightStatus light_status;

    std::vector<Trajectory> msg;
    enum Source
    {
        HOST_VEHICLE = 0,
        V2X = 2,
    };
    Source source;
    enum SemanticType
    {
        SM_UNKNOWN = 0,
        SM_IGNORE = 1,
        SM_GROUND = 2,
        SM_OBJECT = 3,
        SM_CURB = 4,
        SM_VEGETATION = 5,
        SM_FENCE = 6,
        SM_NOISE = 7,
        SM_WALL = 8,
        SM_MAX_OBJECT_SEMANTIC_LABEL = 9,
    };

    SemanticType semantic_type;

};

struct SamplePoint
{
    STPoint path_time_point;
    double ref_v;
};


struct VehicleConfig
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool is_valid = false;
    double Front_edge_to_center;
    double Back_edge_to_center;
};




template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1, const double t)
{
    if(std::abs(t0 - t1) < 1.0e-6)
    {
        std::cout << "input time difference is too small" << std::endl;
        return x0;
    }
    const double weight = (t - t0)/(t1 - t0);
    const T x = x0 + weight*(x1 - x0);
    return x;
}

