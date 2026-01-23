#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_util/line_segment.h>

#include "math_utils/min_enclosing_ball.h"

#include "free_regions_graph/free_regions_graph.h"
#include "gap_extractor/gap_extractor.h"

#include <chrono>

#include <planner_manager/GapCandidates.h>
#include <planner_manager/GapCandidate.h>

struct Gaps{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d dir_scan_frame;
    Eigen::Vector3d dir_odom_frame;

    // scan frame parameters
    float center_yaw;
    float center_elev;
    float range_mean;

    // yaw and elev bias for limited gap
    float yaw_bias = 0.0f;
    float elev_bias = 0.0f;

    float yaw_span;
    float elev_span;
    int   size;

    int   type = -1;           // 0=open, 1=limited, 2=free, 3=goal
};

class PlannerManager {
    private:
    ros::NodeHandle node_;
    bool env_type_;
    Eigen::Vector3d size_of_cropped_pointcloud_;

    // gap candidates
    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> gap_candidates_open_;
    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> gap_candidates_limited_;
    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> gap_candidates_free_;

    public:
    PlannerManager() {}
    ~PlannerManager() {}
    void setEnvType(int env_type) { env_type_ = env_type; }
    void setSizeOfCroppedPointcloud(const Eigen::Vector3d &size) { size_of_cropped_pointcloud_ = size; }

    typedef std::unique_ptr<PlannerManager> Ptr;

    std::vector<Eigen::Vector3d> graph_points_for_visualization_;

    /* ROS Subscriber */
    ros::Subscriber velodyne_sub_;   // pointcloud for 3D environment
    ros::Subscriber scan2d_sub_;     // laser scan for 2D environment
    ros::Subscriber candidate_gaps_sub_;

    /* ROS Publisher */
    ros::Publisher poly_pub_aniso_;
    ros::Publisher poly_pub_;
    ros::Publisher robot_points_pub_;
    ros::Publisher robot_sphere_pub_;

    ros::Publisher poly_pub_aniso_full_;
    ros::Publisher poly_frtree_pub_;

    ros::Publisher test_cube_pub_;

    void initPlannerModule(ros::NodeHandle &nh);

    /* Callback Functions */
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void odomTimerCallback(const ros::TimerEvent &event);
    void candidateGapsCallback(const planner_manager::GapCandidates::ConstPtr &msg);

    void debugTimerCallback(const ros::TimerEvent &event);
    void publishRobotPoints();
    void publishRobotSphere();

    void publishTestCube();

    ros::Timer odom_timer_;
    ros::Timer debug_timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_odom_;
    tf2_ros::Buffer tf_buffer_odom_;

    geometry_msgs::TransformStamped::Ptr base_scan_ptr_, odom_base_ptr_;
    Eigen::Matrix4f T_base_scan_mat_ = Eigen::Matrix4f::Identity();

    vec_Vec3f pointcloud_cropped_odom_frame_;
    vec_Vec2f pointcloud_cropped_odom_frame_2d_;

    vec_Vec3f robot_shape_points_;
    Ellipsoid3D robot_ellipsoid_;

    vec_Vec2f robot_shape_points_2d_;
    Ellipsoid2D robot_ellipsoid_2d_;

    vec_E<Polyhedron2D> polys_aniso_2d_;
    vec_E<Polyhedron2D> polys_2d_;
    vec_E<Polyhedron3D> polys_aniso_3d_;
    vec_E<Polyhedron3D> polys_3d_;
    
    vec_E<Polyhedron2D> polys_aniso_full_2d_;
    vec_E<Polyhedron2D> polys_FRTree_2d_;
    vec_E<Polyhedron3D> polys_FRTree_3d_;

    FreeRegionsGraph::Ptr free_regions_graph_ptr_;
    GraphNode *current_node_;

    GapExtractor::Ptr gap_extractor_ptr_;

    int num_of_yaw_samples_ = 0;

    // plan trajectory function
    void planTrajectory(Eigen::Vector3d &start_pos, Eigen::Vector3d &goal_pos, GraphNode* current_node);
    void sortAllCandidatesGap(Eigen:: Vector3d &start_pos,
                              std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates);
    void reorderCandidatesGapWithGoal(Eigen:: Vector3d &goal_pos, 
                                     std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates);

    void decomposeAlongGapDirections(Eigen::Vector3d &start_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates);

    void decomposeAlongGapDirectionsTEST(Eigen::Vector3d &start_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates);
    void decomposeAlongGapDirections_FRTreeTEST(Eigen::Vector3d &start_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates);
    
    bool getTrajectory(Eigen::Vector3d &start_pos, GraphNode* current_node);
    double supportValueVertices(Eigen::Vector3d &norm, vec_Vec3f &vertices, const Eigen::Matrix3d& R);
    double supportValueVertices(Eigen::Vector2d &norm, vec_Vec2f &vertices, const Eigen::Matrix2d& R);

    double solveLPByEnumeratingVertices2D(const Eigen::MatrixXd &A, const Eigen::VectorXd &bprime, const Eigen::Vector2d &dir, Eigen::Vector2d &best_vertex);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TESTING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    bool getTrajectoryTemp(Eigen::Vector3d &start_pos, GraphNode* current_node);
    std::vector<Eigen::Vector3d> trajectory_points_temp_;
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TESTING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // PLANNER_MANAGER_H