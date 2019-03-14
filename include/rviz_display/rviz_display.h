///  @file rviz_display.h
/// \class RvizDisplay
/// \brief A simple marker display interface for rviz
///
/// This class can display ros shape messages in rviz, including meshes and solid primitives
///
///
/// \author Philip Long <philip.long01@gmail.com>, RiVER Lab Northeastern University
/// \date Mar, 2019

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/surface/concave_hull.h>
#include <geometric_shapes/shape_to_marker.h>

#ifndef RVIZ_DISPLAY_HPP
#define RVIZ_DISPLAY_HPP

class RvizDisplay
{
private:
    ros::NodeHandle nh_;
    /// Joint state Subscriber
    ros::Publisher mkr_pub;

    /// If true, polytopes are not over-written
    bool hold_on_;
    /// Increments polytope marker id if hold is on
    int counter_;
public:
    RvizDisplay ( ros::NodeHandle nh );
    /// hold_on_ setter function
    void setHoldFlag ( bool val );

    /** Compute a polytopes volume and/or plot it
     *   The vertices are shifted to the offset position (offset_position), for instance the robot end effector
     *   PCL is used to obtain the convex hull
     *   A mesh message is created from the convex hull and published to RVIZ
     *   The facet color is defined by color_line, while points by color_pts
     *   The return value is the volume of the polytope
     */
    double plotPolytope ( Eigen::MatrixXd  vertices,
                          Eigen::Vector3d offset_position,
                          std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
                          std::vector<double> color_line= {1.0,0.4,0.4,1.0},
                          bool plot=false
                        );

    /** Compute a polytopes volume and/or plot it
    *   The vertices are shifted to the offset position (offset_position), for instance the robot end effector
    *   std::string frame, to define the frame of reference.
    *   PCL is used to obtain the convex hull
    *   A mesh message is created from the convex hull and published to RVIZ
    *   The facet color is defined by color_line, while points by color_pts
    *   The return value is the volume of the polytope.
    */
    double plotPolytope ( Eigen::MatrixXd  vertices,
                          Eigen::Vector3d offset_position,
                          std::string frame,
                          std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
                          std::vector<double> color_line= {1.0,0.4,0.4,1.0},
                          bool plot=false );

    /** Plot a Polytope defined a a set of vertices
    *   The vertices are shifted to the offset position (offset_position), for instance the robot end effector
    *   PCL is used to obtain the convex hull
    *   A mesh message is created from the convex hull and published to RVIZ
    *   The facet color is defined by color_line, while points by color_pts
    *   The return value is the volume of the polytope
    */
    bool plotPolytope ( std::string polytope_name,
                        Eigen::MatrixXd  vertices,
                        Eigen::Vector3d offset_position,
                        std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
                        std::vector<double> color_line= {1.0,0.4,0.4,1.0}
                      );

    /** Plot a Polytope defined a a set of vertices
    *   The vertices are shifted to the offset position (offset_position), for instance the robot end effector
    *   std::string frame, to define the frame of reference.
    *   PCL is used to obtain the convex hull
    *   A mesh message is created from the convex hull and published to RVIZ
    *   The facet color is defined by color_line, while points by color_pts
    *   The return value is the volume of the polytope
    */

    bool plotPolytope ( std::string polytope_name,
                        Eigen::MatrixXd  vertices,
                        std::string frame,
                        Eigen::Vector3d offset_position,
                        std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
                        std::vector<double> color_line= {1.0,0.4,0.4,1.0}
                      );



    /// Display a marker defined by a solid shape primitive
    bool displayMarker ( shape_msgs::SolidPrimitive s1,
                         const Eigen::Affine3d & T,
                         unsigned int obj_id,
                         const Eigen::Vector4d & color );

    /// Display a marker defined by a mesh primitive
    bool displayMarker ( shape_msgs::Mesh s1,
                         const Eigen::Affine3d & T,
                         unsigned int obj_id,
                         const Eigen::Vector4d & color );

    /// Display a marker, in a given reference frame defined by a solid shape primitive
    bool displayMarker ( shape_msgs::SolidPrimitive s1,
                         const Eigen::Affine3d & T,
                         std::string frame,
                         unsigned int obj_id,
                         const Eigen::Vector4d & color );

    /// Display a marker, in a given reference frame defined by a mesh primitive
    bool displayMarker ( shape_msgs::Mesh s1,
                         const Eigen::Affine3d & T,
                         std::string frame,
                         unsigned int obj_id,
                         const Eigen::Vector4d & color );

    /// Pubish a point in RVIZ
    void publishPoint ( geometry_msgs::Pose pose,
                        std::string mkr_namespace="object",
                        unsigned int id=1,
                        std::string frame="world",
                        std::vector<double> color= {1.0,0.4,0.4,1.0},
                        std::vector<double> scale= {0.05,0.05,0.05} );
    /// Pubish a point in RVIZ
    void publishPoint ( geometry_msgs::Point pose,
                        std::string mkr_namespace="object",
                        unsigned int id=1,
                        std::string frame="world",
                        std::vector<double> color= {1.0,0.4,0.4,1.0},
                        std::vector<double> scale= {0.05,0.05,0.05} );
    /// Pubish a point in RVIZ
    void publishPoint ( Eigen::Vector3d pose,
                        std::string mkr_namespace="object",
                        unsigned int id=1,
                        std::string frame="world",
                        std::vector<double> color= {1.0,0.4,0.4,1.0},
                        std::vector<double> scale= {0.05,0.05,0.05} );
    /// Pubish a plane in RVIZ
    void publishPlane ( geometry_msgs::Pose pose,
                        std::string mkr_namespace,
                        unsigned int id,
                        std::string frame,
                        std::vector<double> color,
                        std::vector<double> scale
                      );

    ~RvizDisplay() {};
};
#endif
