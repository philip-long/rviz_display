#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iterator>
#include <math.h>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <boost/thread/mutex.hpp>
#include <pcl/surface/concave_hull.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <geometric_shapes/shape_to_marker.h>

#ifndef RVIZ_DISPLAY_HPP
#define RVIZ_DISPLAY_HPP

class RvizDisplay
{
private:
    ros::NodeHandle nh_;
    /// Joint state Subscriber
    ros::Publisher mkr_pub; //  rviz visualization
    int counter_;
    bool hold_on_;
public:
    RvizDisplay ( ros::NodeHandle nh);
    void setHoldFlag(bool val);
    // Direct dynamic model, for a given state and joint torque calculate the acceleration
    double plotPolytope ( Eigen::MatrixXd  vertices,
                          Eigen::Vector3d offset_position,
                          std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
                          std::vector<double> color_line= {1.0,0.4,0.4,1.0},
                          bool plot=false
                        );
    bool plotPolytope (  std::string polytope_name,
		      Eigen::MatrixXd  vertices,
		      Eigen::Vector3d offset_position,
		      std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
		      std::vector<double> color_line= {1.0,0.4,0.4,1.0}
		    );

    void publishPoint ( geometry_msgs::Pose pose,
                        std::string mkr_namespace="object",
                        unsigned int id=1,
                        std::string frame="world",
                        std::vector<double> color= {1.0,0.4,0.4,1.0},
                        std::vector<double> scale= {0.05,0.05,0.05} );
    bool displayMarker ( shape_msgs::SolidPrimitive s1,
                         const Eigen::Affine3d & T,
                         unsigned int obj_id,
                         const Eigen::Vector4d & color );

    void publishPoint ( geometry_msgs::Point pose,
                        std::string mkr_namespace="object",
                        unsigned int id=1,
                        std::string frame="world",
                        std::vector<double> color= {1.0,0.4,0.4,1.0},
                        std::vector<double> scale= {0.05,0.05,0.05} );

    void publishPoint ( Eigen::Vector3d pose,
                        std::string mkr_namespace="object",
                        unsigned int id=1,
                        std::string frame="world",
                        std::vector<double> color= {1.0,0.4,0.4,1.0},
                        std::vector<double> scale= {0.05,0.05,0.05} );
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
