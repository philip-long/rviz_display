/*** INCLUDE FILES ***/
#include <rviz_display/rviz_display.h>


RvizDisplay::RvizDisplay ( ros::NodeHandle nh ) : nh_ ( nh ) {
    ROS_INFO ( "RVIZ Initialized" );

    mkr_pub=nh_.advertise<visualization_msgs::Marker>
            ( "/visualization_marker",1 );

    counter_=1;
    hold_on_=false;
}
void RvizDisplay::setHoldFlag ( bool val ) {
    hold_on_=val;
}

bool RvizDisplay::displayMarker ( shape_msgs::SolidPrimitive s1,
                                  const Eigen::Affine3d & T,
                                  unsigned int obj_id,
                                  const Eigen::Vector4d & color ) {
    displayMarker ( s1,T,"world",obj_id,color );
}


bool RvizDisplay::displayMarker ( shape_msgs::Mesh s1,
                                  const Eigen::Affine3d & T,
                                  unsigned int obj_id,
                                  const Eigen::Vector4d & color ) {

    displayMarker ( s1,T,"world",obj_id,color );
}


bool RvizDisplay::displayMarker ( shape_msgs::SolidPrimitive s1,
                                  const Eigen::Affine3d & T,
                                  std::string frame,
                                  unsigned int obj_id,
                                  const Eigen::Vector4d & color ) {
    visualization_msgs::Marker mkr;

    geometric_shapes::constructMarkerFromShape ( s1,mkr );
    while ( mkr_pub.getNumSubscribers() <1 ) {
        ROS_INFO_ONCE ( "Waiting until marker is displayed in RVIZ" );
        ros::spinOnce();
        ros::Duration ( 0.05 ).sleep();
    }
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.header.frame_id=frame;
    mkr.ns="Objects";
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.id=obj_id;
    mkr.color.r=color ( 0 );
    mkr.color.g=color ( 1 );
    mkr.color.b=color ( 2 );
    mkr.color.a=color ( 3 );
    Eigen::Quaterniond q ( T.linear() );
    mkr.pose.position.x=T ( 0,3 );
    mkr.pose.position.y=T ( 1,3 );
    mkr.pose.position.z=T ( 2,3 );
    mkr.pose.orientation.w=q.w();
    mkr.pose.orientation.x=q.x();
    mkr.pose.orientation.y=q.y();
    mkr.pose.orientation.z=q.z();
    mkr_pub.publish ( mkr );
    ros::spinOnce();
}


bool RvizDisplay::displayMarker ( shape_msgs::Mesh s1,
                                  const Eigen::Affine3d & T,
                                  std::string frame,
                                  unsigned int obj_id,
                                  const Eigen::Vector4d & color ) {
    visualization_msgs::Marker mkr;
    geometric_shapes::constructMarkerFromShape ( s1,mkr );

    while ( mkr_pub.getNumSubscribers() <1 ) {
        ROS_INFO_ONCE ( "Waiting until marker is displayed in RVIZ" );
        ros::spinOnce();
        ros::Duration ( 0.05 ).sleep();
    }
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.header.frame_id=frame;
    mkr.ns="Objects";
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.id=obj_id;
    mkr.color.r=color ( 0 );
    mkr.color.g=color ( 1 );
    mkr.color.b=color ( 2 );
    mkr.color.a=color ( 3 );
    Eigen::Quaterniond q ( T.linear() );
    mkr.pose.position.x=T ( 0,3 );
    mkr.pose.position.y=T ( 1,3 );
    mkr.pose.position.z=T ( 2,3 );
    mkr.pose.orientation.w=q.w();
    mkr.pose.orientation.x=q.x();
    mkr.pose.orientation.y=q.y();
    mkr.pose.orientation.z=q.z();
    mkr_pub.publish ( mkr );
    ros::spinOnce();
}




void RvizDisplay::publishPlane ( geometry_msgs::Pose pose,
                                 std::string mkr_namespace,
                                 unsigned int id,
                                 std::string frame,
                                 std::vector<double> color,
                                 std::vector<double> scale
                               ) {
    visualization_msgs::Marker mkr;
    mkr.ns=mkr_namespace;
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.type=visualization_msgs::Marker::CUBE;
    mkr.id=id;
    mkr.pose=pose;
    mkr.header.frame_id=frame;
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.color.r=color[0];
    mkr.color.g=color[1];
    mkr.color.b=color[2];
    mkr.color.a=0.5;
    mkr.scale.x=scale[0];
    mkr.scale.y=scale[1];
    mkr.scale.z=scale[2];
    while ( mkr_pub.getNumSubscribers() <1 ) {
        ROS_INFO ( "Waiting for Marker Subs" );
        ros::spinOnce();
    }
    mkr_pub.publish
    ( mkr );

}


void RvizDisplay::publishPoint ( geometry_msgs::Pose pose,
                                 std::string mkr_namespace,
                                 unsigned int id,
                                 std::string frame,
                                 std::vector<double> color,
                                 std::vector<double> scale
                               ) {
    visualization_msgs::Marker mkr;
    mkr.ns=mkr_namespace;
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.type=visualization_msgs::Marker::SPHERE;
    mkr.id=id;
    mkr.pose=pose;
    mkr.header.frame_id=frame;
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.color.r=color[0];
    mkr.color.g=color[1];
    mkr.color.b=color[2];
    mkr.color.a=0.8;
    mkr.scale.x=scale[0];
    mkr.scale.y=scale[1];
    mkr.scale.z=scale[2];
    while ( mkr_pub.getNumSubscribers() <1 ) {
        ROS_INFO_ONCE ( "Waiting until marker is displayed in RVIZ" );
        ros::spinOnce();
    }
    mkr_pub.publish ( mkr );

}


void RvizDisplay::publishPoint ( geometry_msgs::Point pose,
                                 std::string mkr_namespace,
                                 unsigned int id,
                                 std::string frame,
                                 std::vector<double> color,
                                 std::vector<double> scale
                               ) {
    visualization_msgs::Marker mkr;
    mkr.ns=mkr_namespace;
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.type=visualization_msgs::Marker::SPHERE;
    mkr.id=id;
    mkr.pose.position=pose;
    mkr.pose.orientation.x=0.0;
    mkr.pose.orientation.y=0.0;
    mkr.pose.orientation.z=0.0;
    mkr.pose.orientation.w=0.0;
    mkr.header.frame_id=frame;
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.color.r=color[0];
    mkr.color.g=color[1];
    mkr.color.b=color[2];
    mkr.color.a=0.8;
    mkr.scale.x=scale[0];
    mkr.scale.y=scale[1];
    mkr.scale.z=scale[2];
    while ( mkr_pub.getNumSubscribers() <1 && ros::ok() ) {
        ROS_INFO ( "Waiting for subs" );
        ros::spinOnce();
    }
    mkr_pub.publish ( mkr );

}


void RvizDisplay::publishPoint ( Eigen::Vector3d pose,
                                 std::string mkr_namespace,
                                 unsigned int id,
                                 std::string frame,
                                 std::vector<double> color,
                                 std::vector<double> scale
                               ) {
    visualization_msgs::Marker mkr;
    mkr.ns=mkr_namespace;
    mkr.action=visualization_msgs::Marker::ADD;
    mkr.type=visualization_msgs::Marker::SPHERE;
    mkr.id=id;
    mkr.pose.position.x=pose ( 0 );
    mkr.pose.position.y=pose ( 1 );
    mkr.pose.position.z=pose ( 2 );
    mkr.pose.orientation.x=0.0;
    mkr.pose.orientation.y=0.0;
    mkr.pose.orientation.z=0.0;
    mkr.pose.orientation.w=0.0;
    mkr.header.frame_id=frame;
    mkr.lifetime=ros::Duration ( 0.0 );
    mkr.color.r=color[0];
    mkr.color.g=color[1];
    mkr.color.b=color[2];
    mkr.color.a=0.8;
    mkr.scale.x=scale[0];
    mkr.scale.y=scale[1];
    mkr.scale.z=scale[2];
    while ( mkr_pub.getNumSubscribers() <1 ) {
        ROS_INFO ( "Waiting for subs" );
        ros::spinOnce();
    }
    mkr_pub.publish ( mkr );

}




bool RvizDisplay::plotPolytope ( std::string polytope_name,
                                 Eigen::MatrixXd  vertices,
                                 Eigen::Vector3d position,
                                 std::vector<double>  color_pts,
                                 std::vector<double>  color_line ) {


    return plotPolytope ( polytope_name,
                          vertices,
                          "world",
                          position
                          ,color_pts,color_line );
}


bool RvizDisplay::plotPolytope ( std::string polytope_name,
                                 Eigen::MatrixXd  vertices,
                                 std::string frame,
                                 Eigen::Vector3d position,
                                 std::vector<double>  color_pts,
                                 std::vector<double>  color_line ) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected ( new pcl::PointCloud<pcl::PointXYZ> );
    double vol ( 0.0 );
    // std::cout<<"Plotted Polytope Points "<<std::endl;
    for ( int var = 0; var < vertices.rows(); ++var ) {
        pcl::PointXYZ p ( vertices ( var,0 ) +position ( 0 ),
                          vertices ( var,1 ) +position ( 1 ),
                          vertices ( var,2 ) +position ( 2 ) );
        cloud_projected->points.push_back ( p );
        // std::cout<<p<<std::endl;
    }


    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector< pcl::Vertices > polygons;
    try {
        chull.setInputCloud ( cloud_projected );
        chull.reconstruct ( *cloud_hull,polygons );
    } catch ( ... ) {
        ROS_ERROR ( "qhull error" );
        return false;
    }






    if ( ! ( cloud_hull->points.empty() ) ) {

        std::vector<geometry_msgs::Point> points;
        points.clear();
        //points.resize(cloud_hull->points.size());

        // Plottling
        visualization_msgs::Marker mkr;

        mkr.ns=polytope_name;
        mkr.action=visualization_msgs::Marker::ADD;
        mkr.type=visualization_msgs::Marker::TRIANGLE_LIST;
        mkr.header.frame_id=frame;

        // polygons is a vector of triangles represented by 3 indices
        // The indices correspond to points in cloud_hull
        // Therefore for each triangle in the polgyon
        // we find its three vertices and extract their x y z coordinates
        // this is then put in a
        for ( int tri = 0; tri<polygons.size(); ++tri ) {
            pcl::Vertices triangle=polygons[tri];
            for ( int var = 0; var<3; ++var ) {
                geometry_msgs::Point pp;
                pp.x=cloud_hull->points[triangle.vertices[var]].x;
                pp.y=cloud_hull->points[triangle.vertices[var]].y;
                pp.z=cloud_hull->points[triangle.vertices[var]].z;
                points.push_back ( pp );
            }
        }

        if ( hold_on_ ) {
            mkr.id=counter_*100;
        } else {
            mkr.id=2;
        }
        mkr.lifetime=ros::Duration ( 0.0 );
        mkr.color.r=color_line[0];
        mkr.color.g=color_line[1];
        mkr.color.b=color_line[2];
        mkr.color.a=color_line[3];//fmax(auto_alpha,0.1);

        //  double auto_alpha=(((double) (counter_))/(double) (nbr_ctrl_steps+3))-1;
        //  std::cout<<"autp alpha ="<<auto_alpha<<std::endl;



        mkr.scale.x=1.0;
        mkr.scale.y=1.0;
        mkr.scale.z=1.0;
        mkr.points=points;
        while ( mkr_pub.getNumSubscribers() <1 ) {
            ROS_INFO ( "Waiting for subs" );
            ros::spinOnce();
        }
        mkr_pub.publish ( mkr );


        mkr.type=visualization_msgs::Marker::SPHERE_LIST;
        mkr.header.frame_id=frame;
        mkr.id=1;


        mkr.lifetime=ros::Duration ( 0.0 );
        mkr.color.r=color_pts[0];
        mkr.color.g=color_pts[1];
        mkr.color.b=color_pts[2];
        mkr.color.a=color_pts[3];
        mkr.scale.x=0.005;
        mkr.scale.y=0.005;
        mkr.scale.z=0.005;
        mkr.points=points;

        while ( mkr_pub.getNumSubscribers() <1 ) {
            ROS_INFO ( "Waiting for subs" );
            ros::spinOnce();
        }
        mkr_pub.publish ( mkr );
        counter_++;
    } else {
        ROS_WARN ( "plotPolytope: Hull empty" );
        return false;
    }
    return true;
}





double RvizDisplay::plotPolytope ( Eigen::MatrixXd  vertices,
                                   Eigen::Vector3d position,
                                   std::vector<double>  color_pts,
                                   std::vector<double>  color_line,
                                   bool plot ) {
    return plotPolytope ( vertices,
                          position,
                          "world",
                          color_pts,
                          color_line,
                          plot );
}

double RvizDisplay::plotPolytope ( Eigen::MatrixXd  vertices,
                                   Eigen::Vector3d position,
                                   std::string frame,
                                   std::vector<double>  color_pts,
                                   std::vector<double>  color_line,
                                   bool plot ) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected ( new pcl::PointCloud<pcl::PointXYZ> );
    double vol ( 0.0 );
    // std::cout<<"Plotted Polytope Points "<<std::endl;
    for ( int var = 0; var < vertices.rows(); ++var ) {
        pcl::PointXYZ p ( vertices ( var,0 ) +position ( 0 ),
                          vertices ( var,1 ) +position ( 1 ),
                          vertices ( var,2 ) +position ( 2 ) );
        cloud_projected->points.push_back ( p );
        // std::cout<<p<<std::endl;
    }


    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector< pcl::Vertices > polygons;
    try {
        chull.setComputeAreaVolume ( true );
        chull.setInputCloud ( cloud_projected );
        chull.reconstruct ( *cloud_hull,polygons );
    } catch ( ... ) {
        ROS_ERROR ( "qhull error" );
        return 0.0;
    }


    vol=chull.getTotalVolume();




    if ( plot && ! ( cloud_hull->points.empty() ) ) {

        std::vector<geometry_msgs::Point> points;
        points.clear();
        //points.resize(cloud_hull->points.size());

        // Plottling
        visualization_msgs::Marker mkr;

        mkr.ns="polytope_new";
        mkr.action=visualization_msgs::Marker::ADD;
        mkr.type=visualization_msgs::Marker::TRIANGLE_LIST;
        mkr.header.frame_id=frame;

        // polygons is a vector of triangles represented by 3 indices
        // The indices correspond to points in cloud_hull
        // Therefore for each triangle in the polgyon
        // we find its three vertices and extract their x y z coordinates
        // this is then put in a
        for ( int tri = 0; tri<polygons.size(); ++tri ) {
            pcl::Vertices triangle=polygons[tri];
            for ( int var = 0; var<3; ++var ) {
                geometry_msgs::Point pp;
                pp.x=cloud_hull->points[triangle.vertices[var]].x;
                pp.y=cloud_hull->points[triangle.vertices[var]].y;
                pp.z=cloud_hull->points[triangle.vertices[var]].z;
                points.push_back ( pp );
            }
        }

        if ( hold_on_ ) {
            mkr.id=counter_*100;
        } else {
            mkr.id=2;
        }
        mkr.lifetime=ros::Duration ( 0.0 );
        mkr.color.r=color_line[0];
        mkr.color.g=color_line[1];
        mkr.color.b=color_line[2];
        mkr.color.a=color_line[3];//fmax(auto_alpha,0.1);


        mkr.scale.x=1.0;
        mkr.scale.y=1.0;
        mkr.scale.z=1.0;
        mkr.points=points;
        while ( mkr_pub.getNumSubscribers() <1 ) {
            ROS_INFO ( "Waiting for subs" );
            ros::spinOnce();
        }
        mkr_pub.publish ( mkr );


        mkr.type=visualization_msgs::Marker::SPHERE_LIST;
        mkr.header.frame_id=frame;
        mkr.id=1;


        mkr.lifetime=ros::Duration ( 0.0 );
        mkr.color.r=color_pts[0];
        mkr.color.g=color_pts[1];
        mkr.color.b=color_pts[2];
        mkr.color.a=color_pts[3];
        mkr.scale.x=0.005;
        mkr.scale.y=0.005;
        mkr.scale.z=0.005;
        mkr.points=points;

        while ( mkr_pub.getNumSubscribers() <1 ) {
            ROS_INFO ( "Waiting for subs" );
            ros::spinOnce();
        }
        mkr_pub.publish ( mkr );
        counter_++;
    } else {
        ROS_WARN ( "plotPolytope: Hull empty" );
    }
    return vol;
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "rviz_display" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    RvizDisplay rviz ( nh );
    ros::spin();
    return 0;
}
