#include "RobotModel.hpp"
#include "Planner.hpp"
#include "MotionPlanningRequest.hpp"
#include "MotionPlanningResponse.hpp"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <stdio.h>
#include <ctime>
#include <algorithm>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Utils.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>


#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>



#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>




#include <vector>
#include "math.h"
#include <iostream>
#include <algorithm>
#include <limits>



#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>




using namespace manipulator_planner_library;



boost::filesystem::path resolve_path( const boost::filesystem::path& p, const boost::filesystem::path& base )
{
    boost::filesystem::path abs_p = boost::filesystem::absolute(p,base);
    boost::filesystem::path result;
    for(boost::filesystem::path::iterator it=abs_p.begin(); it!=abs_p.end(); ++it)
    {
        if(*it == "..")
        {
            // /a/b/.. is not necessarily /a if b is a symbolic link
            if(boost::filesystem::is_symlink(result) )
                result /= *it;
            // /a/b/../.. is not /a/b/.. under most circumstances
            // We can end up with ..s in our result because of symbolic links
            else if(result.filename() == "..")
                result /= *it;
            // Otherwise it should be safe to resolve the parent
            else
                result = result.parent_path();
        }
        else if(*it == ".")
        {
            // Ignore
        }
        else
        {
            // Just cat other path entries
            result /= *it;
        }
    }
    return result;
}

ros::Publisher vis_pub ;


void publish_octree(ros::NodeHandle &node_handle, octomap::OcTree &octomap_ocTree)
{

    ros::Publisher  octomap_ocTree_Publisher = node_handle.advertise<octomap_msgs::Octomap>("octomapbinary", 1, true);
    std::string link_name="kuka_lbr_base";
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = link_name;
    octomap_msg.header.stamp = ros::Time::now();
    if (octomap_msgs::binaryMapToMsg(octomap_ocTree,octomap_msg))
    {
        ros::WallDuration sleep_time(4.0);
        sleep_time.sleep();
        octomap_ocTree_Publisher.publish(octomap_msg);
        sleep_time.sleep();
    }
    else
    {
        std::cout<<"Error serializing OctoMap"<<std::endl;
    }
}

void markers_visulizer(ros::NodeHandle &node_handle, std::vector<visualization_msgs::Marker> &markers)
{
    visualization_msgs::MarkerArray marker_array;
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

    for(int i=0;i<markers.size();i++)
    {
        marker_array.markers.push_back(markers.at(i));
    }
    ros::WallDuration sleep_t(0.4);
    sleep_t.sleep();
    vis_pub.publish( marker_array );
    sleep_t.sleep();
    return;
}

void wipe_marker(ros::NodeHandle &node_handle, std::vector<visualization_msgs::Marker> &markers)
{
    for(std::size_t i=0;i<markers.size();i++)
    {
        markers.at(i).action = visualization_msgs::Marker::DELETE;
    }
    markers_visulizer(node_handle, markers);
    markers.clear();
}


void generate_marker(std::vector<boost::shared_ptr<urdf::Visual> > visual_array, std::vector<visualization_msgs::Marker> &markers, std::string link_name,std::string &name_space,std::string urdf_file_abs_path, double red, double green, double blue)
{
    for(int i=0;i<visual_array.size();i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = link_name;
        marker.header.stamp = ros::Time();
        marker.id = i;
        marker.color.a = 1.0;
        marker.ns=name_space;
        marker.lifetime = ros::Duration();
/*
        srand (time(NULL));
        marker.color.r = (1.0*(rand() % 100) )/100;
        marker.color.g = (1.0*(rand() % 100) )/100;
        marker.color.b = (1.0*(rand() % 100) )/100;
*/

        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;



        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x= visual_array.at(i)->origin.position.x;
        marker.pose.position.y=visual_array.at(i)->origin.position.y;
        marker.pose.position.z=visual_array.at(i)->origin.position.z;

        marker.pose.orientation.w= visual_array.at(i)->origin.rotation.w;
        marker.pose.orientation.x=visual_array.at(i)->origin.rotation.x;
        marker.pose.orientation.y=visual_array.at(i)->origin.rotation.y;
        marker.pose.orientation.z=visual_array.at(i)->origin.rotation.z;
        marker.text="visual_array.at(i)->group_name";

        if(visual_array.at(i)->geometry->type == urdf::Geometry::MESH)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            boost::shared_ptr<urdf::Mesh> mesh_ptr= boost::static_pointer_cast <urdf::Mesh> (visual_array.at(i)->geometry);

            boost::filesystem::path relative_path_of_mesh_file_in_urdf_file=mesh_ptr->filename;
            std::string urdf_directory_path_string=urdf_file_abs_path.substr(0, urdf_file_abs_path.find_last_of("/") );


            boost::filesystem::path urdf_directory_path(urdf_directory_path_string);
            boost::filesystem::path abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );
//            std::cout<<"abs_path_of_mesh_file: " <<abs_path_of_mesh_file<<std::endl;
            marker.mesh_resource="file://"+abs_path_of_mesh_file.string();
            marker.scale.x=mesh_ptr->scale.x;
            marker.scale.y=mesh_ptr->scale.y;
            marker.scale.z=mesh_ptr->scale.z;
        }
        else if(visual_array.at(i)->geometry->type == urdf::Geometry::BOX)
        {
            boost::shared_ptr<urdf::Box> box_ptr= boost::static_pointer_cast <urdf::Box> (visual_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x=box_ptr->dim.x;
            marker.scale.y=box_ptr->dim.y;
            marker.scale.z=box_ptr->dim.z;

        }
        else if(visual_array.at(i)->geometry->type == urdf::Geometry::CYLINDER)
        {
            boost::shared_ptr<urdf::Cylinder> cylinder_ptr= boost::static_pointer_cast <urdf::Cylinder> (visual_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x=cylinder_ptr->radius;
            marker.scale.y=cylinder_ptr->radius;
            marker.scale.z=cylinder_ptr->length;

        }
        else if(visual_array.at(i)->geometry->type == urdf::Geometry::SPHERE)
        {
            boost::shared_ptr<urdf::Sphere> sphere_ptr= boost::static_pointer_cast <urdf::Sphere> (visual_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x=sphere_ptr->radius;
            marker.scale.y=sphere_ptr->radius;
            marker.scale.z=sphere_ptr->radius;
        }
        markers.push_back(marker);
    }

    return;

}

void generate_marker(std::vector<boost::shared_ptr<urdf::Collision> > collision_array, std::vector<visualization_msgs::Marker> &markers, std::string link_name,std::string &name_space,std::string urdf_file_abs_path, double red, double green, double blue)
{
    for(int i=0;i<collision_array.size();i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = link_name;
        marker.header.stamp = ros::Time();
        marker.id = i;
        marker.color.a = 1.0;
        marker.ns=name_space;
        marker.lifetime = ros::Duration();
/*
        srand (time(NULL));
        marker.color.r = (1.0*(rand() % 100) )/100;
        marker.color.g = (1.0*(rand() % 100) )/100;
        marker.color.b = (1.0*(rand() % 100) )/100;
*/

        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;



        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x= collision_array.at(i)->origin.position.x;
        marker.pose.position.y=collision_array.at(i)->origin.position.y;
        marker.pose.position.z=collision_array.at(i)->origin.position.z;

        marker.pose.orientation.w= collision_array.at(i)->origin.rotation.w;
        marker.pose.orientation.x=collision_array.at(i)->origin.rotation.x;
        marker.pose.orientation.y=collision_array.at(i)->origin.rotation.y;
        marker.pose.orientation.z=collision_array.at(i)->origin.rotation.z;
        marker.text="visual_array.at(i)->group_name";

        if(collision_array.at(i)->geometry->type == urdf::Geometry::MESH)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            boost::shared_ptr<urdf::Mesh> mesh_ptr= boost::static_pointer_cast <urdf::Mesh> (collision_array.at(i)->geometry);

            boost::filesystem::path relative_path_of_mesh_file_in_urdf_file=mesh_ptr->filename;
            std::string urdf_directory_path_string=urdf_file_abs_path.substr(0, urdf_file_abs_path.find_last_of("/") );


            boost::filesystem::path urdf_directory_path(urdf_directory_path_string);
            boost::filesystem::path abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );
//            std::cout<<"abs_path_of_mesh_file: " <<abs_path_of_mesh_file<<std::endl;
            marker.mesh_resource="file://"+abs_path_of_mesh_file.string();
            marker.scale.x=mesh_ptr->scale.x;
            marker.scale.y=mesh_ptr->scale.y;
            marker.scale.z=mesh_ptr->scale.z;
        }
        else if(collision_array.at(i)->geometry->type == urdf::Geometry::BOX)
        {
            boost::shared_ptr<urdf::Box> box_ptr= boost::static_pointer_cast <urdf::Box> (collision_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x=box_ptr->dim.x;
            marker.scale.y=box_ptr->dim.y;
            marker.scale.z=box_ptr->dim.z;

        }
        else if(collision_array.at(i)->geometry->type == urdf::Geometry::CYLINDER)
        {
            boost::shared_ptr<urdf::Cylinder> cylinder_ptr= boost::static_pointer_cast <urdf::Cylinder> (collision_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x=cylinder_ptr->radius;
            marker.scale.y=cylinder_ptr->radius;
            marker.scale.z=cylinder_ptr->length;

        }
        else if(collision_array.at(i)->geometry->type == urdf::Geometry::SPHERE)
        {
            boost::shared_ptr<urdf::Sphere> sphere_ptr= boost::static_pointer_cast <urdf::Sphere> (collision_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x=sphere_ptr->radius;
            marker.scale.y=sphere_ptr->radius;
            marker.scale.z=sphere_ptr->radius;
        }
        markers.push_back(marker);
    }

    return;

}

void generate_marker(std::vector<boost::shared_ptr<urdf::Collision> > collision_array, std::vector<visualization_msgs::Marker> &markers, std::string link_name,std::string &name_space,std::string urdf_file_abs_path)
{
    for(int i=0;i<collision_array.size();i++)
    {

        visualization_msgs::Marker marker;
        marker.header.frame_id = link_name;
        marker.header.stamp = ros::Time();
        marker.ns=name_space;
        marker.lifetime = ros::Duration();


        marker.id = i;
        marker.color.a = 1.0;



        timeval t1;
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec * t1.tv_sec)


/**/
        /*srand (time(NULL))*/;
//        srand (time(0));
        marker.color.r = (1.0*(rand() % 100) )/100;
        marker.color.g = (1.0*(rand() % 100) )/100;
        marker.color.b = (1.0*(rand() % 100) )/100;


/*        marker.color.r = 1.0;
        marker.color.g = 0;
        marker.color.b = 0;
*/


        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x= collision_array.at(i)->origin.position.x;
        marker.pose.position.y=collision_array.at(i)->origin.position.y;
        marker.pose.position.z=collision_array.at(i)->origin.position.z;

        marker.pose.orientation.w= collision_array.at(i)->origin.rotation.w;
        marker.pose.orientation.x=collision_array.at(i)->origin.rotation.x;
        marker.pose.orientation.y=collision_array.at(i)->origin.rotation.y;
        marker.pose.orientation.z=collision_array.at(i)->origin.rotation.z;
        marker.text="visual_array.at(i)->group_name";

        if(collision_array.at(i)->geometry->type == urdf::Geometry::MESH)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            boost::shared_ptr<urdf::Mesh> mesh_ptr= boost::static_pointer_cast <urdf::Mesh> (collision_array.at(i)->geometry);

            boost::filesystem::path relative_path_of_mesh_file_in_urdf_file=mesh_ptr->filename;
            std::string urdf_directory_path_string=urdf_file_abs_path.substr(0, urdf_file_abs_path.find_last_of("/") );


            boost::filesystem::path urdf_directory_path(urdf_directory_path_string);
            boost::filesystem::path abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );
//            std::cout<<"abs_path_of_mesh_file: " <<abs_path_of_mesh_file<<std::endl;
            marker.mesh_resource="file://"+abs_path_of_mesh_file.string();
            marker.scale.x=mesh_ptr->scale.x;
            marker.scale.y=mesh_ptr->scale.y;
            marker.scale.z=mesh_ptr->scale.z;
        }
        else if(collision_array.at(i)->geometry->type == urdf::Geometry::BOX)
        {
            boost::shared_ptr<urdf::Box> box_ptr= boost::static_pointer_cast <urdf::Box> (collision_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x=box_ptr->dim.x;
            marker.scale.y=box_ptr->dim.y;
            marker.scale.z=box_ptr->dim.z;

        }
        else if(collision_array.at(i)->geometry->type == urdf::Geometry::CYLINDER)
        {
            boost::shared_ptr<urdf::Cylinder> cylinder_ptr= boost::static_pointer_cast <urdf::Cylinder> (collision_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x=cylinder_ptr->radius;
            marker.scale.y=cylinder_ptr->radius;
            marker.scale.z=cylinder_ptr->length;

        }
        else if(collision_array.at(i)->geometry->type == urdf::Geometry::SPHERE)
        {
            boost::shared_ptr<urdf::Sphere> sphere_ptr= boost::static_pointer_cast <urdf::Sphere> (collision_array.at(i)->geometry);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x=sphere_ptr->radius;
            marker.scale.y=sphere_ptr->radius;
            marker.scale.z=sphere_ptr->radius;
        }
        markers.push_back(marker);
    }

    return;

}

void visualize_manipulability_map_in_space(ros::NodeHandle &node_handle,  std::string id, std::string frame_id,std::string absolute_path_to_file, double manipulability_value_threshold, int colormap)
{
    int number_of_cubes=0;
    double x;
    double y;
    double z;
    double manipulability_value;



    std::vector<float> manipulability_value_vector;
    std::vector<float> x_vector;
    std::vector<float> y_vector;
    std::vector<float> z_vector;



    double manipulability_value_min,manipulability_value_max;
    manipulability_value_max=-10000000;
    manipulability_value_min=10000000;

    double quaternion_x,quaternion_y,quaternion_z,quaternion_w;
    std::string line;
    std::ifstream myfile (absolute_path_to_file.c_str());

    std::vector<std::string> strs;
    std::string delim=" ";
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            strs.clear();
            boost::split(strs , line, boost::is_any_of(" "));
            x=lexical_cast<double>(strs.at(0) );
            y=lexical_cast<double>(strs.at(1) );
            z=lexical_cast<double>(strs.at(2) );

            quaternion_x=lexical_cast<double>(strs.at(3) );
            quaternion_y=lexical_cast<double>(strs.at(4) );
            quaternion_z=lexical_cast<double>(strs.at(5) );
            quaternion_w=lexical_cast<double>(strs.at(6) );

            manipulability_value=lexical_cast<double>(strs.at(7) );
            if(manipulability_value<manipulability_value_min)
            {
                manipulability_value_min=manipulability_value;
            }
            if(manipulability_value>manipulability_value_max)
            {
                manipulability_value_max=manipulability_value;
            }

                if (manipulability_value>manipulability_value_threshold)
                {
                    x_vector.push_back( lexical_cast<float>(strs.at(0) )  ) ;
                    y_vector.push_back( lexical_cast<float>(strs.at(1) )  ) ;
                    z_vector.push_back( lexical_cast<float>(strs.at(2) )  ) ;
                    manipulability_value_vector.push_back( lexical_cast<float>(strs.at(7) ) );
                    number_of_cubes++;
                }
        }
        myfile.close();
    }

    std::cout<<"total number of cubes in "+id+" is :" <<number_of_cubes <<std::endl;
    std::cout<<"manipulability_value_min: " <<manipulability_value_min <<std::endl;
    std::cout<<"manipulability_value_max: " <<manipulability_value_max <<std::endl;


    cv::Mat mat_raw_manipulability_value(manipulability_value_vector);
    std::cout<< "manipulability_value_vector.size() "<< manipulability_value_vector.size()<<std::endl;

    std::cout<<"mat_raw_manipulability_value.rows " <<mat_raw_manipulability_value.rows<<std::endl;
    std::cout<<"mat_raw_manipulability_value.cols " <<mat_raw_manipulability_value.cols<<std::endl;


    cv::Mat mat_normilized_manipulability_value(mat_raw_manipulability_value.rows,mat_raw_manipulability_value.cols ,CV_8UC1);
    cv::normalize( mat_raw_manipulability_value, mat_normilized_manipulability_value, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat() );

    cv::Mat cm_img0;

    cv::applyColorMap(mat_normilized_manipulability_value, cm_img0,colormap);



     std::cout<<"cm_img0.rows: " <<cm_img0.rows <<std::endl;
     std::cout<<"cm_img0.cols: " <<cm_img0.cols <<std::endl;





    std::vector<cv::Mat> bgr_planes;
    cv::split( cm_img0, bgr_planes );

    cv::Mat blue, green, red;

    blue=bgr_planes.at(0);
    green=bgr_planes.at(1);
    red=bgr_planes.at(2);




    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker >( "visualization_marker", 0 );


    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    //     marker1.ns = "my_namespace";
    marker.id = lexical_cast<int>(id);
    //marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;



    for(int i=0;i<x_vector.size();i++)
    {
        geometry_msgs::Point point;
        std_msgs::ColorRGBA color;

        //color.a=1.0;
        color.a=0.04;
        point.x=x_vector.at(i);

        point.y=y_vector.at(i);
        point.z=z_vector.at(i);

        color.r=( red.at<uchar>(i,0)*1.0 )/255.0;
        color.g=(green.at<uchar>(i,0)*1.0) /255.0;
        color.b=(blue.at<uchar>(i,0)*1.0 )/255.0 ;




        marker.points.push_back(point);
        marker.colors.push_back(color);
        marker.scale.x=0.02;
        marker.scale.y=0.02;
        marker.scale.z=0.02;

    }

    std::string tmp_keyboard_press = "";

    ros::WallDuration sleep_t(2);
    sleep_t.sleep();
    ros::spinOnce();
    std::cout << "Please enter to start publishing :"<<  std::endl;
    getline(std::cin, tmp_keyboard_press);


    vis_pub.publish( marker );
    sleep_t.sleep();
    ros::spinOnce();



    std::cout << "Please enter to continue :"<<  std::endl;
    getline(std::cin, tmp_keyboard_press);
    return;
}

void create_pointcloud_from_box(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,
                                            double x, double y, double z, Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, bool dense)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()) ;
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,-z/2 ) );

    if(dense)
    {
        double delta_x= 0.005;
        double delta_y= 0.005;
        double delta_z= 0.005;

        for(double deltaX = -x/2 + delta_x; deltaX < x/2; deltaX = deltaX+delta_x)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,-z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,-z/2 ) );

            for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,deltaZ ) );

            }

            for(double deltaY = -y/2 + delta_y; deltaY < y/2; deltaY = deltaY+delta_y)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,-z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,-z/2 ) );}

        }

        for(double deltaY = -y/2 + delta_y; deltaY < y/2; deltaY = deltaY+delta_y)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,-z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,-z/2 ) );

            for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,deltaZ ) );

            }

        }

        /*for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,deltaZ ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,deltaZ ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,deltaZ ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,deltaZ ) );

        }*/

    }

    pcl::transformPointCloud (*box_cloud_ptr, *transformed_box_cloud_ptr, link_visual_pose_in_sensor_frame_eigen_matrix);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




template< class PointT>
void createPCLHorizontalBAR(pcl::PointCloud<PointT>& pclCloud, double delta=0)
{
    double x,y,z, lenght, width,height, step_size;

    x=1.00;
    y=0.0;
    z=0.0 ;
    lenght=0.20;
    width=0.20;
    height=0.20;
    step_size=0.01;

    pcl::PointXYZ pcl_point;
    double epsilon =0.0001;

    for(double i=0.80+epsilon;i<1.10+epsilon;i=i+step_size)//x
    {
        for(double j=0.0+epsilon;j<0.10+epsilon;j=j+step_size)//y
        {
            for(double k=0.0+epsilon;k<1.10+epsilon;k=k+step_size)//z
            {
                pcl_point.x=i+delta;
                pcl_point.y=j+delta;
                pcl_point.z=k+delta;
                pclCloud.points.push_back(pcl_point);
            }
        }
    }

}


template< class PointT>
void createPCLWall(pcl::PointCloud<PointT>& pclCloud)
{
    double x,y,z, lenght, width,height, step_size;

    x=1.00;
    y=0.0;
    z=0.0 ;
    lenght=0.20;
    width=0.20;
    height=0.20;
    step_size=0.01;

    pcl::PointXYZ pcl_point;
    double epsilon =0.0001;

    for(double i=1.80+epsilon;i<1.85+epsilon;i=i+step_size)//x
    {
        for(double j=-1+epsilon;j<1+epsilon;j=j+step_size)//y
        {
            for(double k=-1+epsilon;k<2+epsilon;k=k+step_size)//z
            {
                pcl_point.x=i;
                pcl_point.y=j;
                pcl_point.z=k;
                pclCloud.points.push_back(pcl_point);
            }
        }
    }

}

template <class PointT>
void convertPCLpointcloudToOctomapPointCloud(pcl::PointCloud<PointT>& pclCloud, octomap::Pointcloud& octomapCloud)
{
    octomapCloud.reserve(pclCloud.points.size());
    typename   pcl::PointCloud<PointT>::const_iterator it;
    for (it = pclCloud.begin(); it != pclCloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan (it->x) && !std::isnan (it->y) && !std::isnan (it->z))
        octomapCloud.push_back(it->x, it->y, it->z);
    }
}




void convertOctomapPointCloudToOctomaptree(octomap::Pointcloud &octomapCloud, octomap::OcTree &octomap_Octree )
{
    octomap::point3d origin (0.0f, 0.0f, 0.0f);
    octomap_Octree.insertPointCloud(octomapCloud, origin);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void move_joints(ros::NodeHandle &node_handle,std::map<std::string, double> solution_for_given_pose)
{

    ros::Publisher joint_state_from_source_list_publisher=node_handle.advertise<sensor_msgs::JointState>("input_joint_msg",1,true);
    ros::WallDuration sleep_time(0.1);
//    sleep_time.sleep();

    std::string joint_name;
    double joint_value;

    sensor_msgs::JointState arbitrary_joints_values;

    for(std::map<std::string,double>::iterator it=solution_for_given_pose.begin();it!=solution_for_given_pose.end();it++)
    {
        joint_name=it->first;
        joint_value=it->second;


        arbitrary_joints_values.name.push_back(joint_name);
        arbitrary_joints_values.position.push_back(joint_value);


    }

    joint_state_from_source_list_publisher.publish<sensor_msgs::JointState>(arbitrary_joints_values);
    sleep_time.sleep();
}


void test_compute_manipubility_index(int argc, char ** argv)
{
/*
    With Yoshikawa’s manipulability index [3] a quality measure for redundant manipulators was introduced, which describes the distance to singular configurations.
*/




    std::string srdf_file_abs_path="/home/behnam/robot_models/kuka/kuka_config/config/kuka_lbr.srdf";
    std::string abs_path_to_urdf_file="/home/behnam/robot_models/kuka/kuka_description/urdf/kuka_lbr_relative_path.urdf";
    std::string ik_fast_shared_object_abs_path="/home/behnam/catkin_ws/devel/lib";
    boost::shared_ptr<RobotModel>  robot_model_ptr;

    robot_model_ptr.reset(new RobotModel(abs_path_to_urdf_file , srdf_file_abs_path,ik_fast_shared_object_abs_path) ) ;
    robot_model_ptr->robotModelInitialization();


//    std::string planningGroupName="right_arm";
//    std::string planningGroupName="left_arm";
//    std::vector< std::pair<std::string,urdf::Joint> > planning_groups_joints;
    std::string base_link;
    std::string tip_link;


//    std::map<std::string, double> joints_name_values;

//    robot_model_ptr->getPlanningGroupJointinformation(planningGroupName,planning_groups_joints,base_link,tip_link);
//    for(std::size_t i=0;i<planning_groups_joints.size();i++)
//    {
//        joints_name_values[planning_groups_joints.at(i).first ]=0.02;
//    }


    double manipulability_index;
    double arm_step_size=0.25;


    RobotState robot_state= robot_model_ptr->getRobotState();
    RobotJoint arm_1_joint= robot_state.robot_joints["kuka_lbr_r_joint_1"];
    RobotJoint arm_2_joint= robot_state.robot_joints["kuka_lbr_r_joint_2"];
    RobotJoint arm_3_joint= robot_state.robot_joints["kuka_lbr_r_joint_3"];
    RobotJoint arm_4_joint= robot_state.robot_joints["kuka_lbr_r_joint_4"];
    RobotJoint arm_5_joint= robot_state.robot_joints["kuka_lbr_r_joint_5"];
    RobotJoint arm_6_joint= robot_state.robot_joints["kuka_lbr_r_joint_6"];
    RobotJoint arm_7_joint= robot_state.robot_joints["kuka_lbr_r_joint_7"];


//    RobotJoint arm_1_joint= robot_state.robot_joints["kuka_lbr_r_joint_1"];
//    RobotJoint arm_2_joint= robot_state.robot_joints["kuka_lbr_r_joint_2"];
//    RobotJoint arm_3_joint= robot_state.robot_joints["kuka_lbr_r_joint_3"];
//    RobotJoint arm_4_joint= robot_state.robot_joints["kuka_lbr_r_joint_4"];
//    RobotJoint arm_5_joint= robot_state.robot_joints["kuka_lbr_r_joint_5"];
//    RobotJoint arm_6_joint= robot_state.robot_joints["kuka_lbr_r_joint_6"];
//    RobotJoint arm_7_joint= robot_state.robot_joints["kuka_lbr_r_joint_7"];


    //robot_model_ptr->getWorlCollision().get


    std::ofstream myfile;
    myfile.open ("/home/behnam/Desktop/kuka_calculate_manipulability_right_arm_forward_kinematic_log_in_base_link.txt");
//    myfile.open ("/home/behnam/Desktop/kuka_calculate_manipulability_left_arm_forward_kinematic_log_in_base_link.txt");

    double x,y,z, quaternion_x,quaternion_y,quaternion_z,quaternion_w;
    std::string line;
    for(double d0=arm_1_joint.getJointInfo().limits->lower ;d0<arm_1_joint.getJointInfo().limits->upper;d0=d0+arm_step_size)
    {
        std::cout<<"d0: "<<d0 <<std::endl;
        for(double d1=arm_2_joint.getJointInfo().limits->lower;d1<arm_2_joint.getJointInfo().limits->upper;d1=d1+arm_step_size)
        {
            for(double d2=arm_3_joint.getJointInfo().limits->lower;d2<arm_3_joint.getJointInfo().limits->upper;d2=d2+arm_step_size)
            {
                std::cout<<"d2: "<<d2 <<std::endl;

                for(double d3=arm_4_joint.getJointInfo().limits->lower;d3<arm_4_joint.getJointInfo().limits->upper;d3=d3+arm_step_size)
                {
//                    for(double d4=arm_5_joint.getJointInfo().limits->lower;d4<arm_5_joint.getJointInfo().limits->upper;d4=d4+arm_step_size)
//                    {
//                        for(double d5=arm_6_joint.getJointInfo().limits->lower;d5<arm_6_joint.getJointInfo().limits->upper;d5=d5+arm_step_size)
//                        {
//                            for(double d6=arm_7_joint.getJointInfo().limits->lower;d6<arm_7_joint.getJointInfo().limits->upper;d6=d6+arm_step_size)
//                            {
                                std::map<std::string, double> arm_joint_state_map;
                                arm_joint_state_map["kuka_lbr_r_joint_1"]=d0;
                                arm_joint_state_map["kuka_lbr_r_joint_2"]=d1;
                                arm_joint_state_map["kuka_lbr_r_joint_3"]=d2;
                                arm_joint_state_map["kuka_lbr_r_joint_4"]=d3;
                                arm_joint_state_map["kuka_lbr_r_joint_5"]=0;
                                arm_joint_state_map["kuka_lbr_r_joint_6"]=0;
                                arm_joint_state_map["kuka_lbr_r_joint_7"]=0;

                                KDL::Jacobian  jacobian;
                                robot_model_ptr->ComputeJacobain(base_link,tip_link, arm_joint_state_map, jacobian);
                                robot_model_ptr->ManipulabilityIndex(jacobian, manipulability_index);
//                                myfile << manipulability_index;
//                                std::cout<<"manipulability_index"<<manipulability_index <<std::endl;
                                robot_model_ptr->updateJointGroup( arm_joint_state_map);
                                KDL::Frame  eff_frame=robot_model_ptr->getRobotState().robot_links["kuka_lbr_r_link_7"].getLinkFrame();
                                x=eff_frame.p.x();
                                y=eff_frame.p.y();
                                z=eff_frame.p.z();
                                eff_frame.M.GetQuaternion( quaternion_x,quaternion_y,quaternion_z,quaternion_w);
                                line=lexical_cast<std::string>(x)+" "+
                                lexical_cast<std::string>(y)+" "+
                                lexical_cast<std::string>(z)+" "+
                                lexical_cast<std::string>(quaternion_x)+" "+
                                lexical_cast<std::string>(quaternion_y)+" "+
                                lexical_cast<std::string>(quaternion_z)+" "+
                                lexical_cast<std::string>(quaternion_w)+" "+
                                lexical_cast<std::string>(manipulability_index)+" "+
                                lexical_cast<std::string>(d0)+" "+
                                lexical_cast<std::string>(d1)+" "+
                                lexical_cast<std::string>(d2)+" "+
                                lexical_cast<std::string>(d3)+" "+
                                lexical_cast<std::string>(0)+" "+
                                lexical_cast<std::string>(0)+" "+
                                lexical_cast<std::string>(0)+"\n";

                                myfile << line;

//                            }
//                        }
//                    }
                }
            }
        }
    }

     myfile.close();
}


int calculate_manipulability_forward_kinematic(int argc, char ** argv)
{

    ros::init (argc, argv, "calculate_manipulability_forward_kinematic");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;

//    test_compute_manipubility_index(argc, argv);

    double right_arm_manipulability_value_threshold;
    double left_arm_manipulability_value_threshold;

    right_arm_manipulability_value_threshold=-10.0;
    left_arm_manipulability_value_threshold=-10.0;



    visualize_manipulability_map_in_space(node_handle, "0", "kuka_lbr_base","/home/behnam/Desktop/kuka_calculate_manipulability_right_arm_forward_kinematic_log_in_base_link.txt",right_arm_manipulability_value_threshold, cv::COLORMAP_AUTUMN);
    visualize_manipulability_map_in_space(node_handle, "1",  "kuka_lbr_base","/home/behnam/Desktop/kuka_calculate_manipulability_left_arm_forward_kinematic_log_in_base_link.txt",left_arm_manipulability_value_threshold, cv::COLORMAP_AUTUMN);

    return 0;

}


void test_planning(int argc, char ** argv)
{


    ros::init(argc,argv,"test_planning_collision_aila_dynamic_environment_replanning",1);
    ros::NodeHandle node_handle;
    ros::Publisher joint_state_from_source_list_publisher=node_handle.advertise<sensor_msgs::JointState>("input_joint_msg",1,true);
    vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

    ros::WallDuration sleep_time(0.05);




///////////////////////////////////////////////////////////Initializing the robot /////////////////////////////////////////////////////////////////////





    std::string srdf_file_abs_path="/home/behnam/robot_models/kuka/kuka_config/config/kuka_lbr.srdf";
    std::string abs_path_to_urdf_file="/home/behnam/robot_models/kuka/kuka_description/urdf/kuka_lbr_relative_path.urdf";
    std::string ik_fast_shared_object_abs_path="/home/behnam/catkin_ws/devel/lib";
    boost::shared_ptr<RobotModel>  robot_model_ptr;

    robot_model_ptr.reset(new RobotModel(abs_path_to_urdf_file , srdf_file_abs_path,ik_fast_shared_object_abs_path) ) ;
    robot_model_ptr->robotModelInitialization();



///////////////////////////////////////////////////////////seting up start and goal pose///////////////////////////////////////////////////////////////




    std::map<std::string,double> strat_robot_status;
    std::map<std::string,double> goal_robot_status;



    strat_robot_status["kuka_lbr_r_joint_1"]=0.0;
    strat_robot_status["kuka_lbr_r_joint_2"]=0.0;
    strat_robot_status["kuka_lbr_r_joint_3"]=0.0;
    strat_robot_status["kuka_lbr_r_joint_4"]=0.0;
    strat_robot_status["kuka_lbr_r_joint_5"]=0.0;
    strat_robot_status["kuka_lbr_r_joint_6"]=0.0;
    strat_robot_status["kuka_lbr_r_joint_7"]=0.0;




    goal_robot_status["kuka_lbr_r_joint_1"]=-1.29;
    goal_robot_status["kuka_lbr_r_joint_2"]=1.46;
    goal_robot_status["kuka_lbr_r_joint_3"]=0.08;
    goal_robot_status["kuka_lbr_r_joint_4"]=-0.33;
    goal_robot_status["kuka_lbr_r_joint_5"]=0.00;
    goal_robot_status["kuka_lbr_r_joint_6"]=-1.48;
    goal_robot_status["kuka_lbr_r_joint_7"]=2.00;





////////////////////////////////////////////////////////// Setting up motion planning request and response////////////////////////////////////////////////

    std::string planningGroupName="right_arm";



    MotionPlanningRequest motion_planning_request( strat_robot_status,goal_robot_status,robot_model_ptr,planningGroupName);
    MotionPlanningResponse motion_planning_response;



/////////////////////////////////////////////////////////////////////// Setting up the planner  ////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////start creating horizantal bar and wall////////////////////////////////////////////////////////////////////


    boost::shared_ptr<urdf::Collision>  horizantal_bar_collision(new urdf::Collision);
    boost::shared_ptr<urdf::Collision>  wall_collision(new urdf::Collision);

    boost::shared_ptr<urdf::Box> urdf_box_ptr(new urdf::Box);
    boost::shared_ptr<urdf::Box> urdf_box2_ptr(new urdf::Box);



    double horizontal_bar_x,horizontal_bar_y,horizontal_bar_z;
    double horizontal_bar_width,horizontal_bar_height,horizontal_bar_lenght;


    horizontal_bar_width=0.05;
    horizontal_bar_height=4.0;
    horizontal_bar_lenght=0.05;



//original location
    horizontal_bar_x=0.73;
    horizontal_bar_y=-1.30;
    horizontal_bar_z=0.0;



    urdf_box_ptr->dim.x=horizontal_bar_width;
    urdf_box_ptr->dim.y=horizontal_bar_lenght;
    urdf_box_ptr->dim.z=horizontal_bar_height ;






    horizantal_bar_collision->geometry=urdf_box_ptr;
    horizantal_bar_collision->origin.position.x=horizontal_bar_x;
    horizantal_bar_collision->origin.position.y=horizontal_bar_y;
    horizantal_bar_collision->origin.position.z=horizontal_bar_z;
    horizantal_bar_collision->geometry->type = urdf::Geometry::BOX;

    std::string world_collision_link_name="kuka_lbr_base";

    robot_model_ptr->addCollisionsToWorld(horizantal_bar_collision, world_collision_link_name);




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<boost::shared_ptr<urdf::Collision> > collision_array;
    double red=0.5;
    double green=0.5;
    double blue=0;

    std::vector<visualization_msgs::Marker> markers;
    std::string link_name="kuka_lbr_base";
    std::string name_space="horizantal_bar";
    collision_array.push_back(horizantal_bar_collision);

    generate_marker(collision_array, markers,link_name,name_space,abs_path_to_urdf_file,red, green, blue);


    markers_visulizer(node_handle, markers);






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    PlannersType type_of_planner= RRTConnect;
    double max_solve_time=100000000;
    unsigned int max_step_smoothing=0;

    std::map<std::string, std::string> planner_specific_parameters_kv;
    planner_specific_parameters_kv["range"]="0.1";
    Planner robot_motion_planner(motion_planning_request,  type_of_planner, max_solve_time, max_step_smoothing,planner_specific_parameters_kv);




///////////////////////////////////////////////////////////////start creating horizantal bar and wall////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////Solving and publishing ////////////////////////////////////////////////////////////////


    if( robot_motion_planner.Solve() )
    {

        motion_planning_response=robot_motion_planner.getMotionPlanningResponse();
        std::vector<std::map<std::string, double> > pathJointValues =motion_planning_response.getPathJointValues() ;

        std::map<std::string, double> pathJointValue_entry;
        sleep_time.sleep();
        for(std::size_t i=0;i<pathJointValues.size();i++)
        {
            std::string tmp_keyboard_press = "";
            std::cout << "Please enter to continue, executing joint command number :"<<i << std::endl;
            getline(std::cin, tmp_keyboard_press);
            std::cout << "----------------------" << std::endl;
            sleep_time.sleep();
            pathJointValue_entry=pathJointValues.at(i);
            sensor_msgs::JointState arbitrary_joints_values;
            for(std::map<std::string, double>::iterator it=pathJointValue_entry.begin(); it!=pathJointValue_entry.end(); it++   )
            {
                std::cout<<it->first<<":"<<it->second <<std::endl;
                arbitrary_joints_values.name.push_back(it->first);
                arbitrary_joints_values.position.push_back(it->second);
            }
            joint_state_from_source_list_publisher.publish<sensor_msgs::JointState>(arbitrary_joints_values);
        }
    }
    else
    {
        std::cout<<"planning task failed" <<std::endl;
    }


}


void optimisation_based(int argc, char** argv)
{

    ros::init(argc,argv,"publishing_joing_msg_source_list",1);
    ros::NodeHandle node_handle;
    ros::Publisher joint_state_from_source_list_publisher=node_handle.advertise<sensor_msgs::JointState>("input_joint_msg",1,true);
    ros::WallDuration sleep_time(0.2);


    vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );


///////////////////////////////////////////////////////////Initializing the robot /////////////////////////////////////////////////////////////////////




    std::string srdf_file_abs_path="/home/behnam/robot_models/kuka/kuka_config/config/kuka_lbr.srdf";
    std::string abs_path_to_urdf_file="/home/behnam/robot_models/kuka/kuka_description/urdf/kuka_lbr_relative_path.urdf";
    std::string ik_fast_shared_object_abs_path="/home/behnam/catkin_ws/devel/lib";
    boost::shared_ptr<RobotModel>  robot_model_ptr;

    robot_model_ptr.reset(new RobotModel(abs_path_to_urdf_file , srdf_file_abs_path) ) ;
    robot_model_ptr->robotModelInitialization();





///////////////////////////////////////////////////////////seting up start and goal pose///////////////////////////////////////////////////////////////
    std::map<std::string, double > start_joint_values_for_given_pose,goal_joint_values_for_given_pose;



    start_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=0.0;




    goal_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=-1.29;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=1.46;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.08;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=-0.33;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=0.00;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=-1.48;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=2.00;









////////////////////////////////////////////////////////// Setting up motion planning request and response////////////////////////////////////////////////

    std::string planningGroupName="right_arm";
    MotionPlanningRequest motion_planning_request( start_joint_values_for_given_pose,goal_joint_values_for_given_pose,robot_model_ptr,planningGroupName);
    MotionPlanningResponse motion_planning_response;




/////////////////////////////////////////////////////////////////////// Setting up the planner  ////////////////////////////////////////////////////////////







    std::map<std::string, std::string> planner_specific_parameters_kv;
    planner_specific_parameters_kv["range"]="0.1";


    PlannersType type_of_planner= RRTConnect;
    double max_solve_time=10;
    unsigned int max_step_smoothing=2;
    Planner robot_motion_planner(motion_planning_request,  type_of_planner, max_solve_time, max_step_smoothing,planner_specific_parameters_kv);



///////////////////////////////////////////////////////////////start creating horizantal bar and wall////////////////////////////////////////////////////////////////////


    boost::shared_ptr<urdf::Collision>  horizantal_bar_collision(new urdf::Collision);
    boost::shared_ptr<urdf::Collision>  wall_collision(new urdf::Collision);

    boost::shared_ptr<urdf::Box> urdf_box_ptr(new urdf::Box);
    boost::shared_ptr<urdf::Box> urdf_box2_ptr(new urdf::Box);



    double horizontal_bar_x,horizontal_bar_y,horizontal_bar_z;
    double horizontal_bar_width,horizontal_bar_height,horizontal_bar_lenght;


    horizontal_bar_width=0.20;
    horizontal_bar_height=4.0;
    horizontal_bar_lenght=0.20;



//original location
    horizontal_bar_x=0.56;
    horizontal_bar_y=-1.39;
    horizontal_bar_z=0.0;



    urdf_box_ptr->dim.x=horizontal_bar_width;
    urdf_box_ptr->dim.y=horizontal_bar_lenght;
    urdf_box_ptr->dim.z=horizontal_bar_height ;






    horizantal_bar_collision->geometry=urdf_box_ptr;
    horizantal_bar_collision->origin.position.x=horizontal_bar_x;
    horizantal_bar_collision->origin.position.y=horizontal_bar_y;
    horizantal_bar_collision->origin.position.z=horizontal_bar_z;
    horizantal_bar_collision->geometry->type = urdf::Geometry::BOX;

    std::string world_collision_link_name="kuka_lbr_base";

    robot_model_ptr->addCollisionsToWorld(horizantal_bar_collision, world_collision_link_name);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<boost::shared_ptr<urdf::Collision> > collision_array;
    double red=0.5;
    double green=0.5;
    double blue=0;

    std::vector<visualization_msgs::Marker> markers;
    std::string link_name="kuka_lbr_base";
    std::string name_space="horizantal_bar_collision";
    collision_array.push_back(horizantal_bar_collision);

    generate_marker(collision_array, markers,link_name,name_space,abs_path_to_urdf_file,red, green, blue);




//    red=0;
//    green=0.5;
//    blue=0;
////    collision_array.clear();
//    collision_array.push_back(horizantal_bar_collision);
//    name_space="horizantal_bar";
//    generate_marker(collision_array, markers,link_name,name_space,abs_path_to_urdf_file,red, green, blue);
//    markers_visulizer(node_handle, markers);









////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_bar_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix= Eigen::Affine3f::Identity();
    link_visual_pose_in_sensor_frame_eigen_matrix.translation() << horizontal_bar_x, horizontal_bar_y, horizontal_bar_z;
    bool dense=true;
    create_pointcloud_from_box(transformed_bar_cloud_ptr,horizontal_bar_width,horizontal_bar_lenght,horizontal_bar_height,link_visual_pose_in_sensor_frame_eigen_matrix,dense);
    octomap::Pointcloud octomapCloud_bar;
    convertPCLpointcloudToOctomapPointCloud<pcl::PointXYZ>( *transformed_bar_cloud_ptr.get()   , octomapCloud_bar);
    octomap::OcTree octomap_Octree(0.01);
    convertOctomapPointCloudToOctomaptree(octomapCloud_bar, octomap_Octree );


    //octomap_Octree.writeBinary("simple_tree.bt");


    publish_octree(node_handle, octomap_Octree);

/*
    robot_model_ptr->AddOctomapForOptmisationPlanning(octomap_Octree,false,1.0 );


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    double wall_x, wall_y, wall_z, wall_lenght,  wall_width,  wall_height;

    wall_lenght=0.05;
    wall_width=5.0;
    wall_height=5.0;

//original values
//    wall_x=-0.5;
//    wall_y=0;
//    wall_z=0.0;


    wall_x=-4;
    wall_y=-4;
    wall_z=0.0;

    urdf_box2_ptr->dim.x=wall_lenght;
    urdf_box2_ptr->dim.y=wall_width;
    urdf_box2_ptr->dim.z=wall_height;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    wall_collision->geometry=urdf_box2_ptr;
    wall_collision->origin.position.x=wall_x;
    wall_collision->origin.position.y=wall_y;
    wall_collision->origin.position.z=wall_z;
    wall_collision->geometry->type = urdf::Geometry::BOX;

    robot_model_ptr->addCollisionsToWorld(wall_collision, world_collision_link_name);







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    robot_model_ptr->updateJointGroup(start_joint_values_for_given_pose);
    std::cout<<"start state is " <<robot_model_ptr->IsStateIsValid()<<std::endl;

    robot_model_ptr->updateJointGroup(goal_joint_values_for_given_pose);
    std::cout<<"goal state is " <<robot_model_ptr->IsStateIsValid()<<std::endl;




////////////////////////////////////////////////////////////////////////Solving and publishing ////////////////////////////////////////////////////////////////

    if( robot_motion_planner.SolveShortestJointPath()  )
//    if( robot_motion_planner.SolveMaximumDistancesToObstacles()  )
    {
        motion_planning_response=robot_motion_planner.getMotionPlanningResponse();
        std::vector<std::map<std::string, double> > pathJointValues =motion_planning_response.getPathJointValues() ;

        std::map<std::string, double> pathJointValue_entry;

        sleep_time.sleep();
        for(std::size_t i=0;i<pathJointValues.size();i++)
        {
            std::string tmp_keyboard_press = "";
            //std::cout << "Please enter to continue, executing joint command number :"<<i << std::endl;
            getline(std::cin, tmp_keyboard_press);
            sleep_time.sleep();
            pathJointValue_entry=pathJointValues.at(i);
            sensor_msgs::JointState arbitrary_joints_values;
            for(std::map<std::string, double>::iterator it=pathJointValue_entry.begin(); it!=pathJointValue_entry.end(); it++   )
            {
                std::cout<<it->first<<":"<<it->second <<std::endl;
                arbitrary_joints_values.name.push_back(it->first);
                arbitrary_joints_values.position.push_back(it->second);
            }

            joint_state_from_source_list_publisher.publish<sensor_msgs::JointState>(arbitrary_joints_values);
            std::cout<<"---------------------------" <<std::endl;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        std::string path_to_log_file="kuka_right_arm.txt";
        writing_joint_log_trajectory(path_to_log_file, pathJointValues);
    }
    else
    {
        std::cout<<"planning task failed" <<std::endl;
    }

*/



}
//template<class T>
//void CreateEDTFromPointCloud(pcl::PointCloud<T>::Ptr cloud_ptr)
//{

//    // create the EDT object and initialize it with the map
//    int maxDistInCells = 2;
//    int sizeX, sizeY, sizeZ;
//    sizeX=4;
//    sizeY=4;
//    sizeZ=4;
//    bool*** map;
//    DynamicEDT3D distmap(maxDistInCells*maxDistInCells);
//    distmap.initializeMap(sizeX, sizeY, sizeZ, map);
//    distmap.update();

//    distmap.initializeEmpty();
//    distmap.occupyCell();

//    distmap.getDistance();


//}

void createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud, double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z )
{
//    #define PRINT_LOG_ASSIMP
    Assimp::Importer assimp_importer;
    const aiScene * scene;


/*
 There are several way to read a cad file, available ways to read he cad file are:
    aiProcess_Triangulate |aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes
*/

    scene=assimp_importer.ReadFile(abs_path_to_mesh_file.c_str() , aiProcess_Triangulate);


    #ifdef PRINT_LOG_ASSIMP
    std::cout<<"start extracting vertex and triangles from mesh file:" <<abs_path_to_mesh_file.c_str() <<std::endl;
    std::cout<<"number of meshes in the scene: " <<scene->mNumMeshes<<std::endl;
    #endif



    pcl::PointXYZ pcl_point;




    for(std::size_t i=0;i<scene->mNumMeshes;i++ )
    {

        #ifdef PRINT_LOG_ASSIMP
        std::cout<<"number of verticies (x,y,z poses) in the mesh number "<< i <<" is: " <<scene->mMeshes[i]->mNumVertices<<std::endl;
        std::cout<<"reading verticies: " <<std::endl;
        #endif
        for(std::size_t j=0;j<scene->mMeshes[i]->mNumVertices;j++ )
        {
            pcl_point.x=scene->mMeshes[i]->mVertices[j].x* scale_for_mesha_files_x;
            pcl_point.y=scene->mMeshes[i]->mVertices[j].y*scale_for_mesha_files_y;
            pcl_point.z=scene->mMeshes[i]->mVertices[j].z*scale_for_mesha_files_z;


            point_cloud.push_back(pcl_point);

        }
    }


//    delete scene;

}


void createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud,
                                          double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z,
                                          Eigen::Affine3f target_frame)
{
    pcl::PointCloud<pcl::PointXYZ> verts;

    createPointCloudFromMesh(abs_path_to_mesh_file, verts,
                             scale_for_mesha_files_x, scale_for_mesha_files_y, scale_for_mesha_files_z );

    pcl::transformPointCloud (verts, point_cloud, target_frame);
}


void visualize_EDT(int argc, char ** argv)
{
    ros::init(argc,argv,"publishing_joing_msg_source_list",1);
    ros::NodeHandle node_handle;
    ros::WallDuration sleep_time(0.2);


/////////////////////////////////////////////////////////////////////////////////////////////////
/*
    double horizontal_bar_x,horizontal_bar_y,horizontal_bar_z;
    double horizontal_bar_width,horizontal_bar_height,horizontal_bar_lenght;

    double epsilone=0.0;
    
    horizontal_bar_width=0.05;
    horizontal_bar_height=2.0;
    horizontal_bar_lenght=0.05;



    horizontal_bar_x=0.50+epsilone;
    horizontal_bar_y=-1.40+epsilone;
    horizontal_bar_z=0.0+epsilone;


    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_bar_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix= Eigen::Affine3f::Identity();
    link_visual_pose_in_sensor_frame_eigen_matrix.translation() << horizontal_bar_x, horizontal_bar_y, horizontal_bar_z;
    bool dense=true;
    create_pointcloud_from_box(transformed_bar_cloud_ptr,horizontal_bar_width,horizontal_bar_lenght,horizontal_bar_height,link_visual_pose_in_sensor_frame_eigen_matrix,dense);
    octomap::Pointcloud octomapCloud_bar;
    convertPCLpointcloudToOctomapPointCloud<pcl::PointXYZ>( *transformed_bar_cloud_ptr.get()   , octomapCloud_bar);
    octomap::OcTree octomap_Octree(0.01);
    convertOctomapPointCloudToOctomaptree(octomapCloud_bar, octomap_Octree );
    octomap_Octree.writeBinary( "octoMap.bt");


    double x_min,y_min,z_min, x_max,y_max,z_max;

    x_min=-3;
    y_min=-3;
    z_min=-3;

    x_max=2;
    y_max=2;
    z_max=2;




    octomap::point3d min(x_min,y_min,z_min);
    std::cout<<"Metric min: "<<min.x()<<","<<min.y()<<","<<min.z()<<std::endl;
    octomap::point3d max(x_max,y_max,z_max);
    std::cout<<"Metric max: "<<max.x()<<","<<max.y()<<","<<max.z()<<std::endl;

    bool unknownAsOccupied = false;
    float maxDist = 4.0;
    DynamicEDTOctomap distmap(maxDist, &octomap_Octree, min, max, unknownAsOccupied);
    distmap.update();
    publish_octree(node_handle, octomap_Octree);
*/
//////////////////////////////////////////////////////////////////////////////////////////
    if(argc<=1)
    {
        std::cout<<"usage: "<<argv[0]<<" <octoMap.bt>"<<std::endl;
        exit(0);
    }
////////////////////////////////////////////////////////////////////////////////////////
    octomap::OcTree *octomap_Octree = NULL;
    octomap_Octree = new octomap::OcTree(0.01);
    //read in octotree
    octomap_Octree->readBinary(argv[1]);
    publish_octree(node_handle, *octomap_Octree);
    std::cout<<"read in tree, "<<octomap_Octree->getNumLeafNodes()<<" leaves "<<std::endl;
    double x_min,y_min,z_min, x_max,y_max,z_max;

    x_min=-2;
    y_min=-2;
    z_min=0;

    x_max=2;
    y_max=0;
    z_max=2;
//    octomap_Octree->getMetricMin(x_min,y_min,z_min);
    octomap::point3d min(x_min,y_min,z_min);
    std::cout<<"Metric min: "<<x_min<<","<<y_min<<","<<z_min<<std::endl;
//    octomap_Octree->getMetricMax(x_max,y_max,z_max);
    octomap::point3d max(x_max,y_max,z_max);
    std::cout<<"Metric max: "<<x_max<<","<<y_max<<","<<z_max<<std::endl;
    bool unknownAsOccupied = false;
    float maxDist = 2.0;
    DynamicEDTOctomap distmap(maxDist, octomap_Octree, min, max, unknownAsOccupied);
    distmap.update();
    publish_octree(node_handle, *octomap_Octree);


    std::vector<float> distances_vector;


     double step_size=0.1;

     for(double x=x_min;x<x_max;x=x+step_size)
     {
         for(double y=y_min;y<y_max;y=y+step_size)
         {
             for(double z=z_min;z<z_max;z=z+step_size)
             {
                 octomap::point3d p(x,y,z);
                 distances_vector.push_back(distmap.getDistance(p));
             }
         }
     }



    cv::Mat mat_raw_distances_vector(distances_vector);




    cv::Mat mat_normilized_distances_value(mat_raw_distances_vector.rows,mat_raw_distances_vector.cols ,CV_8UC1);
    cv::normalize( mat_raw_distances_vector, mat_normilized_distances_value, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat() );

    cv::Mat cm_img0;

//cv::COLORMAP_AUTUMN
//    cv:0:COLORMAP_BONE

    cv::applyColorMap(mat_normilized_distances_value, cm_img0,cv::COLORMAP_HOT);



    std::cout<<"cm_img0.rows: " <<cm_img0.rows <<std::endl;
    std::cout<<"cm_img0.cols: " <<cm_img0.cols <<std::endl;





    std::vector<cv::Mat> bgr_planes;
    cv::split( cm_img0, bgr_planes );

    cv::Mat blue, green, red;

    blue=bgr_planes.at(0);
    green=bgr_planes.at(1);
    red=bgr_planes.at(2);


    
    std::cout<<"blue.rows: " <<blue.rows <<std::endl;
    std::cout<<"blue.cols: " <<blue.cols <<std::endl;
    
    

    std::string frame_id="kuka_lbr_base";

    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker >( "visualization_marker", 0 );


    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = lexical_cast<int>(1);
    marker.type = visualization_msgs::Marker::CUBE_LIST;
//    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;



//    std::string file_name;
//    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFileSTL(file_name,mesh);





     int i=0;
     for(double x=x_min;x<x_max;x=x+step_size)
     {
         for(double y=y_min;y<y_max;y=y+step_size)
         {
             for(double z=z_min;z<z_max;z=z+step_size)
             {
                 //std::cout<<"i: " << i<<std::endl;
                 geometry_msgs::Point point;
                 std_msgs::ColorRGBA color;

                 color.a=0.03;
                 point.x=x;
                 point.y=y;
                 point.z=z;

                 color.r=(red.at<uchar>(i,0)*1.0 )/255.0;
                 color.g=(green.at<uchar>(i,0)*1.0) /255.0;
                 color.b=(blue.at<uchar>(i,0)*1.0 )/255.0 ;

                 marker.points.push_back(point);
                 marker.colors.push_back(color);
                 marker.scale.x=0.1;
                 marker.scale.y=0.1;
                 marker.scale.z=0.1;
                 i++;
             }
         }
     }
    
    


    std::string tmp_keyboard_press = "";

    ros::WallDuration sleep_t(2);
    sleep_t.sleep();
    ros::spinOnce();
    std::cout << "Please enter to start publishing :"<<  std::endl;
    getline(std::cin, tmp_keyboard_press);


    vis_pub.publish( marker );
    sleep_t.sleep();
    ros::spinOnce();



    std::cout << "Please enter to continue :"<<  std::endl;
    getline(std::cin, tmp_keyboard_press);




//    std::string abs_path_to_mesh_file;
//    pcl::PointCloud<pcl::PointXYZ> point_cloud_dude;
//    double scale_for_mesha_files_x, scale_for_mesha_files_y, scale_for_mesha_files_z;


//    abs_path_to_mesh_file="/home/behnam/Desktop/Room.STL";

//    scale_for_mesha_files_x=1.0;
//    scale_for_mesha_files_y=1.0;
//    scale_for_mesha_files_z=1.0;

//    Eigen::Affine3f target_frame=Eigen::Affine3f::Identity();
//    createPointCloudFromMesh(abs_path_to_mesh_file, point_cloud_dude, scale_for_mesha_files_x,scale_for_mesha_files_y, scale_for_mesha_files_z,target_frame);




//    octomap::Pointcloud octomapCloud_dude;
//    convertPCLpointcloudToOctomapPointCloud<pcl::PointXYZ>( point_cloud_dude   , octomapCloud_dude);
//    octomap::OcTree octomap_Octree_dude(0.01);
//    convertOctomapPointCloudToOctomaptree(octomapCloud_dude, octomap_Octree_dude );
//    octomap_Octree_dude.writeBinary( "Room.bt");


    return;



}


template <class Type>
void concatenatePointCloud(pcl::PointCloud<Type> &cloud_a, pcl::PointCloud<Type>  &cloud_b, pcl::PointCloud<Type>  &cloud_c)
{
    cloud_c  = cloud_a;
    cloud_c += cloud_b;
}



void VisualizeTrajectory(std::vector<std::map<std::string, double> > pathJointValues, std::string link_name, boost::shared_ptr<RobotModel>  robot_model_ptr,std::string abs_path_to_urdf_file, std::string name_space_eff_path ,ros::NodeHandle &node_handle ,double red=1.0, double green=0.27 , double blue=0.0 )
{

    std::vector<visualization_msgs::Marker> link_markers;

    for(std::size_t i=0;i<pathJointValues.size();i++)
    {

//        std::vector<boost::shared_ptr<urdf::Collision> > link_collisions;
        std::vector<boost::shared_ptr<urdf::Visual> > link_visuals;
        robot_model_ptr->updateJointGroup(pathJointValues.at(i) );
//        robot_model_ptr->getRobotState().robot_links[link_name].getLinkCollisions(link_collisions);
        robot_model_ptr->getRobotState().robot_links[link_name].getLinkVisuals (link_visuals);
        std::string name_space_eff_path_=name_space_eff_path+ lexical_cast<std::string>(i);
        std::string base_link_frame_id=robot_model_ptr->getURDF()->getRoot()->name;
        generate_marker(link_visuals,link_markers, base_link_frame_id,name_space_eff_path_,abs_path_to_urdf_file, red, blue, green);
//        generate_marker(link_collisions,link_markers, base_link_frame_id,name_space_eff_path_,abs_path_to_urdf_file, red, blue, green);
    }
    markers_visulizer(node_handle, link_markers);


}



void optimisation_based_single_tree(int argc, char ** argv)
{

    ros::init(argc,argv,"publishing_joing_msg_source_list",1);
    ros::NodeHandle node_handle;
    ros::Publisher joint_state_from_source_list_publisher=node_handle.advertise<sensor_msgs::JointState>("input_joint_msg",1,true);
    ros::WallDuration sleep_time(0.2);


    vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );


///////////////////////////////////////////////////////////Initializing the robot /////////////////////////////////////////////////////////////////////




    std::string srdf_file_abs_path="/home/behnam/robot_models/kuka/kuka_config/config/kuka_lbr.srdf";
    std::string abs_path_to_urdf_file="/home/behnam/robot_models/kuka/kuka_description/urdf/kuka_lbr_relative_path.urdf";
    std::string ik_fast_shared_object_abs_path="/home/behnam/catkin_ws/devel/lib";
    boost::shared_ptr<RobotModel>  robot_model_ptr;

    robot_model_ptr.reset(new RobotModel(abs_path_to_urdf_file , srdf_file_abs_path) ) ;
    robot_model_ptr->robotModelInitialization();





///////////////////////////////////////////////////////////seting up start and goal pose///////////////////////////////////////////////////////////////
    std::map<std::string, double > start_joint_values_for_given_pose,goal_joint_values_for_given_pose;



    start_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=0.0;




    goal_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=-1.50;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=1.61;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.08;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=-0.33;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=-1.47;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=0.50;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=0.00;









////////////////////////////////////////////////////////// Setting up motion planning request and response////////////////////////////////////////////////

    std::string planningGroupName="kuka_right_arm";
    MotionPlanningRequest motion_planning_request( start_joint_values_for_given_pose,goal_joint_values_for_given_pose,robot_model_ptr,planningGroupName);
    MotionPlanningResponse motion_planning_response;

/////////////////////////////////////////////////////////////////////// Setting up dude, room  ////////////////////////////////////////////////////////////
/*
    //reading dude.stl and creating pointcloud

    std::string abs_path_to_mesh_file;
    pcl::PointCloud<pcl::PointXYZ> point_cloud_dude;
    double scale_for_mesha_files_x, scale_for_mesha_files_y, scale_for_mesha_files_z;
    abs_path_to_mesh_file="/home/behnam/Desktop/Dude.STL";
    scale_for_mesha_files_x=1.0;
    scale_for_mesha_files_y=1.0;
    scale_for_mesha_files_z=1.0;
    Eigen::Affine3f target_frame=Eigen::Affine3f::Identity();
    createPointCloudFromMesh(abs_path_to_mesh_file, point_cloud_dude, scale_for_mesha_files_x,scale_for_mesha_files_y, scale_for_mesha_files_z,target_frame);



//reading room.stl and creating pointcloud
    pcl::PointCloud<pcl::PointXYZ> point_cloud_room;
    abs_path_to_mesh_file="/home/behnam/Desktop/room.STL";
    createPointCloudFromMesh(abs_path_to_mesh_file, point_cloud_room, scale_for_mesha_files_x,scale_for_mesha_files_y, scale_for_mesha_files_z,target_frame);
    pcl::PointCloud<pcl::PointXYZ> point_cloud_room_dude;

    concatenatePointCloud<pcl::PointXYZ>(point_cloud_room, point_cloud_dude, point_cloud_room_dude);




    octomap::Pointcloud octomapCloud_room_dude;
    convertPCLpointcloudToOctomapPointCloud<pcl::PointXYZ>( point_cloud_room_dude   , octomapCloud_room_dude);
    octomap::OcTree octomap_Octree_dude(0.01);
    convertOctomapPointCloudToOctomaptree(octomapCloud_room_dude, octomap_Octree_dude );
    octomap_Octree_dude.writeBinary( "room_dude.bt");
*/
/////////////////////////////////////////////////////////////////////// Setting up the planner  ////////////////////////////////////////////////////////////

//    argv[1]= "room_dude.bt";
//    visualize_EDT(2, argv);
//    octomap::OcTree *octomap_Octree = NULL;
//    octomap_Octree = new octomap::OcTree(0.01);
//    //read in octotree
//    octomap_Octree->readBinary(argv[1]);


//    double x_min,y_min,z_min, x_max,y_max,z_max;

//    x_min=-2;
//    y_min=-2;
//    z_min=0;

//    x_max=2;
//    y_max=0;
//    z_max=2;

//    robot_model_ptr->AddOctomapForOptmisationPlanning(*octomap_Octree, x_min,y_min,z_min, x_max,y_max,z_max,false, 2);

//    std::string world_collision_link_name="kuka_lbr_base";


//    fcl::OcTree* fcl_octomap_Octree = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octomap_Octree));


//    fcl::CollisionObject tree_obj((boost::shared_ptr<fcl::CollisionGeometry>(fcl_octomap_Octree )));




//    boost::shared_ptr<fcl::CollisionObject> collisionObject_ptr=boost::make_shared<fcl::CollisionObject>(tree_obj);
//    robot_model_ptr->addCollisionsToWorld(collisionObject_ptr , world_collision_link_name);



//    boost::shared_ptr<fcl::CollisionObject> collisionObject_ptr;
//    collisionObject_ptr.reset(new fcl::CollisionObject(tree_obj) );
//    robot_model_ptr->addCollisionsToWorld(collisionObject_ptr , world_collision_link_name);



    std::map<std::string, std::string> planner_specific_parameters_kv;
    planner_specific_parameters_kv["range"]="0.4";


//    PlannersType type_of_planner= RRTstar;
    PlannersType type_of_planner= RRTConnect;
//    PlannersType type_of_planner= RRT;
    double max_solve_time=20;
    unsigned int max_step_smoothing=2;
    Planner robot_motion_planner(motion_planning_request,type_of_planner, max_solve_time, max_step_smoothing,planner_specific_parameters_kv);




    robot_model_ptr->updateJointGroup(start_joint_values_for_given_pose);
    if(robot_model_ptr->IsStateIsValid())
    {
        std::cout<<"start state is valid" <<std::endl;
    }
    else
    {
        std::cout<<"start state is not valid" <<std::endl;
    }

    robot_model_ptr->updateJointGroup(goal_joint_values_for_given_pose);
    if(robot_model_ptr->IsStateIsValid())
    {
        std::cout<<"goal state is valid" <<std::endl;
    }
    else
    {
        std::cout<<"goal state is not valid" <<std::endl;
    }







////////////////////////////////////////////////////////////////////////Solving and publishing ////////////////////////////////////////////////////////////////

    //if( robot_motion_planner.SingleTreeSolveMaximumDistancesToObstacles()  )
//    if( robot_motion_planner.SolveTaskInJointSpace()  )
    if( robot_motion_planner.SolveShortestJointPath()  )
    {
        motion_planning_response=robot_motion_planner.getMotionPlanningResponse();
        std::vector<std::map<std::string, double> > pathJointValues =motion_planning_response.getPathJointValues();
        std::vector<std::map<std::string, double> > RRTStarpathJointValues =motion_planning_response.getRRTStarPathJointValues();


        std::string eff_link_name="kuka_lbr_r_link_7";
        std::string name_space_eff_path="RRTConnect";


        double red=1.0;
        double green=0.27;
        double blue=0.0;

        VisualizeTrajectory(pathJointValues, eff_link_name, robot_model_ptr, abs_path_to_urdf_file, name_space_eff_path ,node_handle,red, green, blue );


        red=0.0;
        green=1.0;
        blue=0.0;
        name_space_eff_path="RRTStar";



        VisualizeTrajectory(RRTStarpathJointValues, eff_link_name, robot_model_ptr, abs_path_to_urdf_file, name_space_eff_path ,node_handle,red, green, blue );


        std::map<std::string, double> pathJointValue_entry;

        sleep_time.sleep();
        for(std::size_t i=0;i<pathJointValues.size();i++)
        {
            std::string tmp_keyboard_press = "";
            //std::cout << "Please enter to continue, executing joint command number :"<<i << std::endl;
            getline(std::cin, tmp_keyboard_press);
            sleep_time.sleep();
            pathJointValue_entry=pathJointValues.at(i);
            sensor_msgs::JointState arbitrary_joints_values;
            for(std::map<std::string, double>::iterator it=pathJointValue_entry.begin(); it!=pathJointValue_entry.end(); it++   )
            {
                std::cout<<it->first<<":"<<it->second <<std::endl;
                arbitrary_joints_values.name.push_back(it->first);
                arbitrary_joints_values.position.push_back(it->second);
            }

            joint_state_from_source_list_publisher.publish<sensor_msgs::JointState>(arbitrary_joints_values);
            std::cout<<"---------------------------" <<std::endl;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        std::string path_to_log_file="kuka_right_arm.txt";
        writing_joint_log_trajectory(path_to_log_file, pathJointValues);
    }
    else
    {
        std::cout<<"planning task failed" <<std::endl;
    }
/**/

}


void test_fk_right_arm_kuka(int argc, char ** argv)
{
    ros::init(argc,argv,"publishing_joing_msg_source_list",1);
    ros::NodeHandle node_handle;
    ros::Publisher joint_state_from_source_list_publisher=node_handle.advertise<sensor_msgs::JointState>("input_joint_msg",1,true);
    ros::WallDuration sleep_time(0.2);


    std::string srdf_file_abs_path="/home/behnam/robot_models/kuka/kuka_config/config/kuka_lbr.srdf";
    std::string abs_path_to_urdf_file="/home/behnam/robot_models/kuka/kuka_description/urdf/kuka_lbr_relative_path.urdf";
    std::string ik_fast_shared_object_abs_path="/home/behnam/catkin_ws/devel/lib";
    boost::shared_ptr<RobotModel>  robot_model_ptr;

    robot_model_ptr.reset(new RobotModel(abs_path_to_urdf_file , srdf_file_abs_path,ik_fast_shared_object_abs_path) ) ;
    robot_model_ptr->robotModelInitialization();





///////////////////////////////////////////////////////////seting up start and goal pose///////////////////////////////////////////////////////////////
    std::map<std::string, double > start_joint_values_for_given_pose,goal_joint_values_for_given_pose;



//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=0.0;
//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=0.0;
//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.0;
//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=0.0;
//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=0.0;
//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=0.0;
//    goal_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=0.0;



    goal_joint_values_for_given_pose["kuka_lbr_l_joint_1"]=0.1;
    goal_joint_values_for_given_pose["kuka_lbr_l_joint_2"]=0.1;
    goal_joint_values_for_given_pose["kuka_lbr_l_joint_3"]=0.1;
    goal_joint_values_for_given_pose["kuka_lbr_l_joint_4"]=0.1;
    goal_joint_values_for_given_pose["kuka_lbr_l_joint_5"]=0.1;
    goal_joint_values_for_given_pose["kuka_lbr_l_joint_6"]=0.1;
    goal_joint_values_for_given_pose["kuka_lbr_l_joint_7"]=0.1;









     KDL::Frame pose_in_base_link;

//    std::string chain_root_link="kuka_lbr_r_link_0";
    std::string chain_root_link="kuka_lbr_l_link_0";

//     std::string tip_link="kuka_lbr_r_link_7";
     std::string tip_link="kuka_lbr_l_link_7";




    std::cout<<"fk using KDL" <<std::endl;
    robot_model_ptr->fkSolverUsingKDL(chain_root_link,tip_link,goal_joint_values_for_given_pose,  pose_in_base_link);
    move_joints(node_handle,goal_joint_values_for_given_pose);




     double Quaternion_x,Quaternion_y,Quaternion_z,Quaternion_w;

     pose_in_base_link.M.GetQuaternion(Quaternion_x,Quaternion_y,Quaternion_z,Quaternion_w);


     std::cout<<"pose_in_base_link.p.x() " <<pose_in_base_link.p.x() <<std::endl;
     std::cout<<"pose_in_base_link.p.y() "<<pose_in_base_link.p.y() <<std::endl;
     std::cout<<"pose_in_base_link.p.z() " <<pose_in_base_link.p.z() <<std::endl;

     std::cout<<"Quaternion_x " <<Quaternion_x <<std::endl;
     std::cout<<"Quaternion_y " <<Quaternion_y <<std::endl;
     std::cout<<"Quaternion_z " <<Quaternion_z <<std::endl;
     std::cout<<"Quaternion_w " <<Quaternion_w <<std::endl;




    std::cout<<"fk using IKFAST" <<std::endl;
//    std::string planningGroupName="kuka_right_arm";
    std::string planningGroupName="kuka_left_arm";
    robot_model_ptr->fkSolverUsingIKFAST(goal_joint_values_for_given_pose,planningGroupName,pose_in_base_link);


    pose_in_base_link.M.GetQuaternion(Quaternion_x,Quaternion_y,Quaternion_z,Quaternion_w);


    std::cout<<"pose_in_base_link.p.x() " <<pose_in_base_link.p.x() <<std::endl;
    std::cout<<"pose_in_base_link.p.y() "<<pose_in_base_link.p.y() <<std::endl;
    std::cout<<"pose_in_base_link.p.z() " <<pose_in_base_link.p.z() <<std::endl;

    std::cout<<"Quaternion_x " <<Quaternion_x <<std::endl;
    std::cout<<"Quaternion_y " <<Quaternion_y <<std::endl;
    std::cout<<"Quaternion_z " <<Quaternion_z <<std::endl;
    std::cout<<"Quaternion_w " <<Quaternion_w <<std::endl;
}

void test_orientation_constraint_joint_space(int argc, char** argv )
{
    ros::init(argc,argv,"publishing_joing_msg_source_list",1);
    ros::NodeHandle node_handle;
    ros::Publisher joint_state_from_source_list_publisher=node_handle.advertise<sensor_msgs::JointState>("input_joint_msg",1,true);
    ros::WallDuration sleep_time(0.2);

    std::string srdf_file_abs_path="/home/behnam/robot_models/kuka/kuka_config/config/kuka_lbr.srdf";
    std::string abs_path_to_urdf_file="/home/behnam/robot_models/kuka/kuka_description/urdf/kuka_lbr_relative_path.urdf";
    std::string ik_fast_shared_object_abs_path="/home/behnam/catkin_ws/devel/lib";
    boost::shared_ptr<RobotModel>  robot_model_ptr;

    robot_model_ptr.reset(new RobotModel(abs_path_to_urdf_file ,srdf_file_abs_path, ik_fast_shared_object_abs_path) ) ;
    robot_model_ptr->robotModelInitialization();


    double x_start,y_start,z_start,roll_start,pitch_start, yaw_start,x_goal,y_goal,z_goal,roll_goal,pitch_goal, yaw_goal;
    std::string A_Frame="kuka_lbr_r_link_0";
    std::string C_Frame="kuka_lbr_r_link_7";
    std::string B_Frame="kuka_lbr_base";
    KDL::Frame start_pose_in_base_link,goal_pose_in_base_link ,start_pose_in_kuka_lbr_r_link_0, goal_pose_in_kuka_lbr_r_link_0;

    std::map<std::string, double> start_pose_ik_result, goal_pose_ik_result;

///////////////////////////////////////////////////////////////////free joint param///////////////////////////////////////////////////////////////////////

    std::string planningGroupName="kuka_right_arm";
    std::vector<RobotFreeJointParameter>  vector_robot_free_joint_parameters;
    std::string free_joint_name="kuka_lbr_r_joint_3";
    double step_size=0.01;
    RobotFreeJointParameter free_joint_parameters(free_joint_name,step_size , robot_model_ptr);
    vector_robot_free_joint_parameters.push_back(free_joint_parameters);



/////////////////////////////////////////////////////////////////////start pose ////////////////////////////////////////////////////////////////////////
    std::map<std::string, double> start_joint_values_for_given_pose, goal_joint_values_for_given_pose;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=0.1;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=0.2;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=-0.1;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=0.0;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=0.1;
    start_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=0.2;









    robot_model_ptr->updateJointGroup(start_joint_values_for_given_pose);
    start_pose_in_base_link= robot_model_ptr->getRobotState().robot_links["kuka_lbr_r_link_7"].getLinkFrame();
    robot_model_ptr->ConvertPoseBetweenFrames(B_Frame,start_pose_in_base_link,A_Frame,start_pose_in_kuka_lbr_r_link_0);


    x_start=start_pose_in_kuka_lbr_r_link_0.p.x();
    y_start=start_pose_in_kuka_lbr_r_link_0.p.y();
    z_start=start_pose_in_kuka_lbr_r_link_0.p.z();
    start_pose_in_kuka_lbr_r_link_0.M.GetRPY(roll_start,pitch_start, yaw_start);

    std::cout<<"x_start " << x_start<<std::endl;
    std::cout<<"y_start " << y_start <<std::endl;
    std::cout<<"z_start " << z_start<<std::endl;

    std::cout<<"roll_start " <<roll_start <<std::endl;
    std::cout<<"pitch_start" <<pitch_start <<std::endl;
    std::cout<<"yaw_start" <<yaw_start <<std::endl;
    bool success;
    success=robot_model_ptr->ikSolverUsingIKFAST(start_pose_in_kuka_lbr_r_link_0,planningGroupName,vector_robot_free_joint_parameters,start_pose_ik_result);
    std::cout<<"ik success: " <<success<<std::endl;


/////////////////////////////////////////////////////////////////////goal pose ////////////////////////////////////////////////////////////////////////


    goal_joint_values_for_given_pose["kuka_lbr_r_joint_1"]=-1.50;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_2"]=1.61;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_3"]=0.08;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_4"]=-0.33;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_5"]=-1.47;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_6"]=0.50;
    goal_joint_values_for_given_pose["kuka_lbr_r_joint_7"]=0.00;

    robot_model_ptr->updateJointGroup(goal_joint_values_for_given_pose);
    goal_pose_in_base_link= robot_model_ptr->getRobotState().robot_links["kuka_lbr_r_link_7"].getLinkFrame();
    robot_model_ptr->ConvertPoseBetweenFrames(B_Frame,goal_pose_in_base_link,A_Frame,goal_pose_in_kuka_lbr_r_link_0);

    x_goal=goal_pose_in_kuka_lbr_r_link_0.p.x();
    y_goal=goal_pose_in_kuka_lbr_r_link_0.p.y();
    z_goal=goal_pose_in_kuka_lbr_r_link_0.p.z();
    goal_pose_in_kuka_lbr_r_link_0.M.GetRPY(roll_goal,pitch_goal, yaw_goal);

    std::cout<<"x_goal " << x_goal<<std::endl;
    std::cout<<"y_goal " << y_goal <<std::endl;
    std::cout<<"z_goal " << z_goal<<std::endl;

    std::cout<<"roll_goal " <<roll_goal<<std::endl;
    std::cout<<"pitch_goal" <<pitch_goal<<std::endl;
    std::cout<<"yaw_goal" <<yaw_goal<<std::endl;





    success=robot_model_ptr->ikSolverUsingIKFAST(goal_pose_in_kuka_lbr_r_link_0,planningGroupName,vector_robot_free_joint_parameters,goal_pose_ik_result);
    std::cout<<"ik success: " <<success<<std::endl;




    MotionPlanningRequest motion_planning_request(start_pose_in_kuka_lbr_r_link_0,goal_pose_in_kuka_lbr_r_link_0,robot_model_ptr,planningGroupName,vector_robot_free_joint_parameters);
    MotionPlanningResponse motion_planning_response;
    OrientationConstraint orientation_constraint;

    double epsilon=0.1;

    orientation_constraint.setRollTolerance(+2*M_PI+epsilon ,-2*M_PI -epsilon);
    orientation_constraint.setPitchTolerance(+2*M_PI/2+epsilon,-2*M_PI-epsilon);
    orientation_constraint.setYawTolerance(+2*M_PI+epsilon,-2*M_PI-epsilon  );

    orientation_constraint.setXAxisTolerance(2,-2);
    orientation_constraint.setYAxisTolerance(2,-2);
    orientation_constraint.setZAxisTolerance(2,-2);
    motion_planning_request.setOrientationConstraint(orientation_constraint);


    PlannersType type_of_planner= RRTConnect;
    double max_solve_time=100000000;
    unsigned int max_step_smoothing=2;


    std::map<std::string, std::string> planner_specific_parameters_kv;
    planner_specific_parameters_kv["range"]="1.0";


    Planner robot_motion_planner(motion_planning_request,  type_of_planner, max_solve_time, max_step_smoothing,planner_specific_parameters_kv);
////////////////////////////////////////////////////////////////////////Solving and publishing ////////////////////////////////////////////////////////////////

    if( robot_motion_planner.Solve() )
    {


        motion_planning_response=robot_motion_planner.getMotionPlanningResponse();
        std::vector<std::map<std::string, double> > pathJointValues =motion_planning_response.getPathJointValues() ;

        std::map<std::string, double> pathJointValue_entry;

        sleep_time.sleep();
        for(std::size_t i=0;i<pathJointValues.size();i++)
        {
            std::string tmp_keyboard_press = "";
            std::cout << "Please enter to continue, executing joint command number :"<<i << std::endl;
            getline(std::cin, tmp_keyboard_press);
            sleep_time.sleep();
            pathJointValue_entry=pathJointValues.at(i);
            sensor_msgs::JointState arbitrary_joints_values;
            for(std::map<std::string, double>::iterator it=pathJointValue_entry.begin(); it!=pathJointValue_entry.end(); it++   )
            {
                std::cout<<it->first<<":"<<it->second <<std::endl;
                arbitrary_joints_values.name.push_back(it->first);
                arbitrary_joints_values.position.push_back(it->second);
            }

            joint_state_from_source_list_publisher.publish<sensor_msgs::JointState>(arbitrary_joints_values);
            std::cout<<"---------------------------" <<std::endl;
        }
    }
    else
    {
        std::cout<<"planning task failed" <<std::endl;
    }
/**/
}

void convertMeshtoEDT(int argc, char ** argv)
{
    //reading dude.stl and creating pointcloud

    std::string abs_path_to_mesh_file;
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    double scale_for_mesha_files_x, scale_for_mesha_files_y, scale_for_mesha_files_z;
    abs_path_to_mesh_file="/home/behnam/Desktop/omplapp-1.1.0-Source/resources/3D/Apartment_robot.stl";
    scale_for_mesha_files_x=1.0;
    scale_for_mesha_files_y=1.0;
    scale_for_mesha_files_z=1.0;
    Eigen::Affine3f target_frame=Eigen::Affine3f::Identity();
    createPointCloudFromMesh(abs_path_to_mesh_file, pcl_point_cloud, scale_for_mesha_files_x,scale_for_mesha_files_y, scale_for_mesha_files_z,target_frame);







    octomap::Pointcloud octomapCloud;
    convertPCLpointcloudToOctomapPointCloud<pcl::PointXYZ>( pcl_point_cloud, octomapCloud);
    octomap::OcTree octomap_Octree(0.01);
    convertOctomapPointCloudToOctomaptree(octomapCloud, octomap_Octree);
    octomap_Octree.writeBinary( "Apartment_robot.bt");

/////////////////////////////////////////////////////////////////////// Setting up the planner  ////////////////////////////////////////////////////////////

    argv[1]= "Apartment_robot.bt";
    visualize_EDT(2, argv);
//    octomap::OcTree *octomap_Octree = NULL;
//    octomap_Octree = new octomap::OcTree(0.01);
//    //read in octotree
//    octomap_Octree->readBinary(argv[1]);


//    double x_min,y_min,z_min, x_max,y_max,z_max;

//    x_min=-2;
//    y_min=-2;
//    z_min=0;

//    x_max=2;
//    y_max=0;
//    z_max=2;


}

int main (int argc, char ** argv)
{
    //calculate_manipulability_forward_kinematic(argc,argv);
    //test_planning( argc, argv);
//    optimisation_based(argc, argv);

    //visualize_EDT( argc, argv);
    optimisation_based_single_tree( argc, argv);
////    test_fk_right_arm_kuka( argc, argv);

////    test_orientation_constraint_joint_space(argc,argv );

//    convertMeshtoEDT(argc,argv );
    return 0;
}
