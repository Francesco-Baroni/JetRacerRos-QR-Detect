#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include <math.h>

double const PI = 3.14159265358979323846;

class QrDetector
{
public:
    QrDetector()
    {
        ros::NodeHandle nh("~");

        nh.param("camera_name", camera_name_, std::string("csi_cam_0"));
        nh.param("topic_name", topic_name_, std::string("qr_detector"));
        nh.param("qr_dim", qr_dim_, 0.1);

        image_sub_ = nh.subscribe("/" + camera_name_ + "/image_raw", 1, &QrDetector::callbackImage, this);
        laser_sub_ = nh.subscribe("/scan", 1, &QrDetector::callbackLaser, this);
        image_pub_ = nh.advertise<sensor_msgs::Image>(topic_name_, 10);
        qr_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name_ + "/markers", 10);

        cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_name_ + "/camera_info");
        scan_data_ = nullptr;
    }

private:
    std::string camera_name_;
    std::string topic_name_;
    double qr_dim_;
    tf::TransformListener listener_;
    sensor_msgs::CameraInfo::ConstPtr cam_info_;
    sensor_msgs::LaserScan::ConstPtr scan_data_;
    visualization_msgs::MarkerArray rooms_;
    std::map<std::string, std::vector<double>> detected_;

    ros::Subscriber image_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher image_pub_;
    ros::Publisher qr_pub_;

    void callbackLaser(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        scan_data_ = msg;
    }

    void callbackImage(const sensor_msgs::Image::ConstPtr &data)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            // Convert to grayscale
            cv::Mat frame_gray;
            cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

            // Initialize zbar scanner
            zbar::ImageScanner scanner;
            scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

            // Wrap OpenCV Mat data for zbar
            zbar::Image image(frame_gray.cols, frame_gray.rows, "Y800", frame_gray.data, frame_gray.cols * frame_gray.rows);

            // Scan the image for QR codes
            int n = scanner.scan(image);

            for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
            {
                std::string retval = symbol->get_data();
                if (!retval.empty())
                {
                    std::vector<cv::Point> points;
                    for (int i = 0; i < symbol->get_location_size(); i++)
                    {
                        points.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
                    }

                    // Calculate the center of the QR code
                    int cx = 0, cy = 0;
                    for (const auto &point : points)
                    {
                        cx += point.x;
                        cy += point.y;
                    }
                    cx /= points.size();
                    cy /= points.size();

                    // Draw QR code outline and text
                    for (size_t i = 0; i < points.size(); ++i)
                    {
                        cv::line(frame, points[i], points[(i + 1) % points.size()], cv::Scalar(0, 255, 0), 4);
                    }
                    cv::putText(frame, retval, cv::Point(cx, cy - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);

                    // Calculate QR code position and estimate distance
                    double distance_lim = 1.5; // Limit distance to 1.5m
                    double cx_qr = (points[0].x+points[1].x) / 2;
                    double cy_qr = (points[0].y+points[1].y) / 2;
                    double cx_qr_norm = (cx_qr - cam_info_->K[2]) / cam_info_->K[0];
                    double cy_qr_norm = (cy_qr - cam_info_->K[5]) / cam_info_->K[4];
                    double distance = (qr_dim_ * cam_info_->K[0]) / abs(points[0].x - points[1].x);
                    distance = distance > distance_lim ? distance_lim : distance;
                    double x = cx_qr_norm * distance;
                    double y = cy_qr_norm * distance;

                    // Use the laser scan data to get the real distance to the QR code
                    // scan data ranges start from -x in the laser_feame (anti-clockwise)
                    if (scan_data_ != nullptr)
                    {
                        double angle = atan(x/distance);
                        int index = fmod((2*PI-angle),(2*PI)) / scan_data_->angle_increment;
                        double distance_laser = scan_data_->ranges[index];
                        if (distance_laser == std::numeric_limits<float>::infinity())
                        {
                            distance_laser = distance_lim;
                        }
                        distance = distance_laser;
                    }

                    try
                    {
                        geometry_msgs::PointStamped point, transformed_point;
                        point.header.stamp = ros::Time();
                        point.header.frame_id = "/laser_frame";
                        point.point.x = -distance;
                        point.point.y = x;
                        //point.point.z = -y;
                        point.point.z = 0;

                        listener_.transformPoint("/map", point, transformed_point);

                        if (detected_.find(retval) == detected_.end()) // New QR code detected
                        {
                            ROS_INFO("QR code detected: %s", retval.c_str());
                            // Create marker for text
                            visualization_msgs::Marker marker_txt;
                            marker_txt.header.frame_id = "/map";
                            marker_txt.header.stamp = ros::Time::now();
                            marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                            marker_txt.text = retval;
                            marker_txt.id = rooms_.markers.size();
                            marker_txt.action = visualization_msgs::Marker::ADD;
                            marker_txt.scale.z = 0.2;
                            marker_txt.color.a = 1.0;
                            marker_txt.color.g = 1.0;
                            marker_txt.pose.position.x = transformed_point.point.x;
                            marker_txt.pose.position.y = transformed_point.point.y;
                            marker_txt.pose.position.z = transformed_point.point.z + 0.1;

                            rooms_.markers.push_back(marker_txt);

                            // Create marker for sphere
                            visualization_msgs::Marker marker_sphere;
                            marker_sphere.header.frame_id = "/map";
                            marker_sphere.header.stamp = ros::Time::now();
                            marker_sphere.type = visualization_msgs::Marker::SPHERE;
                            marker_sphere.id = rooms_.markers.size();
                            marker_sphere.action = visualization_msgs::Marker::ADD;
                            marker_sphere.scale.x = marker_sphere.scale.y = marker_sphere.scale.z = 0.1;
                            marker_sphere.color.a = 1.0;
                            marker_sphere.color.g = 1.0;
                            marker_sphere.pose.position.x = transformed_point.point.x;
                            marker_sphere.pose.position.y = transformed_point.point.y;
                            marker_sphere.pose.position.z = transformed_point.point.z;

                            rooms_.markers.push_back(marker_sphere);
                            detected_[retval] = {x, (double)(marker_txt.id), (double)(marker_sphere.id)};
                        }
                        else //if (detected_[retval][0] > x) // Update position if closer, better QR code pos. estimation
                        {
                            auto &text_marker = rooms_.markers[detected_[retval][1]];
                            auto &sphere_marker = rooms_.markers[detected_[retval][2]];

                            text_marker.pose.position.x = transformed_point.point.x;
                            text_marker.pose.position.y = transformed_point.point.y;
                            text_marker.pose.position.z = transformed_point.point.z + 0.1;

                            sphere_marker.pose.position.x = transformed_point.point.x;
                            sphere_marker.pose.position.y = transformed_point.point.y;
                            sphere_marker.pose.position.z = transformed_point.point.z;

                            detected_[retval][0] = x;
                        }
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_WARN("%s", ex.what());
                        return;
                    }
                }
            }

            // Publish results
            qr_pub_.publish(rooms_);
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(data->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
            image_pub_.publish(out_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qr_detector");
    QrDetector qr_detector;

    ros::spin();
    return 0;
}
