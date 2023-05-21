/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <UwbPlugin.h>

namespace gazebo
{

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(UwbPlugin)

    /// Constructor
    UwbPlugin::UwbPlugin() 
    : ModelPlugin(), sequence(0)
    {
        this->updatePeriod = common::Time(0.0);
    }

    /// Destructor
    UwbPlugin::~UwbPlugin()
    {
    }

    /// Load function 
    void UwbPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Configure the plugin from the SDF file
        node = gazebo_ros::Node::Get(_sdf);

        const gazebo_ros::QoS & qos = node->get_qos();

        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), (std::string("A ROS node for Gazebo has not been initialized, unable to load plugin. ")
                                            + std::string("Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)")).c_str());
            return;
        }

        if (!_sdf->HasElement("update_rate"))
        {
            RCLCPP_ERROR(node->get_logger(), "UWB Plugin needs the parameter: update_rate");
        }

        this->model = _parent;
        this->world = _parent->GetWorld();
        this->SetUpdateRate(_sdf->Get<double>("update_rate"));
        this->nlosSoftWallWidth = 0.25;
        this->tagZOffset = 0;
        this->tagId = std::string("0x00");
        this->maxDBDistance = 14;
        this->stepDBDistance = 0.1;
        this->allBeaconsAreLOS = false;
        this->useParentAsReference = false;


        if (_sdf->HasElement("all_los"))
        {
            this->allBeaconsAreLOS = _sdf->Get<double>("all_los");
        }

        if (_sdf->HasElement("tag_id"))
        {
            this->tagId = _sdf->Get<std::string>("tag_id");
        }

        if (_sdf->HasElement("tag_z_offset"))
        {
            this->tagZOffset = _sdf->Get<double>("tag_z_offset");
        }

        if (_sdf->HasElement("nlosSoftWallWidth"))
        {
            this->nlosSoftWallWidth = _sdf->Get<double>("nlosSoftWallWidth");
        }

        if (_sdf->HasElement("tag_link"))
        {
            std::string tag_link = _sdf->Get<std::string>("tag_link");
            this->tagLink = _parent->GetLink(tag_link);


            if (this->tagLink == NULL)
            {
                RCLCPP_INFO(node->get_logger(), "Parent name: %s ChildCount: %d", _parent->GetName().c_str(), _parent->GetChildCount());
                std::vector<physics::LinkPtr> links = _parent->GetLinks();
                for (int i = 0; i < links.size(); ++i)
                {
                    RCLCPP_INFO(node->get_logger(), "Link[%d]: %s", i, links[i]->GetName().c_str());
                }
                RCLCPP_INFO(node->get_logger(), "UWB Plugin Tag link Is NULL We use The Parent As Reference");
                this->useParentAsReference = true;
            } else {
                RCLCPP_INFO(node->get_logger(), "Parent name: %s tag_link: %s", _parent->GetName().c_str(), this->tagLink->GetName().c_str());
            }
        }

        if (_sdf->HasElement("anchor_prefix"))
        {
            this->anchorPrefix = _sdf->Get<std::string>("anchor_prefix");
        }
        else
        {
            this->anchorPrefix = "uwb_anchor";
        }

        // RCLCPP_INFO(node->get_logger(), "UWB Plugin is running. Tag %s", this->tagId.c_str());
        // RCLCPP_INFO(node->get_logger(), "UWB Plugin All parameters loaded");

        this->lastUpdateTime = common::Time(0.0);

        std::string topicRanging = "/uwb_ranging";
        std::string topicAnchors = "/uwb_anchors";
  
        this->Anchors_Pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(topicAnchors, qos.get_publisher_qos(topicAnchors, rclcpp::SensorDataQoS().reliable()));     
        this->Uwb_Pub = node->create_publisher<rosmsgs::msg::Ranging>(topicRanging, qos.get_publisher_qos(topicRanging, rclcpp::SensorDataQoS().reliable()));

        RCLCPP_INFO(node->get_logger(), "Publishing on topic [%s]", this->Uwb_Pub->get_topic_name());
        RCLCPP_INFO(node->get_logger(), "Publishing on topic [%s]", this->Anchors_Pub->get_topic_name());

        this->firstRay = boost::dynamic_pointer_cast<physics::RayShape>(
                                this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));

        this->secondRay = boost::dynamic_pointer_cast<physics::RayShape>(
                                this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));

        this->updateConnection =
            event::Events::ConnectWorldUpdateBegin(boost::bind(&UwbPlugin::OnUpdate, this, _1));
    }

    void UwbPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
        common::Time simTime = _info.simTime;
        common::Time elapsed = simTime - this->lastUpdateTime;
        if (elapsed >= this->updatePeriod)
        {
            this->lastUpdateTime = _info.simTime;

            /// Pose of the tag
            ignition::math::Pose3d tagPose;

            if (!this->useParentAsReference)
            {
                tagPose = this->tagLink->WorldPose();
            }
            else
            {
                tagPose = this->model->WorldPose();
            }

            ignition::math::Vector3d posCorrectedZ(tagPose.Pos().X(), tagPose.Pos().Y(), tagPose.Pos().Z() + this->tagZOffset);
            tagPose.Set(posCorrectedZ, tagPose.Rot());
            ignition::math::Vector3d currentTagPose(tagPose.Pos());

            tf2::Quaternion q(tagPose.Rot().X(),
                              tagPose.Rot().Y(),
                              tagPose.Rot().Z(),
                              tagPose.Rot().W());

            tf2::Matrix3x3 m(q);
            double roll, pitch, currentYaw;
            m.getRPY(roll, pitch, currentYaw);

            // if (currentYaw < 0)
            // {
            //     currentYaw = 2 * M_PI + currentYaw;
            // }

            double startAngle = currentYaw;
            double currentAngle = 0;
            double arc = 3 * M_PI / 2;
            int numAnglesToTestBySide = 30;
            double incrementAngle = arc / numAnglesToTestBySide;
            int totalNumberAnglesToTest = 1 + 2 * numAnglesToTestBySide;
            double anglesToTest[totalNumberAnglesToTest];

            anglesToTest[0] = startAngle;
            for (int i = 1; i < totalNumberAnglesToTest; ++i)
            {
                double angleToTest;
                if (i % 2 == 0)
                {
                    angleToTest = startAngle - (i / 2) * incrementAngle;
                    // if (angleToTest < 0)
                    // {
                    //     angleToTest = 2 * M_PI + angleToTest;
                    // }
                }
                else
                {
                    angleToTest = startAngle + (i - (i - 1) / 2) * incrementAngle;
                    // if (angleToTest > 2 * M_PI)
                    // {
                    //     angleToTest = angleToTest - 2 * M_PI;
                    // }
                }
                anglesToTest[i] = angleToTest;
            }

            visualization_msgs::msg::MarkerArray markerArray;
            visualization_msgs::msg::MarkerArray interferencesArray;

            int anchor_i = 0;

            physics::Model_V models = this->world->Models();
            for (physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)
            {

                if ((*iter)->GetName().find(this->anchorPrefix) == 0)
                {
                    physics::ModelPtr anchor = *iter;
                    std::string aidStr = anchor->GetName().substr(this->anchorPrefix.length());
                    int aid = std::stoi(aidStr);
                    ignition::math::Pose3d anchorPose = anchor->WorldPose();

                    LOSType losType = LOS;
                    double distance = tagPose.Pos().Distance(anchorPose.Pos());
                    double distanceAfterRebounds = 0;

                    if (!allBeaconsAreLOS)
                    {
                        //We check if a ray can reach the anchor:
                        double distanceToObstacleFromTag;
                        std::string obstacleName;

                        ignition::math::Vector3d directionToAnchor = (anchorPose.Pos() - tagPose.Pos()).Normalize();
                        this->firstRay->Reset();
                        this->firstRay->SetPoints(tagPose.Pos(), anchorPose.Pos());
                        this->firstRay->GetIntersection(distanceToObstacleFromTag, obstacleName);

                        if (obstacleName.compare("") == 0)
                        {
                            //There is no obstacle between anchor and tag, we use the LOS model
                            losType = LOS;
                            distanceAfterRebounds = distance;
                        }
                        else
                        {

                            //We use a second ray to measure the distance from anchor to tag, so we can
                            //know what is the width of the walls
                            double distanceToObstacleFromAnchor;
                            std::string otherObstacleName;

                            this->secondRay->Reset();
                            this->secondRay->SetPoints(anchorPose.Pos(), tagPose.Pos());
                            this->secondRay->GetIntersection(distanceToObstacleFromAnchor, otherObstacleName);

                            double wallWidth = distance - distanceToObstacleFromTag - distanceToObstacleFromAnchor;

                            if (wallWidth <= this->nlosSoftWallWidth && obstacleName.compare(otherObstacleName) == 0)
                            {
                                //We use NLOS - SOFT model
                                losType = NLOS_S;
                                distanceAfterRebounds = distance;
                            }
                            else
                            {
                                //We try to find a rebound to reach the anchor from the tag
                                bool end = false;

                                double maxDistance = 30;
                                double distanceToRebound = 0;
                                double distanceToFinalObstacle = 0;
                                double distanceNlosHard = 0;

                                double stepFloor = 1;
                                double startFloorDistanceCheck = 2;
                                int numStepsFloor = 6;


                                std::string finalObstacleName;
                                int indexRay = 0;
                                bool foundNlosH = false;

                                int currentFloorDistance = 0;

                                while (!end)
                                {

                                    currentAngle = anglesToTest[indexRay];


                                    double x = currentTagPose.X() + maxDistance * cos(currentAngle);
                                    double y = currentTagPose.Y() + maxDistance * sin(currentAngle);
                                    double z = currentTagPose.Z();

                                    if (currentFloorDistance>0){
                                        double tanAngleFloor = (startFloorDistanceCheck + stepFloor*(currentFloorDistance-1))/currentTagPose.Z();
                                        double angleFloor = atan(tanAngleFloor);

                                        double h = sin(angleFloor)*maxDistance;

                                        double horizontalDistance = sqrt(maxDistance*maxDistance - h*h);

                                        x = currentTagPose.X() + horizontalDistance * cos(currentAngle);
                                        y = currentTagPose.Y() + horizontalDistance * sin(currentAngle);

                                        z = -1*(h - currentTagPose.Z());

                                    }

                                    ignition::math::Vector3d rayPoint(x, y, z);

                                    this->firstRay->Reset();
                                    this->firstRay->SetPoints(currentTagPose, rayPoint);
                                    this->firstRay->GetIntersection(distanceToRebound, obstacleName);

                                    if (obstacleName.compare("") != 0)
                                    {
                                        ignition::math::Vector3d collisionPoint(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), currentTagPose.Z());

                                        if (currentFloorDistance>0){
                                            //if (obstacleName.compare("FloorStatic)") == 0){
                                                // RCLCPP_INFO(node->get_logger(), "TOUCHED GROUND %s - Z: %f", obstacleName.c_str(), z);   
                                            //}
                                            
                                            collisionPoint.Set(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), 0.0);
                                        }

                                        //We try to reach the anchor from here
                                        this->secondRay->Reset();
                                        this->secondRay->SetPoints(collisionPoint, anchorPose.Pos());
                                        this->secondRay->GetIntersection(distanceToFinalObstacle, finalObstacleName);

                                        if (finalObstacleName.compare("") == 0)
                                        {



                                            //We reach the anchor after one rebound
                                            distanceToFinalObstacle = anchorPose.Pos().Distance(collisionPoint);

                                            if (currentFloorDistance>0 ){
                                                    //RCLCPP_INFO(node->get_logger(), "Rebound in GROUND %s - Distance: %f", obstacleName.c_str(), distanceToFinalObstacle);   
                                            }


                                            if (distanceToRebound + distanceToFinalObstacle <= maxDBDistance)
                                            {
                                                foundNlosH = true;
                                                //We try to find the shortest rebound
                                                if (distanceNlosHard < 0.1)
                                                {
                                                    distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                }
                                                else if (distanceNlosHard > distanceToRebound + distanceToFinalObstacle)
                                                {
                                                    distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                }
                                            }
                                        }
                                    }

                                    if (indexRay < totalNumberAnglesToTest - 1)
                                    {
                                        indexRay += 1;
                                    }
                                    else
                                    {
                                        if (currentFloorDistance<numStepsFloor){

                                        currentFloorDistance+=1;
                                        indexRay= 0;

                                        } else {
                                            end = true;  
                                        }

                                    }
                                }

                                if (foundNlosH)
                                {
                                    //We use the NLOS Hard Model with distance = distanceNlosHard
                                    losType = NLOS_H;
                                    distanceAfterRebounds = distanceNlosHard;
                                }
                                else
                                {
                                    //We can not reach the anchor, no ranging.
                                    losType = NLOS;
                                }
                            }
                        }

                    }
                    else
                    {
                        //All beacons are LOS
                        losType = LOS;
                        distanceAfterRebounds = distance;
                    }

                    if ((losType == LOS || losType == NLOS_S) && distanceAfterRebounds > maxDBDistance)
                    {
                        losType = NLOS;
                    }

                    if (losType == NLOS_H && distanceAfterRebounds > maxDBDistance)
                    {
                        losType = NLOS;
                    }

                    if (losType != NLOS)
                    {

                        int indexScenario = 0;
                        if (losType == NLOS_S)
                        {
                            indexScenario = 2;
                        }
                        else if (losType == NLOS_H)
                        {
                            indexScenario = 1;
                        }

                        int indexRangingOffset = (int) round(distanceAfterRebounds / stepDBDistance);

                        double distanceAfterReboundsWithOffset = distanceAfterRebounds;
                        if (losType == LOS)
                        {
                            distanceAfterReboundsWithOffset = distanceAfterRebounds + rangingOffset[indexRangingOffset][0] / 1000.0;
                        }
                        else if (losType == NLOS_S)
                        {
                            distanceAfterReboundsWithOffset = distanceAfterRebounds + rangingOffset[indexRangingOffset][1] / 1000.0;
                        }

                        int indexRanging = (int) round(distanceAfterReboundsWithOffset / stepDBDistance);


                        std::normal_distribution<double> distributionRanging(distanceAfterReboundsWithOffset * 1000, rangingStd[indexRanging][indexScenario]);
                        std::normal_distribution<double> distributionRss(rssMean[indexRanging][indexScenario], rssStd[indexRanging][indexScenario]);

                        double rangingValue = distributionRanging(this->random_generator);
                        double powerValue = distributionRss(this->random_generator);

                        if (powerValue < minPower[indexScenario])
                        {
                            losType = NLOS;
                        }
                        

                        if (losType!=NLOS)
                        {
                            rosmsgs::msg::Ranging ranging_msg;
                            ranging_msg.anchor_id = aidStr;
                            ranging_msg.tag_id = this->tagId;
                            ranging_msg.range = rangingValue;
                            ranging_msg.seq = this->sequence;
                            ranging_msg.rss = powerValue;
                            ranging_msg.error_estimation = 0.00393973;
                            this->Uwb_Pub->publish(ranging_msg);
                        }
                    }

                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "world";
                    marker.header.stamp = node->now();
                    marker.id = anchor_i;
                    anchor_i++;
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.lifetime.sec = 5;
                    marker.pose.position.x = anchorPose.Pos().X();
                    marker.pose.position.y = anchorPose.Pos().Y();
                    marker.pose.position.z = anchorPose.Pos().Z();
                    marker.pose.orientation.x = anchorPose.Rot().X();
                    marker.pose.orientation.y = anchorPose.Rot().Y();
                    marker.pose.orientation.z = anchorPose.Rot().Z();
                    marker.pose.orientation.w = anchorPose.Rot().W();
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.2;
                    marker.scale.z = 0.5;
                    marker.color.a = 1.0;

                    if (losType == LOS)
                    {
                        marker.color.r = 0.0;
                        marker.color.g = 0.6;
                        marker.color.b = 0.0;
                    }
                    else if (losType == NLOS_S)
                    {
                        marker.color.r = 0.6;
                        marker.color.g = 0.6;
                        marker.color.b = 0.0;
                    }
                    else if (losType == NLOS_H)
                    {
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.6;
                    }
                    else if (losType == NLOS)
                    {
                        marker.color.r = 0.6;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                    }

                    markerArray.markers.push_back(marker);
                }
            }

            this->Anchors_Pub->publish(markerArray);
            this->sequence++;
        }
    }

    /// Set update rate
    void UwbPlugin::SetUpdateRate(double _rate)
    {
        if (_rate > 0.0)
        {
            this->updatePeriod = 1.0 / _rate;
        }
        else
        {
            this->updatePeriod = 0.0;
        }
    }

    /// Reset function
    void UwbPlugin::Reset()
    {
        RCLCPP_INFO(node->get_logger(), "UWB Plugin RESET");
        this->lastUpdateTime = common::Time(0.0);
    }

} // namespace gazebo
