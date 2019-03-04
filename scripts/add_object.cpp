// Collision tools
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Rviz visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_populater");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    const std::string PLANNING_GROUP = "leg_1";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("body");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
// Adding/Removing Objects and Attaching/Detaching Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "wall";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);
    // visual_tools.publishText(1, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    // Wait for MoveGroup to recieve and process the collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    return 1;
}