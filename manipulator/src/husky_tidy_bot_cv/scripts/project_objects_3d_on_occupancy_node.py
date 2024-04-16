import rospy
import message_filters
from nav_msgs.msg import OccupancyGrid
from husky_tidy_bot_cv.msg import Objects3d
import numpy as np


def project_objects_3d_on_occupancy(objects_3d_msg: Objects3d, grid_msg: OccupancyGrid):
    global grid_pub

    assert objects_3d_msg.header.frame_id == grid_msg.header.frame_id, \
        "Objects and map are from different frames"

    assert grid_msg.info.origin.orientation.x == 0, "Do not support rotated occupancy grid"
    assert grid_msg.info.origin.orientation.y == 0, "Do not support rotated occupancy grid"
    assert grid_msg.info.origin.orientation.z == 0, "Do not support rotated occupancy grid"
    assert grid_msg.info.origin.orientation.w == 1, "Do not support rotated occupancy grid"

    width = grid_msg.info.width
    height = grid_msg.info.height
    grid = np.array(grid_msg.data, dtype=np.int8).reshape(height, width)

    shift_x = grid_msg.info.origin.position.x
    shift_y = grid_msg.info.origin.position.y
    resolution = grid_msg.info.resolution
    for position in objects_3d_msg.positions:
        x = position.pose.position.x
        y = position.pose.position.y
        x -= shift_x
        y -= shift_y
        i = int(y // resolution)
        j = int(x // resolution)
        if i < 0 or i >= height or j < 0 or j >= width:  # object is out of grid map
            continue
        grid[i, j] = 100

    out_grid_msg = OccupancyGrid()
    out_grid_msg.header = grid_msg.header
    out_grid_msg.info = grid_msg.info
    out_grid_msg.data = grid.ravel().tolist()

    grid_pub.publish(out_grid_msg)


if __name__ == '__main__':
    rospy.init_node("project_objects_3d_on_occupancy")

    grid_pub = rospy.Publisher("/grid_with_objects", OccupancyGrid, queue_size=10)

    objects_3d_sub = message_filters.Subscriber("/tracked_objects_3d", Objects3d)
    grid_sub = message_filters.Subscriber("/occupancy_grid_map/grid_map", OccupancyGrid)
    sync_sub = message_filters.ApproximateTimeSynchronizer(
        [objects_3d_sub, grid_sub], 10, 0.1, allow_headerless=True)
    sync_sub.registerCallback(project_objects_3d_on_occupancy)

    print("Spinning...")
    rospy.spin()
