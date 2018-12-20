# dvs_hot_pixel_filter

Sometimes, you have a rosbag that contains hot pixels that fire many noise events.
This package automatically detects hot pixels and creates a new rosbag without hot pixel events.

To use it:

        rosrun dvs_hot_pixel_filter dvs_hot_pixel_filter path_to_input.bag 


A new rosbag will be written, name ```input.bag.filtered```. It is an exact copy of the original bag, except hot pixel events have been removed.

