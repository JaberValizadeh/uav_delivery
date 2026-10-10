# uav_delivery/ParcelArray
#
# Every parcel in the system, published by the dispatcher.
#
# Includes delivered ones until their slot is reused, so a monitor or an RViz
# view can show what has just been completed rather than only what is
# outstanding.

std_msgs/Header header
Parcel[]         parcels
