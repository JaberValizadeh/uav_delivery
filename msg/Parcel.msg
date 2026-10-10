# uav_delivery/Parcel
#
# One box, with an address at each end.
#
# The two-ended-ness is the whole difference from the packages this grew out
# of. A treasure or a fire is one place the aircraft has to get to; a parcel is
# two, in order, and the second one only becomes reachable by doing something
# at the first. That is why this message carries two coordinate pairs and a
# state, rather than a position and a boolean.
#
# The named addresses are not decoration. Depots are fixed, surveyed places
# with names, so a task reads "C to F" rather than "(-2.47, 1.98) to
# (3.50, 0.00)", which is what makes a log of the run followable.
#
# carrier is the exclusivity guarantee, in one field: at most one drone
# namespace ever appears here for a given parcel, and it is set when the parcel
# is claimed rather than when it is picked up. Two aircraft cannot be sent to
# the same box.

uint8 WAITING   = 0     # on the ground at src, nobody assigned
uint8 CLAIMED   = 1     # assigned to `carrier`, who is on the way
uint8 CARRIED   = 2     # welded to `carrier`, in transit to dst
uint8 DELIVERED = 3     # confirmed on the ground at dst

uint32 id               # parcel number, unique for the life of the run
string box              # Gazebo model name of the box itself

string  src_name        # source depot, e.g. "C"
float64 src_x           # where the box was put down, world frame (m)
float64 src_y

string  dst_name        # destination depot, e.g. "F"
float64 dst_x           # where it has to end up (m)
float64 dst_y

uint8  state            # one of the constants above
string carrier          # drone namespace that owns this parcel, or ""
