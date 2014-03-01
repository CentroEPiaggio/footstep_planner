# Contains all the information from one run of the plane detection node

# The information for the plane that has been detected
Plane plane
#Plane[] plane

# Whether the detection has succeeded or failed
int32 NO_CLOUD_RECEIVED = 1
int32 NO_PLANES = 2
int32 OTHER_ERROR = 3
int32 SUCCESS = 4
int32 result