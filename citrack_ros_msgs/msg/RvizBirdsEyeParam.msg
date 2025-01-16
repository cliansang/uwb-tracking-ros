std_msgs/Header header

# Size of the BirdEye image in px
int32 height
int32 width

geometry_msgs/Point offset # Pixel from upper left corner to the origin of the car coordinate system. (z=0)
float64 resolution # Resolution of the bird's-eye view image in px / m.