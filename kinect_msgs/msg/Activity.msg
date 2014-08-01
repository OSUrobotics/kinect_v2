uint8 EYELEFTCLOSED  = 0
uint8 EYERIGHTCLOSED = 1
uint8 MOUTHOPEN      = 2
uint8 MOUTHMOVED     = 3
uint8 LOOKINGAWAY    = 4
uint8 COUNT          = 5

Header header
uint32 id
kinect_msgs/DetectionResult[] activities
