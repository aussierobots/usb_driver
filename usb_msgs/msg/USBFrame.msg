# A message representing a single USB frame.
std_msgs/Header header
uint8 FRAME_XFER_TYPE_IN = 0
uint8 FRAME_XFER_TYPE_OUT = 1
uint8 frame_xfer_type    # Frame Transfer Type  - in or out
string encoding          # Encoding of the data in the frame
                         # from the list of strings in include/usb_msgs/usb_encodings.hpp
uint8[] data