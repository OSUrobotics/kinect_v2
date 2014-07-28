/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/cameron/catkin_ws/src/kinect_msgs/msg/SkeletonArray.msg
 *
 */


#ifndef KINECT_MSGS_MESSAGE_SKELETONARRAY_H
#define KINECT_MSGS_MESSAGE_SKELETONARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include "Skeleton.h"

namespace kinect_msgs
{
template <class ContainerAllocator>
struct SkeletonArray_
{
  typedef SkeletonArray_<ContainerAllocator> Type;

  SkeletonArray_()
    : header()
    , bodies()  {
    }
  SkeletonArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , bodies(_alloc)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::kinect_msgs::Skeleton_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kinect_msgs::Skeleton_<ContainerAllocator> >::other >  _bodies_type;
  _bodies_type bodies;




  typedef boost::shared_ptr< ::kinect_msgs::SkeletonArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinect_msgs::SkeletonArray_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct SkeletonArray_

typedef ::kinect_msgs::SkeletonArray_<std::allocator<void> > SkeletonArray;

typedef boost::shared_ptr< ::kinect_msgs::SkeletonArray > SkeletonArrayPtr;
typedef boost::shared_ptr< ::kinect_msgs::SkeletonArray const> SkeletonArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinect_msgs::SkeletonArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace kinect_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'kinect_msgs': ['/home/cameron/catkin_ws/src/kinect_msgs/msg'], 'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/groovy/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinect_msgs::SkeletonArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinect_msgs::SkeletonArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinect_msgs::SkeletonArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6c9a8e8095d4269e29426066661db501";
  }

  static const char* value(const ::kinect_msgs::SkeletonArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6c9a8e8095d4269eULL;
  static const uint64_t static_value2 = 0x29426066661db501ULL;
};

template<class ContainerAllocator>
struct DataType< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinect_msgs/SkeletonArray";
  }

  static const char* value(const ::kinect_msgs::SkeletonArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
Skeleton[] bodies\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: kinect_msgs/Skeleton\n\
Header header\n\
uint32 id\n\
geometry_msgs/Pose[] joints\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::kinect_msgs::SkeletonArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.bodies);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct SkeletonArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinect_msgs::SkeletonArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinect_msgs::SkeletonArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "bodies[]" << std::endl;
    for (size_t i = 0; i < v.bodies.size(); ++i)
    {
      s << indent << "  bodies[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kinect_msgs::Skeleton_<ContainerAllocator> >::stream(s, indent + "    ", v.bodies[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINECT_MSGS_MESSAGE_SKELETONARRAY_H
