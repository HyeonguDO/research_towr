// Generated by gencpp from file xpp_msgs/StateLin3d.msg
// DO NOT EDIT!


#ifndef XPP_MSGS_MESSAGE_STATELIN3D_H
#define XPP_MSGS_MESSAGE_STATELIN3D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace xpp_msgs
{
template <class ContainerAllocator>
struct StateLin3d_
{
  typedef StateLin3d_<ContainerAllocator> Type;

  StateLin3d_()
    : pos()
    , vel()
    , acc()  {
    }
  StateLin3d_(const ContainerAllocator& _alloc)
    : pos(_alloc)
    , vel(_alloc)
    , acc(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pos_type;
  _pos_type pos;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _vel_type;
  _vel_type vel;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acc_type;
  _acc_type acc;





  typedef boost::shared_ptr< ::xpp_msgs::StateLin3d_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xpp_msgs::StateLin3d_<ContainerAllocator> const> ConstPtr;

}; // struct StateLin3d_

typedef ::xpp_msgs::StateLin3d_<std::allocator<void> > StateLin3d;

typedef boost::shared_ptr< ::xpp_msgs::StateLin3d > StateLin3dPtr;
typedef boost::shared_ptr< ::xpp_msgs::StateLin3d const> StateLin3dConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xpp_msgs::StateLin3d_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xpp_msgs::StateLin3d_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xpp_msgs::StateLin3d_<ContainerAllocator1> & lhs, const ::xpp_msgs::StateLin3d_<ContainerAllocator2> & rhs)
{
  return lhs.pos == rhs.pos &&
    lhs.vel == rhs.vel &&
    lhs.acc == rhs.acc;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xpp_msgs::StateLin3d_<ContainerAllocator1> & lhs, const ::xpp_msgs::StateLin3d_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xpp_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xpp_msgs::StateLin3d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xpp_msgs::StateLin3d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xpp_msgs::StateLin3d_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c4069b8f5d3058377f8685efad96ae30";
  }

  static const char* value(const ::xpp_msgs::StateLin3d_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc4069b8f5d305837ULL;
  static const uint64_t static_value2 = 0x7f8685efad96ae30ULL;
};

template<class ContainerAllocator>
struct DataType< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xpp_msgs/StateLin3d";
  }

  static const char* value(const ::xpp_msgs::StateLin3d_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This contains the 3D representation of a linear state, including:\n"
"# position, velocity, acceleration\n"
"\n"
"geometry_msgs/Point pos\n"
"geometry_msgs/Vector3 vel\n"
"geometry_msgs/Vector3 acc\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::xpp_msgs::StateLin3d_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pos);
      stream.next(m.vel);
      stream.next(m.acc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StateLin3d_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xpp_msgs::StateLin3d_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xpp_msgs::StateLin3d_<ContainerAllocator>& v)
  {
    s << indent << "pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pos);
    s << indent << "vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vel);
    s << indent << "acc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XPP_MSGS_MESSAGE_STATELIN3D_H
