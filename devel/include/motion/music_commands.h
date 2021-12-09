// Generated by gencpp from file motion/music_commands.msg
// DO NOT EDIT!


#ifndef MOTION_MESSAGE_MUSIC_COMMANDS_H
#define MOTION_MESSAGE_MUSIC_COMMANDS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motion
{
template <class ContainerAllocator>
struct music_commands_
{
  typedef music_commands_<ContainerAllocator> Type;

  music_commands_()
    : tempo()
    , right_arm_motions()
    , left_arm_motions()  {
    }
  music_commands_(const ContainerAllocator& _alloc)
    : tempo(_alloc)
    , right_arm_motions(_alloc)
    , left_arm_motions(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _tempo_type;
  _tempo_type tempo;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _right_arm_motions_type;
  _right_arm_motions_type right_arm_motions;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _left_arm_motions_type;
  _left_arm_motions_type left_arm_motions;





  typedef boost::shared_ptr< ::motion::music_commands_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motion::music_commands_<ContainerAllocator> const> ConstPtr;

}; // struct music_commands_

typedef ::motion::music_commands_<std::allocator<void> > music_commands;

typedef boost::shared_ptr< ::motion::music_commands > music_commandsPtr;
typedef boost::shared_ptr< ::motion::music_commands const> music_commandsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motion::music_commands_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motion::music_commands_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motion

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'motion': ['/home/cc/ee106a/fl21/class/ee106a-abw/ros_workspaces/robot_conductor_ryu/src/motion/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motion::music_commands_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motion::music_commands_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motion::music_commands_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motion::music_commands_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motion::music_commands_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motion::music_commands_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motion::music_commands_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ed9308da9714234f1e05e7985bf4f8c3";
  }

  static const char* value(const ::motion::music_commands_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xed9308da9714234fULL;
  static const uint64_t static_value2 = 0x1e05e7985bf4f8c3ULL;
};

template<class ContainerAllocator>
struct DataType< ::motion::music_commands_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motion/music_commands";
  }

  static const char* value(const ::motion::music_commands_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motion::music_commands_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] tempo\n\
string[] right_arm_motions\n\
string[] left_arm_motions\n\
";
  }

  static const char* value(const ::motion::music_commands_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motion::music_commands_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tempo);
      stream.next(m.right_arm_motions);
      stream.next(m.left_arm_motions);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct music_commands_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motion::music_commands_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motion::music_commands_<ContainerAllocator>& v)
  {
    s << indent << "tempo[]" << std::endl;
    for (size_t i = 0; i < v.tempo.size(); ++i)
    {
      s << indent << "  tempo[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tempo[i]);
    }
    s << indent << "right_arm_motions[]" << std::endl;
    for (size_t i = 0; i < v.right_arm_motions.size(); ++i)
    {
      s << indent << "  right_arm_motions[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.right_arm_motions[i]);
    }
    s << indent << "left_arm_motions[]" << std::endl;
    for (size_t i = 0; i < v.left_arm_motions.size(); ++i)
    {
      s << indent << "  left_arm_motions[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.left_arm_motions[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTION_MESSAGE_MUSIC_COMMANDS_H