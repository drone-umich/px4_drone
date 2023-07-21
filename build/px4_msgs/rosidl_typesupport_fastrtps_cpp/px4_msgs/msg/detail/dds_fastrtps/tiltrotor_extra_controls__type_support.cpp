// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/TiltrotorExtraControls.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/tiltrotor_extra_controls__rosidl_typesupport_fastrtps_cpp.hpp"
#include "px4_msgs/msg/detail/tiltrotor_extra_controls__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace px4_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_serialize(
  const px4_msgs::msg::TiltrotorExtraControls & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: collective_tilt_normalized_setpoint
  cdr << ros_message.collective_tilt_normalized_setpoint;
  // Member: collective_thrust_normalized_setpoint
  cdr << ros_message.collective_thrust_normalized_setpoint;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs::msg::TiltrotorExtraControls & ros_message)
{
  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: collective_tilt_normalized_setpoint
  cdr >> ros_message.collective_tilt_normalized_setpoint;

  // Member: collective_thrust_normalized_setpoint
  cdr >> ros_message.collective_thrust_normalized_setpoint;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
get_serialized_size(
  const px4_msgs::msg::TiltrotorExtraControls & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: collective_tilt_normalized_setpoint
  {
    size_t item_size = sizeof(ros_message.collective_tilt_normalized_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: collective_thrust_normalized_setpoint
  {
    size_t item_size = sizeof(ros_message.collective_thrust_normalized_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
max_serialized_size_TiltrotorExtraControls(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: collective_tilt_normalized_setpoint
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: collective_thrust_normalized_setpoint
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _TiltrotorExtraControls__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::TiltrotorExtraControls *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TiltrotorExtraControls__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<px4_msgs::msg::TiltrotorExtraControls *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TiltrotorExtraControls__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::TiltrotorExtraControls *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TiltrotorExtraControls__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TiltrotorExtraControls(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TiltrotorExtraControls__callbacks = {
  "px4_msgs::msg",
  "TiltrotorExtraControls",
  _TiltrotorExtraControls__cdr_serialize,
  _TiltrotorExtraControls__cdr_deserialize,
  _TiltrotorExtraControls__get_serialized_size,
  _TiltrotorExtraControls__max_serialized_size
};

static rosidl_message_type_support_t _TiltrotorExtraControls__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TiltrotorExtraControls__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace px4_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_px4_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::TiltrotorExtraControls>()
{
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_TiltrotorExtraControls__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4_msgs, msg, TiltrotorExtraControls)() {
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_TiltrotorExtraControls__handle;
}

#ifdef __cplusplus
}
#endif
