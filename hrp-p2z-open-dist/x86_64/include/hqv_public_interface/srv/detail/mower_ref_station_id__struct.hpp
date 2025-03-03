// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/MowerRefStationId.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_ID__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_ID__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MowerRefStationId_Request_
{
  using Type = MowerRefStationId_Request_<ContainerAllocator>;

  explicit MowerRefStationId_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit MowerRefStationId_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Request
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Request
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerRefStationId_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerRefStationId_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerRefStationId_Request_

// alias to use template instance with default allocator
using MowerRefStationId_Request =
  hqv_public_interface::srv::MowerRefStationId_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MowerRefStationId_Response_
{
  using Type = MowerRefStationId_Response_<ContainerAllocator>;

  explicit MowerRefStationId_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->station_id = 0ul;
    }
  }

  explicit MowerRefStationId_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->station_id = 0ul;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;
  using _station_id_type =
    uint32_t;
  _station_id_type station_id;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__station_id(
    const uint32_t & _arg)
  {
    this->station_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Response
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationId_Response
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationId_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerRefStationId_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->station_id != other.station_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerRefStationId_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerRefStationId_Response_

// alias to use template instance with default allocator
using MowerRefStationId_Response =
  hqv_public_interface::srv::MowerRefStationId_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct MowerRefStationId
{
  using Request = hqv_public_interface::srv::MowerRefStationId_Request;
  using Response = hqv_public_interface::srv::MowerRefStationId_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_ID__STRUCT_HPP_
