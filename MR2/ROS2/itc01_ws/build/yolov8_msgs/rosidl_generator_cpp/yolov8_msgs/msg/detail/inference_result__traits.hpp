// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolov8_msgs:msg/InferenceResult.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_MSGS__MSG__DETAIL__INFERENCE_RESULT__TRAITS_HPP_
#define YOLOV8_MSGS__MSG__DETAIL__INFERENCE_RESULT__TRAITS_HPP_

#include "yolov8_msgs/msg/detail/inference_result__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolov8_msgs::msg::InferenceResult>()
{
  return "yolov8_msgs::msg::InferenceResult";
}

template<>
inline const char * name<yolov8_msgs::msg::InferenceResult>()
{
  return "yolov8_msgs/msg/InferenceResult";
}

template<>
struct has_fixed_size<yolov8_msgs::msg::InferenceResult>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolov8_msgs::msg::InferenceResult>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolov8_msgs::msg::InferenceResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLOV8_MSGS__MSG__DETAIL__INFERENCE_RESULT__TRAITS_HPP_
