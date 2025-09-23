// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rover_control:msg/DriveCommand.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__FUNCTIONS_H_
#define ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rover_control/msg/rosidl_generator_c__visibility_control.h"

#include "rover_control/msg/detail/drive_command__struct.h"

/// Initialize msg/DriveCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rover_control__msg__DriveCommand
 * )) before or use
 * rover_control__msg__DriveCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
bool
rover_control__msg__DriveCommand__init(rover_control__msg__DriveCommand * msg);

/// Finalize msg/DriveCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
void
rover_control__msg__DriveCommand__fini(rover_control__msg__DriveCommand * msg);

/// Create msg/DriveCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rover_control__msg__DriveCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
rover_control__msg__DriveCommand *
rover_control__msg__DriveCommand__create();

/// Destroy msg/DriveCommand message.
/**
 * It calls
 * rover_control__msg__DriveCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
void
rover_control__msg__DriveCommand__destroy(rover_control__msg__DriveCommand * msg);

/// Check for msg/DriveCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
bool
rover_control__msg__DriveCommand__are_equal(const rover_control__msg__DriveCommand * lhs, const rover_control__msg__DriveCommand * rhs);

/// Copy a msg/DriveCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
bool
rover_control__msg__DriveCommand__copy(
  const rover_control__msg__DriveCommand * input,
  rover_control__msg__DriveCommand * output);

/// Initialize array of msg/DriveCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * rover_control__msg__DriveCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
bool
rover_control__msg__DriveCommand__Sequence__init(rover_control__msg__DriveCommand__Sequence * array, size_t size);

/// Finalize array of msg/DriveCommand messages.
/**
 * It calls
 * rover_control__msg__DriveCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
void
rover_control__msg__DriveCommand__Sequence__fini(rover_control__msg__DriveCommand__Sequence * array);

/// Create array of msg/DriveCommand messages.
/**
 * It allocates the memory for the array and calls
 * rover_control__msg__DriveCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
rover_control__msg__DriveCommand__Sequence *
rover_control__msg__DriveCommand__Sequence__create(size_t size);

/// Destroy array of msg/DriveCommand messages.
/**
 * It calls
 * rover_control__msg__DriveCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
void
rover_control__msg__DriveCommand__Sequence__destroy(rover_control__msg__DriveCommand__Sequence * array);

/// Check for msg/DriveCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
bool
rover_control__msg__DriveCommand__Sequence__are_equal(const rover_control__msg__DriveCommand__Sequence * lhs, const rover_control__msg__DriveCommand__Sequence * rhs);

/// Copy an array of msg/DriveCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_control
bool
rover_control__msg__DriveCommand__Sequence__copy(
  const rover_control__msg__DriveCommand__Sequence * input,
  rover_control__msg__DriveCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__FUNCTIONS_H_
