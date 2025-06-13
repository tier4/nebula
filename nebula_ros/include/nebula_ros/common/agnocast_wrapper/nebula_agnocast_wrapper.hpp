// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/agnocast.hpp>

#define NEBULA_MESSAGE_UNIQUE_PTR(MessageT) agnocast::ipc_shared_ptr<MessageT>
#define NEBULA_MESSAGE_SHARED_PTR(MessageT) agnocast::ipc_shared_ptr<MessageT>
#define NEBULA_SUBSCRIPTION_PTR(MessageT) typename agnocast::Subscription<MessageT>::SharedPtr
#define NEBULA_PUBLISHER_PTR(MessageT) typename agnocast::Publisher<MessageT>::SharedPtr

#define NEBULA_CREATE_SUBSCRIPTION(message_type, node_ptr, topic, qos, callback, options) \
  agnocast::create_subscription<message_type>(node_ptr, topic, qos, callback, options)
#define NEBULA_CREATE_PUBLISHER2(message_type, node_ptr, arg1, arg2) \
  agnocast::create_publisher<message_type>(node_ptr, arg1, arg2)
#define NEBULA_CREATE_PUBLISHER3(message_type, node_ptr, arg1, arg2, arg3) \
  agnocast::create_publisher<message_type>(node_ptr, arg1, arg2, arg3)

#define NEBULA_SUBSCRIPTION_OPTIONS agnocast::SubscriptionOptions
#define NEBULA_PUBLISHER_OPTIONS agnocast::PublisherOptions

#define NEBULA_HAS_ANY_SUBSCRIPTIONS(publisher) (publisher->get_subscription_count() > 0)

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) publisher->borrow_loaned_message()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) publisher->borrow_loaned_message()

#else

#include <rclcpp/rclcpp.hpp>

#define NEBULA_MESSAGE_UNIQUE_PTR(MessageT) std::unique_ptr<MessageT>
#define NEBULA_MESSAGE_SHARED_PTR(MessageT) std::shared_ptr<MessageT>
#define NEBULA_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define NEBULA_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr

#define NEBULA_CREATE_SUBSCRIPTION(message_type, node_ptr, topic, qos, callback, options) \
  (node_ptr)->create_subscription<message_type>(topic, qos, callback, options)
#define NEBULA_CREATE_PUBLISHER2(message_type, node_ptr, arg1, arg2) \
  (node_ptr)->create_publisher<message_type>(arg1, arg2)
#define NEBULA_CREATE_PUBLISHER3(message_type, node_ptr, arg1, arg2, arg3) \
  (node_ptr)->create_publisher<message_type>(arg1, arg2, arg3)

#define NEBULA_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define NEBULA_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define NEBULA_HAS_ANY_SUBSCRIPTIONS(publisher) \
  (publisher->get_subscription_count() > 0 || publisher->get_intra_process_subscription_count() > 0)

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) \
  std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) \
  std::make_shared<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()

#endif
