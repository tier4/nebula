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

// Public NEBULA_* / ALLOCATE_OUTPUT_* macro surface. The expansions reference types from the
// sibling headers, so include nebula_agnocast_wrapper.hpp rather than this file alone.

#include <memory>
#include <type_traits>

#ifdef USE_AGNOCAST_ENABLED

#define NEBULA_MESSAGE_UNIQUE_PTR(MessageT) \
  nebula::agnocast_wrapper::message_ptr<MessageT, nebula::agnocast_wrapper::OwnershipType::Unique>
// For publisher (mutable message)
#define NEBULA_MESSAGE_SHARED_PTR(MessageT) \
  nebula::agnocast_wrapper::message_ptr<MessageT, nebula::agnocast_wrapper::OwnershipType::Shared>
// For subscription (read-only message)
#define NEBULA_MESSAGE_CONST_SHARED_PTR(MessageT) \
  nebula::agnocast_wrapper::message_ptr<          \
    const MessageT, nebula::agnocast_wrapper::OwnershipType::Shared>
#define NEBULA_SERVER_REQUEST_PTR(ServiceT) \
  nebula::agnocast_wrapper::message_ptr<    \
    const typename ServiceT::Request, nebula::agnocast_wrapper::OwnershipType::Shared>
#define NEBULA_SERVER_RESPONSE_PTR(ServiceT) \
  nebula::agnocast_wrapper::message_ptr<     \
    typename ServiceT::Response, nebula::agnocast_wrapper::OwnershipType::Shared>
#define NEBULA_CLIENT_REQUEST_PTR(ServiceT) \
  nebula::agnocast_wrapper::message_ptr<    \
    typename ServiceT::Request, nebula::agnocast_wrapper::OwnershipType::Shared>
// The client hands the response over as a plain std::shared_ptr in both builds: the agnocast
// backend aliases the received handle, so nothing is copied.
#define NEBULA_CLIENT_RESPONSE_PTR(ServiceT) std::shared_ptr<const typename ServiceT::Response>
#define NEBULA_SUBSCRIPTION_PTR(MessageT) \
  typename nebula::agnocast_wrapper::Subscription<MessageT>::SharedPtr
#define NEBULA_PUBLISHER_PTR(MessageT) \
  typename nebula::agnocast_wrapper::Publisher<MessageT>::SharedPtr
#define NEBULA_GENERIC_SUBSCRIPTION_PTR nebula::agnocast_wrapper::GenericSubscription::SharedPtr
#define NEBULA_GENERIC_PUBLISHER_PTR nebula::agnocast_wrapper::GenericPublisher::SharedPtr
#define NEBULA_CLIENT_PTR(ServiceT) typename nebula::agnocast_wrapper::Client<ServiceT>::SharedPtr
#define NEBULA_SERVICE_PTR(ServiceT) typename nebula::agnocast_wrapper::Service<ServiceT>::SharedPtr
#define NEBULA_CLIENT_FUTURE(ServiceT) typename nebula::agnocast_wrapper::Client<ServiceT>::Future
#define NEBULA_CLIENT_SHARED_FUTURE(ServiceT) \
  typename nebula::agnocast_wrapper::Client<ServiceT>::SharedFuture
#define NEBULA_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename nebula::agnocast_wrapper::Client<ServiceT>::FutureAndRequestId
#define NEBULA_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename nebula::agnocast_wrapper::Client<ServiceT>::SharedFutureAndRequestId
#define NEBULA_TIMER_PTR nebula::agnocast_wrapper::Timer::SharedPtr

#define NEBULA_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  nebula::agnocast_wrapper::create_subscription<message_type>(this, topic, qos, callback, options)
#define NEBULA_CREATE_SUBSCRIPTION_ON_NODE(message_type, node, topic, qos, callback, options) \
  nebula::agnocast_wrapper::create_subscription<message_type>(node, topic, qos, callback, options)

#define NEBULA_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  nebula::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2)
#define NEBULA_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  nebula::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2, arg3)
#define NEBULA_CREATE_PUBLISHER2_ON_NODE(message_type, node, arg1, arg2) \
  nebula::agnocast_wrapper::create_publisher<message_type>(node, arg1, arg2)
#define NEBULA_CREATE_PUBLISHER3_ON_NODE(message_type, node, arg1, arg2, arg3) \
  nebula::agnocast_wrapper::create_publisher<message_type>(node, arg1, arg2, arg3)

// Method 1 (macro + free function) entry points for the generic (type-erased) publisher and
// subscription: mirrors NEBULA_CREATE_PUBLISHER2/3 and NEBULA_CREATE_SUBSCRIPTION above, but
// with a runtime topic_type string instead of a compile-time message_type.
#define NEBULA_CREATE_GENERIC_PUBLISHER3(topic, topic_type, qos) \
  nebula::agnocast_wrapper::create_generic_publisher(this, topic, topic_type, qos)
#define NEBULA_CREATE_GENERIC_PUBLISHER4(topic, topic_type, qos, options) \
  nebula::agnocast_wrapper::create_generic_publisher(this, topic, topic_type, qos, options)
#define NEBULA_CREATE_GENERIC_PUBLISHER3_ON_NODE(node, topic, topic_type, qos) \
  nebula::agnocast_wrapper::create_generic_publisher(node, topic, topic_type, qos)
#define NEBULA_CREATE_GENERIC_PUBLISHER4_ON_NODE(node, topic, topic_type, qos, options) \
  nebula::agnocast_wrapper::create_generic_publisher(node, topic, topic_type, qos, options)

#define NEBULA_CREATE_GENERIC_SUBSCRIPTION(topic, topic_type, qos, callback, options) \
  nebula::agnocast_wrapper::create_generic_subscription(                              \
    this, topic, topic_type, qos, callback, options)
#define NEBULA_CREATE_GENERIC_SUBSCRIPTION_ON_NODE(      \
  node, topic, topic_type, qos, callback, options)       \
  nebula::agnocast_wrapper::create_generic_subscription( \
    node, topic, topic_type, qos, callback, options)

#define NEBULA_CREATE_CLIENT1(service_type, service_name) \
  nebula::agnocast_wrapper::create_client<service_type>(this, service_name)
#define NEBULA_CREATE_CLIENT2(service_type, service_name, qos) \
  nebula::agnocast_wrapper::create_client<service_type>(this, service_name, qos)
#define NEBULA_CREATE_CLIENT3(service_type, service_name, qos, group) \
  nebula::agnocast_wrapper::create_client<service_type>(this, service_name, qos, group)
#define NEBULA_CREATE_CLIENT1_ON_NODE(service_type, node, service_name) \
  nebula::agnocast_wrapper::create_client<service_type>(node, service_name)
#define NEBULA_CREATE_CLIENT2_ON_NODE(service_type, node, service_name, qos) \
  nebula::agnocast_wrapper::create_client<service_type>(node, service_name, qos)
#define NEBULA_CREATE_CLIENT3_ON_NODE(service_type, node, service_name, qos, group) \
  nebula::agnocast_wrapper::create_client<service_type>(node, service_name, qos, group)

#define NEBULA_CREATE_SERVICE2(service_type, service_name, callback) \
  nebula::agnocast_wrapper::create_service<service_type>(this, service_name, callback)
#define NEBULA_CREATE_SERVICE3(service_type, service_name, callback, qos) \
  nebula::agnocast_wrapper::create_service<service_type>(this, service_name, callback, qos)
#define NEBULA_CREATE_SERVICE4(service_type, service_name, callback, qos, group) \
  nebula::agnocast_wrapper::create_service<service_type>(this, service_name, callback, qos, group)
#define NEBULA_CREATE_SERVICE2_ON_NODE(service_type, node, service_name, callback) \
  nebula::agnocast_wrapper::create_service<service_type>(node, service_name, callback)
#define NEBULA_CREATE_SERVICE3_ON_NODE(service_type, node, service_name, callback, qos) \
  nebula::agnocast_wrapper::create_service<service_type>(node, service_name, callback, qos)
#define NEBULA_CREATE_SERVICE4_ON_NODE(service_type, node, service_name, callback, qos, group) \
  nebula::agnocast_wrapper::create_service<service_type>(node, service_name, callback, qos, group)

#define NEBULA_SUBSCRIPTION_OPTIONS agnocast::SubscriptionOptions
#define NEBULA_PUBLISHER_OPTIONS agnocast::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) publisher->allocate_output_message_unique()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) publisher->allocate_output_message_shared()
#define ALLOCATE_OUTPUT_SERVICE_REQUEST(client) client->allocate_output_service_request()

#else

#define NEBULA_MESSAGE_UNIQUE_PTR(MessageT) std::unique_ptr<MessageT>

// For publisher (mutable message)
#define NEBULA_MESSAGE_SHARED_PTR(MessageT) std::shared_ptr<MessageT>
// For subscription (read-only message)
#define NEBULA_MESSAGE_CONST_SHARED_PTR(MessageT) std::shared_ptr<const MessageT>
#define NEBULA_SERVER_REQUEST_PTR(ServiceT) std::shared_ptr<const typename ServiceT::Request>
#define NEBULA_SERVER_RESPONSE_PTR(ServiceT) std::shared_ptr<typename ServiceT::Response>
#define NEBULA_CLIENT_REQUEST_PTR(ServiceT) std::shared_ptr<typename ServiceT::Request>
#define NEBULA_CLIENT_RESPONSE_PTR(ServiceT) std::shared_ptr<const typename ServiceT::Response>
#define NEBULA_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define NEBULA_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr
#define NEBULA_GENERIC_SUBSCRIPTION_PTR rclcpp::GenericSubscription::SharedPtr
#define NEBULA_GENERIC_PUBLISHER_PTR rclcpp::GenericPublisher::SharedPtr
#define NEBULA_CLIENT_PTR(ServiceT) typename nebula::agnocast_wrapper::Client<ServiceT>::SharedPtr
#define NEBULA_SERVICE_PTR(ServiceT) typename nebula::agnocast_wrapper::Service<ServiceT>::SharedPtr
#define NEBULA_CLIENT_FUTURE(ServiceT) typename nebula::agnocast_wrapper::Client<ServiceT>::Future
#define NEBULA_CLIENT_SHARED_FUTURE(ServiceT) \
  typename nebula::agnocast_wrapper::Client<ServiceT>::SharedFuture
#define NEBULA_CLIENT_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename nebula::agnocast_wrapper::Client<ServiceT>::FutureAndRequestId
#define NEBULA_CLIENT_SHARED_FUTURE_AND_REQUEST_ID(ServiceT) \
  typename nebula::agnocast_wrapper::Client<ServiceT>::SharedFutureAndRequestId
#define NEBULA_TIMER_PTR rclcpp::TimerBase::SharedPtr

#define NEBULA_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  this->create_subscription<message_type>(topic, qos, callback, options)
#define NEBULA_CREATE_SUBSCRIPTION_ON_NODE(message_type, node, topic, qos, callback, options) \
  (node)->create_subscription<message_type>(topic, qos, callback, options)

#define NEBULA_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  this->create_publisher<message_type>(arg1, arg2)
#define NEBULA_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  this->create_publisher<message_type>(arg1, arg2, arg3)
#define NEBULA_CREATE_PUBLISHER2_ON_NODE(message_type, node, arg1, arg2) \
  (node)->create_publisher<message_type>(arg1, arg2)
#define NEBULA_CREATE_PUBLISHER3_ON_NODE(message_type, node, arg1, arg2, arg3) \
  (node)->create_publisher<message_type>(arg1, arg2, arg3)

// Method 1 (macro + free function) entry points for the generic (type-erased) publisher and
// subscription. Unlike NEBULA_CREATE_PUBLISHER2/3 above, these do NOT forward to `this`'s own
// native create_generic_publisher()/create_generic_subscription() directly: rclcpp's generic
// create_*() silently drops qos_overriding_options while the Agnocast-enabled build's rejects it,
// so calling through the wrapper free function here — which checks first — keeps the two builds
// behaving the same way instead of reopening that inconsistency at the macro's own call site (see
// create_generic_publisher()/create_generic_subscription() in generic_publisher.hpp/
// generic_subscription.hpp).
#define NEBULA_CREATE_GENERIC_PUBLISHER3(topic, topic_type, qos) \
  nebula::agnocast_wrapper::create_generic_publisher(this, topic, topic_type, qos)
#define NEBULA_CREATE_GENERIC_PUBLISHER4(topic, topic_type, qos, options) \
  nebula::agnocast_wrapper::create_generic_publisher(this, topic, topic_type, qos, options)
#define NEBULA_CREATE_GENERIC_PUBLISHER3_ON_NODE(node, topic, topic_type, qos) \
  nebula::agnocast_wrapper::create_generic_publisher(node, topic, topic_type, qos)
#define NEBULA_CREATE_GENERIC_PUBLISHER4_ON_NODE(node, topic, topic_type, qos, options) \
  nebula::agnocast_wrapper::create_generic_publisher(node, topic, topic_type, qos, options)

#define NEBULA_CREATE_GENERIC_SUBSCRIPTION(topic, topic_type, qos, callback, options) \
  nebula::agnocast_wrapper::create_generic_subscription(                              \
    this, topic, topic_type, qos, callback, options)
#define NEBULA_CREATE_GENERIC_SUBSCRIPTION_ON_NODE(      \
  node, topic, topic_type, qos, callback, options)       \
  nebula::agnocast_wrapper::create_generic_subscription( \
    node, topic, topic_type, qos, callback, options)

#define NEBULA_CREATE_CLIENT1(service_type, service_name) \
  nebula::agnocast_wrapper::create_client<service_type>(this, service_name)
#define NEBULA_CREATE_CLIENT2(service_type, service_name, qos) \
  nebula::agnocast_wrapper::create_client<service_type>(this, service_name, qos)
#define NEBULA_CREATE_CLIENT3(service_type, service_name, qos, group) \
  nebula::agnocast_wrapper::create_client<service_type>(this, service_name, qos, group)
#define NEBULA_CREATE_CLIENT1_ON_NODE(service_type, node, service_name) \
  nebula::agnocast_wrapper::create_client<service_type>(node, service_name)
#define NEBULA_CREATE_CLIENT2_ON_NODE(service_type, node, service_name, qos) \
  nebula::agnocast_wrapper::create_client<service_type>(node, service_name, qos)
#define NEBULA_CREATE_CLIENT3_ON_NODE(service_type, node, service_name, qos, group) \
  nebula::agnocast_wrapper::create_client<service_type>(node, service_name, qos, group)

#define NEBULA_CREATE_SERVICE2(service_type, service_name, callback) \
  nebula::agnocast_wrapper::create_service<service_type>(this, service_name, callback)
#define NEBULA_CREATE_SERVICE3(service_type, service_name, callback, qos) \
  nebula::agnocast_wrapper::create_service<service_type>(this, service_name, callback, qos)
#define NEBULA_CREATE_SERVICE4(service_type, service_name, callback, qos, group) \
  nebula::agnocast_wrapper::create_service<service_type>(this, service_name, callback, qos, group)
#define NEBULA_CREATE_SERVICE2_ON_NODE(service_type, node, service_name, callback) \
  nebula::agnocast_wrapper::create_service<service_type>(node, service_name, callback)
#define NEBULA_CREATE_SERVICE3_ON_NODE(service_type, node, service_name, callback, qos) \
  nebula::agnocast_wrapper::create_service<service_type>(node, service_name, callback, qos)
#define NEBULA_CREATE_SERVICE4_ON_NODE(service_type, node, service_name, callback, qos, group) \
  nebula::agnocast_wrapper::create_service<service_type>(node, service_name, callback, qos, group)

#define NEBULA_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define NEBULA_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) \
  std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) \
  std::make_shared<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_SERVICE_REQUEST(client) client->allocate_output_service_request()

#endif
