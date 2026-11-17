// Copyright 2026 TIER IV, Inc.
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

/// @file
/// Clang thread safety annotations, see https://clang.llvm.org/docs/ThreadSafetyAnalysis.html.
/// They are only active with Clang and a standard library whose mutexes are annotated (libc++),
/// and expand to nothing otherwise.

#if defined(__clang__) && defined(_LIBCPP_HAS_THREAD_SAFETY_ANNOTATIONS)
#define NEBULA_TSA_ATTRIBUTE(x) __attribute__((x))
#else
#define NEBULA_TSA_ATTRIBUTE(x)
#endif

/// The annotated member may only be accessed while holding the given mutex.
#define NEBULA_TSA_GUARDED_BY(x) NEBULA_TSA_ATTRIBUTE(guarded_by(x))

/// The annotated function may only be called while holding the given mutexes.
#define NEBULA_TSA_REQUIRES(...) NEBULA_TSA_ATTRIBUTE(requires_capability(__VA_ARGS__))
