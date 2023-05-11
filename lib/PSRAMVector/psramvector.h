#pragma once
#ifndef _PSRAMVECTOR_H_
#define _PSRAMVECTOR_H_

#include <Arduino.h>
#include <vector>

template<typename T>
class PsramAllocator {
public:
  using value_type = T;

  PsramAllocator() noexcept {}

  template<typename U>
  PsramAllocator(const PsramAllocator<U>&) noexcept {}

  T* allocate(std::size_t n) {
    return static_cast<T*>(ps_malloc(n * sizeof(T)));
  }

  void deallocate(T* p, std::size_t n) noexcept {
    free(p);
  }
};

template<typename T, typename U>
bool operator==(const PsramAllocator<T>&, const PsramAllocator<U>&) noexcept {
  return true;
}

template<typename T, typename U>
bool operator!=(const PsramAllocator<T>&, const PsramAllocator<U>&) noexcept {
  return false;
}

template<typename T>
using psvec = std::vector<T, PsramAllocator<T>>;

#endif
