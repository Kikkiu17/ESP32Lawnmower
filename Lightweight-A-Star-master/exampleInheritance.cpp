#include <vector>
#include <cstdlib>
#include <cstdio>

template<typename T>
class PsramAllocator {
public:
  using value_type = T;

  PsramAllocator() noexcept : size_(0) {}

  PsramAllocator(std::size_t size) noexcept : size_(size) {}

  template<typename U>
  PsramAllocator(const PsramAllocator<U>& other) noexcept : size_(other.size()) {}

  T* allocate(std::size_t n) {
    return static_cast<T*>(malloc(n * sizeof(T)));
  }

  void deallocate(T* p, std::size_t n) noexcept {
    free(p);
  }

  std::size_t size() const noexcept {
    return size_;
  }

private:
  std::size_t size_;
};

template<typename T, typename U>
bool operator==(const PsramAllocator<T>& lhs, const PsramAllocator<U>& rhs) noexcept {
  return lhs.size() == rhs.size();
}

template<typename T, typename U>
bool operator!=(const PsramAllocator<T>& lhs, const PsramAllocator<U>& rhs) noexcept {
  return !(lhs == rhs);
}

template<typename T>
using psramvec = std::vector<T, PsramAllocator<T>>;

int main()
{
    psramvec<psramvec<int>> vettore;

    vettore.resize(10, psramvec<int>(15, 24));

    for (int i = 0; i < 10; i++)
    {
        printf("%d", vettore[0][i]);
    }

    return 0;
}
