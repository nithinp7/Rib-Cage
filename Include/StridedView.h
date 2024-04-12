#pragma once

#include <vector>
#include <memory>

namespace RibCage {

template<typename _TView>
struct StridedView {
  template<typename _TStruct>
  StridedView(const _TStruct* pStart, size_t offset, size_t count) 
    : m_pStart(reinterpret_cast<const std::byte*>(pStart))
    , m_offset(offset)
    , m_stride(sizeof(_TStruct))
    , m_count(count) {}

  const _TView& operator[](size_t idx) const {
    const std::byte* ptr = m_pStart + m_offset + idx * m_stride;
    return *reinterpret_cast<const _TView*>(ptr);
  }

  size_t getCount() const { return m_count; }

private:
  const std::byte* m_pStart;
  size_t m_offset;
  size_t m_stride;
  size_t m_count;
};
} // namespace RibCage