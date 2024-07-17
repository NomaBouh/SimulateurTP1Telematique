#ifndef _GENERAL_DATA_BUFFER_H_
#define _GENERAL_DATA_BUFFER_H_

#include <algorithm>
#include <cstdint>
#include <vector>

#include "Utils.h"

class DynamicDataBuffer
{
    uint32_t m_size;
    uint8_t* m_data;
public:
    DynamicDataBuffer();
    DynamicDataBuffer(uint32_t dataSize);
    DynamicDataBuffer(uint32_t dataSize, const uint8_t* data);
    DynamicDataBuffer(const DynamicDataBuffer& other);
    DynamicDataBuffer(DynamicDataBuffer&& other);
    ~DynamicDataBuffer();
    void push_back(uint8_t data);

    DynamicDataBuffer& operator=(DynamicDataBuffer&& other);
    DynamicDataBuffer& operator=(const DynamicDataBuffer& other);

    bool operator==(const DynamicDataBuffer& other) const;
    bool operator!=(const DynamicDataBuffer& other) const;

    template<typename T>
    uint32_t write(const T& data)
    {
        return write((uint32_t)SizeOf<T>::data(data), reinterpret_cast<const uint8_t*>(&data), 0);
    }

    template<typename T>
    uint32_t write(const T& data, uint32_t start)
    {
        return write((uint32_t)SizeOf<T>::data(data), reinterpret_cast<const uint8_t*>(&data), start);
    }

    uint32_t write(uint32_t count, const uint8_t* data)
    {
        return write(count, data, 0);
    }

    uint32_t write(uint32_t count, const uint8_t* data, uint32_t start)
    {
        uint32_t end = std::min(m_size, count + start);
        for (uint32_t i = start; i < end; ++i)
        {
            m_data[i] = data[i - start];
        }
        return end;
    }

    template<typename T>
    T read(uint32_t start = 0) const
    {
        return *reinterpret_cast<T*>(((void*)&(m_data[start])));
    }

    void readTo(uint8_t* data, uint32_t start, uint32_t size)
    {
        for (uint32_t i = 0; i < size; ++i)
        {
            data[i] = m_data[start + i];
        }
    }

    void replaceData(const uint8_t* newData, uint32_t newDataSize, uint32_t start = 0);
    void replaceData(const DynamicDataBuffer& buffer, uint32_t start = 0);

    uint8_t operator[](size_t index) const;
    uint8_t& operator[](size_t index);

    uint8_t* data();
    const uint8_t* data() const;

    uint32_t size() const;

private:
    std::vector<uint8_t> buffer;
};

#endif //_GENERAL_DATA_BUFFER_H_
