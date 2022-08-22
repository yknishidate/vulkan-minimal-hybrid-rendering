#pragma once

#include "context.hpp"

using vkBU = vk::BufferUsageFlagBits;
using vkMP = vk::MemoryPropertyFlagBits;
using vkDT = vk::DescriptorType;

struct Buffer
{
    vk::UniqueBuffer buffer;
    vk::UniqueDeviceMemory memory;
    vk::DeviceSize size;
    uint64_t deviceAddress;
    vk::DescriptorBufferInfo bufferInfo;
    void* mapped = nullptr;

    Buffer() = default;

    Buffer(vk::DeviceSize size, vk::BufferUsageFlags usage,
           vk::MemoryPropertyFlags properties)
        : size{ size }
    {
        buffer = Context::device.createBufferUnique({ {}, size, usage });

        vk::MemoryRequirements requirements = Context::device.getBufferMemoryRequirements(*buffer);
        uint32_t type = Context::findMemoryType(requirements.memoryTypeBits, properties);

        vk::MemoryAllocateInfo allocInfo{ requirements.size, type };
        if (usage & vk::BufferUsageFlagBits::eShaderDeviceAddress) {
            vk::MemoryAllocateFlagsInfo flagsInfo{ vk::MemoryAllocateFlagBits::eDeviceAddress };
            allocInfo.pNext = &flagsInfo;

            memory = Context::device.allocateMemoryUnique(allocInfo);
            Context::device.bindBufferMemory(*buffer, *memory, 0);

            vk::BufferDeviceAddressInfoKHR addressInfo{ *buffer };
            deviceAddress = Context::device.getBufferAddressKHR(&addressInfo);
        } else {
            memory = Context::device.allocateMemoryUnique(allocInfo);
            Context::device.bindBufferMemory(*buffer, *memory, 0);
        }
    }

    void copy(void* data)
    {
        if (!mapped) {
            mapped = Context::device.mapMemory(*memory, 0, size);
        }
        memcpy(mapped, data, size);
    }

    vk::WriteDescriptorSet createDescWrite(vk::DescriptorType type, uint32_t binding)
    {
        bufferInfo = vk::DescriptorBufferInfo{ *buffer, 0, size };
        vk::WriteDescriptorSet bufferWrite;
        bufferWrite.setDescriptorType(type);
        bufferWrite.setDescriptorCount(1);
        bufferWrite.setDstBinding(binding);
        bufferWrite.setBufferInfo(bufferInfo);
        return bufferWrite;
    }
};
