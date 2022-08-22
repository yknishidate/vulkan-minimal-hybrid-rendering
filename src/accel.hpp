#pragma once

#include "buffer.hpp"

using vkBU = vk::BufferUsageFlagBits;
using vkMP = vk::MemoryPropertyFlagBits;
using vkDT = vk::DescriptorType;

struct AccelerationStructure
{
    vk::UniqueAccelerationStructureKHR accel;
    Buffer buffer;

    vk::AccelerationStructureTypeKHR type;
    uint32_t primitiveCount;
    vk::DeviceSize size;
    vk::AccelerationStructureBuildGeometryInfoKHR geometryInfo;
    uint64_t deviceAddress;
    vk::WriteDescriptorSetAccelerationStructureKHR asInfo;

    AccelerationStructure() = default;

    AccelerationStructure(vk::AccelerationStructureGeometryKHR geometry,
                          vk::AccelerationStructureTypeKHR type,
                          uint32_t primitiveCount)
    {
        this->type = type;
        this->primitiveCount = primitiveCount;

        geometryInfo.setType(type);
        geometryInfo.setFlags(vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
        geometryInfo.setGeometries(geometry);

        vk::AccelerationStructureBuildSizesInfoKHR buildSizesInfo =
            Context::device.getAccelerationStructureBuildSizesKHR(vk::AccelerationStructureBuildTypeKHR::eDevice, geometryInfo, primitiveCount);
        size = buildSizesInfo.accelerationStructureSize;
        buffer = Buffer{ size, vkBU::eAccelerationStructureStorageKHR | vkBU::eShaderDeviceAddress, vkMP::eDeviceLocal };

        vk::AccelerationStructureCreateInfoKHR createInfo;
        createInfo.setBuffer(*buffer.buffer);
        createInfo.setSize(size);
        createInfo.setType(type);
        accel = Context::device.createAccelerationStructureKHRUnique(createInfo);

        Buffer scratchBuffer{ size, vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress, vkMP::eDeviceLocal };
        geometryInfo.setScratchData(scratchBuffer.deviceAddress);
        geometryInfo.setDstAccelerationStructure(*accel);

        vk::AccelerationStructureBuildRangeInfoKHR rangeInfo{ primitiveCount , 0, 0, 0 };
        Context::oneTimeSubmit(
            [&](vk::CommandBuffer commandBuffer) {
                commandBuffer.buildAccelerationStructuresKHR(geometryInfo, &rangeInfo);
            });
    }

    vk::WriteDescriptorSet createDescWrite(vk::DescriptorSet descSet, uint32_t binding)
    {
        asInfo = vk::WriteDescriptorSetAccelerationStructureKHR{ *accel };
        vk::WriteDescriptorSet asWrite;
        asWrite.setDstSet(descSet);
        asWrite.setDescriptorType(vkDT::eAccelerationStructureKHR);
        asWrite.setDescriptorCount(1);
        asWrite.setDstBinding(binding);
        asWrite.setPNext(&asInfo);
        return asWrite;
    }
};
