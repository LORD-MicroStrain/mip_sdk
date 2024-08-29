#pragma once

#include <mip/mip_result.hpp>
#include <mip/mip_descriptors.hpp>
#include <mip/extras/descriptor_id.hpp>
#include <mip/mip_interface.hpp>


class SettingBase
{
public:
    virtual mip::CmdResult apply(mip::Interface& device) = 0;
    virtual mip::CmdResult read (mip::Interface& device) = 0;
    virtual mip::CmdResult save (mip::Interface& device) = 0;
    virtual mip::CmdResult load (mip::Interface& device) = 0;
    virtual mip::CmdResult reset(mip::Interface& device) = 0;

    virtual mip::DescriptorId id() const = 0;

    virtual void print() const = 0;

    mip::CompositeDescriptor descriptor() const { return id().descriptor(); }

    bool applied = false;
    bool saved   = false;
};


template<class MipCmd, uint32_t AdditionalTime=0>
class Setting : public SettingBase, public MipCmd
{
public:
    Setting() = default;

    template<typename... Args>
    Setting(Args... args) : MipCmd{mip::FunctionSelector::WRITE, args...} {}

    using MipCmd::MipCmd;

    mip::DescriptorId id() const override {
        return MipCmd::DESCRIPTOR;
    }

    void print() const override
    {
        mip::DescriptorId index = id();
        std::printf("(%02X,%02X)[%d] = TBD\n", index.descriptor().descriptorSet, index.descriptor().fieldDescriptor, index.index());
    }

public:
    mip::CmdResult apply(mip::Interface& device) override
    {
        this->function = mip::FunctionSelector::WRITE;
        mip::CmdResult result = mip::runCommand(device, *this, AdditionalTime);
        if(result)
        {
            this->applied = true;
            this->saved = false;
        }
        return result;
    }
    mip::CmdResult read(mip::Interface& device) override
    {
        this->function = mip::FunctionSelector::READ;
        mip::CmdResult result = mip::runCommand(device, *this, AdditionalTime);
        if(result)
            this->applied = true;
        return result;
    }
    mip::CmdResult save(mip::Interface& device) override
    {
        this->function = mip::FunctionSelector::SAVE;
        typename MipCmd::Response rsp;
        mip::CmdResult result = mip::runCommand(device, *this, rsp, AdditionalTime);
        //*this = rsp;
        if(result)
            this->saved = true;
        return result;
    }
    mip::CmdResult load(mip::Interface& device) override
    {
        this->function = mip::FunctionSelector::LOAD;
        mip::CmdResult result = mip::runCommand(device, *this, AdditionalTime);
        if(!result)
            return result;
        this->applied = false;
        // this->saved = true;
        return read(device);
    }
    mip::CmdResult reset(mip::Interface& device) override
    {
        this->function = mip::FunctionSelector::RESET;
        mip::CmdResult result = mip::runCommand(device, *this, AdditionalTime);
        if(!result)
            return result;
        this->applied = false;
        return read(device);
    }
};

template<class MipCmd, uint32_t AdditionalTime=0>
class MultiSetting : public Setting<MipCmd, AdditionalTime>
{
public:
    template<class... Args>
    MultiSetting(uint16_t instance_, Args... args) : Setting<MipCmd, AdditionalTime>(args...), instance(instance_) {}

    mip::DescriptorId id() const override {
        return { MipCmd::DESCRIPTOR, this->instance };
    }

    uint16_t instance = 0;
};

